"""Actual linked motor/display/radio application on the existing modeled bus.

No device access. Peripheral time advances at foreground boundaries and clock
calls; this is NOT a cycle-accurate WCET, RF or electrical model. Pending SysTick
and UART IRQs execute the real handlers only at unmasked instruction boundaries.
Application clocks, ADC, homing, radio and display functions are never replaced.
"""
import hashlib
import json
from pathlib import Path
import struct
import sys

import display_thumb as display

probe=display.probe
base=probe.base
TIM,ADC,PMU=probe.TIM,probe.ADC,probe.PMU
RCC,PA,PB,PC=probe.RCC,probe.PA,probe.PB,probe.PC
ST,ICSR=0xe000e010,0xe000ed04
PENDING=1<<26
SCENARIOS=("normal","traffic","held_button","bad_factory","adc_timeout",
           "timer_stall","missed_window","foreground_stall","cpu_stall",
           "compare_failure","adc_boundary","display_fault","radio_error","clock_fault",
           "temperature","temperature_traffic","temperature_repeat",
           "temperature_adc_timeout","temperature_bad_raw","temperature_display_fault",
           "temperature_radio_error","temperature_sensor_pin_fault","temperature_release_failure")


class ServiceCore(display.DisplayCore):
    def __init__(self,data,symbols,direction,scenario):
        self.direction=direction
        self.service_scenario=scenario
        self.systick_started=None
        self.pending_tick=False
        self.missed_ticks=0
        self.irq_vector=None
        self.clamps=[]
        self.restores=[]
        self.injected=False
        self.begin_us=None
        self.radio_during_motion=0
        self.traffic_sent=False
        self.traffic_received_at=None
        self.temperature_launches=[]
        self.vbat_launches=[]
        self.vbat_calibrations=0
        self.temperature_bias_at=None
        self.nested_irq_count=0
        self.irq_depth=0
        self.max_irq_depth=0
        self.idle_sleep_count=0
        self.idle_sleep_us=0
        # Supplied independently by the test caller, never inferred from ARR writes.
        self.motor_limit_ms=45000
        super().__init__(data,symbols,"normal")
        # The image is immutable; don't read target memory at every instruction.
        self.wfi_sites={self.code_base+i for i in range(0,len(data)-1,2) if data[i:i+2]==b'\x30\xbf'}
        self.pressed=scenario=="held_button"

    def levels(self,port,mode=None):
        # The frozen base handles GPIO/pulls; this application owns both TIM4 AFs.
        base.Core.levels(self,port,mode)
        if port!=PB:
            return
        mode=self.get(PB) if mode is None else mode
        value=self.get(PB+0x10)
        for pin,ch,af in ((14,1,5),(15,2,1)):
            if (mode>>(2*pin))&3!=2:
                continue
            assert self.get(RCC+0x34)&4 and self.get(PB+0x24)>>(4*(pin-8))&15==af
            assert self.get(TIM+0x20)==0x33
            assert self.get(TIM+0x18) in (0x7078,0x7870)
            compare=self.get(TIM+0x30+4*ch)
            low=self.get(TIM+0x24)>=compare
            value=value&~(1<<pin) if low else value|(1<<pin)
        self.put(PB+0x10,value)

    def _record_pads(self):
        self.levels(PB)
        pads=self.get(PB+0x10)&0xc000
        assert pads in (0,0xc000,0x8000 if self.direction==2 else 0x4000)
        if not self.timer_trace or self.timer_trace[-1][1]!=pads:
            self.timer_trace.append((self.virtual_us,pads))

    def advance_us(self,elapsed):
        assert isinstance(elapsed,int) and elapsed>=0
        old=self.virtual_us
        self.virtual_us+=elapsed
        if self.systick_started is not None and self.get(ST)&1:
            a=(old-self.systick_started)//1000
            b=(self.virtual_us-self.systick_started)//1000
            if b>a:
                self.missed_ticks+=max(0,b-a-1)+(1 if self.pending_tick else 0)
                self.pending_tick=True
                self.put(ICSR,self.get(ICSR)|PENDING)
            self.put(ST+8,47999-((self.virtual_us-self.systick_started)%1000)*48)
        if self.get(RCC+0x34)&4 and self.get(TIM)&1:
            assert self.get(TIM)&~1==0x88 and self.get(TIM+0x28)==47999
            assert self.get(TIM+0x2c)==self.motor_limit_ms-1 and not self.get(TIM+8)
            if self.service_scenario!="timer_stall":
                ticks,self.timer_fraction=divmod(self.timer_fraction+elapsed*48,48000)
                count=self.get(TIM+0x24)
                if count+ticks>=self.motor_limit_ms:
                    self.put(TIM+0x24,0); self.put(TIM,self.get(TIM)&~1)
                    self.put(TIM+0x10,self.get(TIM+0x10)|1)
                    self.timer_runs[-1]["expired_at_us"]=self.virtual_us
                else:
                    self.put(TIM+0x24,count+ticks)
                self._record_pads()
        if self.bgr_ready_at is not None and self.virtual_us>=self.bgr_ready_at:
            self.put(PMU+0x14,0x400); self.bgr_ready_at=None
        if self.cal_pending is not None and self.virtual_us>=self.cal_pending:
            self.put(ADC,1); self.put(ADC+0x10,self.get(ADC+0x10)|0x800)
            self.put(ADC+0x30,21); self.cal_pending=None
        if self.adc_pending is not None and self.virtual_us>=self.adc_pending[0]:
            _,channel,raw=self.adc_pending
            self.put(ADC,1); self.put(ADC+0x10,self.get(ADC+0x10)|12)
            self.put(ADC+0x40,raw)
            self.adc_events.append((self.virtual_us,channel,raw)); self.adc_pending=None
        if self.display_due is not None and self.virtual_us>=self.display_due:
            self.put(display.UART2+0x1c,self.get(display.UART2+0x1c)|0xc0)
            self.display_due=None
        if (self.service_scenario=="traffic" and self.virtual_us>=300000 and
                not self.traffic_sent and not self.selectors_low()):
            self.rx.extend(base.encode(base.Frame(0x42,2,bytes.fromhex("07000407d0"))))
            self.traffic_sent=True; self.traffic_received_at=self.virtual_us

    def code(self,cpu,address,size,user):
        self.instructions+=1
        while self.pending:
            a,value=self.pending.pop(0); self.put(a,value)
        sp=cpu.reg_read(base.UC_ARM_REG_SP)
        self.lowest_sp=min(self.lowest_sp,sp)
        assert 0x20006c20<=sp<=0x20007000
        if address==base.SENTINEL:
            assert self.in_handler
            self.reason="handler_return"; cpu.emu_stop(); return
        if not self.code_base<=address<self.code_base+len(self.data):
            lo,hi=self.symbols.get('__nvcode_start',0),self.symbols.get('__nvcode_end',0)
            assert self.code_base==0x08002000 and lo==0x20006000 and lo<=address<hi<=0x20006980
            if not getattr(self,'nv_ram_checked',False):
                source=self.symbols['__nvcode_load']-self.code_base
                assert 0<=source<source+hi-lo<=len(self.data)
                assert bytes(cpu.mem_read(lo,hi-lo))==self.data[source:source+hi-lo]
                self.nv_ram_checked=True
        if not cpu.reg_read(base.UC_ARM_REG_PRIMASK):
            pending=([15] if self.pending_tick else [])
            if (self.rx or self.uart_error) and self.get(0xe000e100)&(1<<27):pending.append(43)
            if pending:
                vector=min(pending,key=lambda v:(self.irq_priority(v),v))
                active=cpu.reg_read(base.UC_ARM_REG_XPSR)&0x1ff
                if not self.in_handler or self.irq_priority(vector)<self.irq_priority(active):
                    self.irq_vector=vector
                    self.reason="nested_irq_boundary" if self.in_handler else "irq_boundary"
                    cpu.emu_stop(); return
        if address==self.symbols["w600_app_clock"]:
            self.advance_us(5)
        if address in self.wfi_sites:
            # Model the WFI hint, not silicon current or clock-gating latency.
            assert self.symbols.get('w600_app_idle') is not None
            assert cpu.reg_read(base.UC_ARM_REG_PRIMASK)==1 and not self.in_handler
            assert self.get(0xe000ed10)==0 and self.get(ST)&7==7
            assert self.get(ST+4)==47999 and self.get(0xe000e100)==1<<27
            assert self.off() and self.selectors_low() and not self.get(TIM)&1
            assert not self.get(RCC+0x38)&0x100000
            assert self.systick_started is not None
            elapsed=0 if self.rx or self.uart_error or self.pending_tick else \
                1000-(self.virtual_us-self.systick_started)%1000
            self.idle_sleep_count+=1; self.idle_sleep_us+=elapsed
            self.advance_us(elapsed)
            cpu.reg_write(base.UC_ARM_REG_PC,(address+2)|1)
            return
        if address==self.symbols.get("w600_motor_service_begin",-1):
            self.begin_us=self.virtual_us
        if address==self.symbols["memmove"]:
            self.memmove_calls+=1
        if address==self.symbols["w600_app_main"]:
            start,size=self.symbols["__bss_start"],self.symbols["__bss_size"]
            assert not any(cpu.mem_read(start,size))
            n=self.symbols["__data_size"]
            assert bytes(cpu.mem_read(self.symbols["__data_start"],n))==bytes(cpu.mem_read(self.symbols["__data_load"],n))
            self.runtime_zero=True
        if address==self.symbols["fatal"]:
            self.in_fault=True
        if self.in_fault and self.report()[2]==0xe0:
            self.fatal_steps+=1
            if self.fatal_steps>=24:
                self.reason="fatal"; cpu.emu_stop(); return
        if not self.in_handler and address==self.symbols["w600_radio_poll"]:
            if self.timer_runs and not self.off():
                self.radio_during_motion+=1
            if self.skip_boundary:
                self.skip_boundary=False
            else:
                self.reason="foreground_boundary"; cpu.emu_stop()

    def read(self,cpu,access,address,size,value,user):
        if address==ADC+0x40 and self.service_scenario=="adc_boundary" and len(self.adc_reads)%6==5 and not self.injected:
            self.injected=True
            restore=self.timer_runs[0]["started_at_us"]+(self.get(TIM+0x24)+1)*1000+500
            self.advance_us(max(0,restore+1-self.virtual_us))
        super().read(cpu,access,address,size,value,user)

    def write(self,cpu,access,address,size,value,user):
        if getattr(self,'nv_ram_checked',False):
            lo,hi=self.symbols['__nvcode_start'],self.symbols['__nvcode_end']
            assert address+size<=lo or address>=hi,'RAM code changed after first execution'
        if (address==RCC+0x38 and self.service_scenario=="temperature_release_failure" and
                self.temperature_launches and self.get(address)&0x100000 and not value&0x100000):
            super().write(cpu,access,address,size,value,user)
            self.pending.append((address,value|0x100000)); self.injected=True
            return
        if address in (TIM+0x34,TIM+0x38):
            super().write(cpu,access,address,size,value,user)
            effective=value
            if self.service_scenario=="compare_failure" and value==1 and self.timer_runs and self.adc_launches:
                effective=self.motor_limit_ms
                self.pending.append((address,effective))
            self.put(address,effective)
            self.levels(PB)
            return
        if address==ST:
            if value&1 and not self.get(ST)&1:
                self.systick_started=self.virtual_us
                self.pending.append((ST+8,47999))
            return
        if address==ST+8:
            self.systick_started=self.virtual_us
            self.pending.append((ST+8,47999)); return
        if 0x50000000<=address<0x50001000:
            assert size==4
            port,offset=address&~0x3ff,address&0x3ff
            assert self.get(RCC+0x2c)&(1<<((port-PA)//0x400))
            self._power_write(address,value)
            if offset in (0x18,0x28):
                assert value<=0xffff
                if port==PB and value&0x200:
                    assert self.off() and self.selectors_low() and not self.get(RCC+0x34)&4
                    if offset==0x28: self.temperature_bias_at=self.virtual_us
                if offset==0x18 and ((port==PA and value&0x8000) or (port==PC and value&0x400)):
                    assert self.get(TIM)&1 and self.get(PB+0x10)&0xc000==0
                    assert port==(PA if self.direction==2 else PC)
                    self.clamps.append(self.virtual_us)
                if offset==0x28 and not self.selectors_low():
                    if (port==PA and value&0x8000) or (port==PC and value&0x400):
                        self.restores.append(self.virtual_us)
                old=self.get(port+0x14)
                self.put(port+0x14,old|value if offset==0x18 else old&~value)
                self.levels(port)
            elif offset==0:
                if port==PB:
                    assert value&0xf0000000 in (0x50000000,0xa0000000)
                self.levels(port,value)
            elif offset==0x14:
                raise AssertionError("Unexpected direct output-latch write")
            return
        if address==ADC and self.get(ADC+8)==0x80050101 and value in (4,0x80000000):
            assert size==4 and self.get(RCC+0x38)&0x100000
            assert self.off() and self.selectors_low() and not self.get(RCC+0x34)&4
            assert self.power_applied_at_us is not None and self.virtual_us-self.power_applied_at_us>=101000
            assert self.get(RCC+8)==0x40012 and self.get(ADC)==1
            assert (self.get(ADC+4),self.get(ADC+0x18),self.get(ADC+0x0c))==(0x4000,0x77,0)
            if value==0x80000000:
                self.vbat_calibrations+=1
                self.pending.append((address,0x80000001)); self.cal_pending=self.virtual_us+80
                return
            channel=self.get(ADC+0x1c)
            assert channel==(0x2000,0x4000)[len(self.vbat_launches)&1]
            assert not self.get(ADC+0x10)&0x83e
            raw=1480 if channel==0x2000 else (4095 if self.service_scenario=='vbat_bad_range' else 1350)
            self.vbat_launches.append((self.virtual_us,channel,raw))
            self.pending.append((address,5))
            if self.service_scenario!='vbat_timeout':
                self.adc_pending=(self.virtual_us+25,channel,raw)
            return
        if address==ADC and value==4:
            assert size==4 and self.get(RCC+0x38)&0x100000
            channel=self.get(ADC+0x1c)
            if channel==1:
                assert self.off() and self.selectors_low() and not self.get(RCC+0x34)&4
                assert self.temperature_bias_at is not None and self.virtual_us-self.temperature_bias_at>=1000
                assert self.get(PA)&12==12 and not self.get(PA+0x0c)&12
                assert self.get(PB)&(3<<18)==1<<18 and not self.get(PB+0x10)&0x200
                assert self.get(RCC+8)==0x40012 and self.get(ADC)==1
                assert (self.get(ADC+4),self.get(ADC+8),self.get(ADC+0x18),self.get(ADC+0x0c))==(0x4000,0x80010101,0x77,0)
                assert not self.get(ADC+0x10)&0x83e
                if self.temperature_launches and len(self.temperature_launches)%5:
                    assert self.virtual_us-self.temperature_launches[-1][0]>=1000
                raw=0 if self.service_scenario=="temperature_bad_raw" else (2048,2049,2047,2048,2048)[len(self.temperature_launches)%5]
                self.temperature_launches.append((self.virtual_us,channel,raw))
                self.pending.append((address,5))
                if self.service_scenario!="temperature_adc_timeout":
                    self.adc_pending=(self.virtual_us+25,channel,raw)
                return
            assert self.get(PB+0x10)&0xc000==0 and not self.selectors_low()
            assert self.clamps and self.virtual_us-self.clamps[-1]>=500
            assert self.get(RCC+8)==0x40012 and self.get(ADC)==1
            assert (self.get(ADC+4),self.get(ADC+8),self.get(ADC+0x18),self.get(ADC+0x0c))==(0x4000,0x80010101,0x77,0)
            assert not self.get(ADC+0x10)&0x83e
            assert channel==(0x2000,0x10000)[len(self.adc_launches)&1]
            group=len(self.adc_launches)//6
            signal=1 if group==0 or group>=21 else 1700
            raw=1480 if channel==0x2000 else signal
            self.adc_launches.append((self.virtual_us,channel,raw))
            self.pending.append((address,5))
            if self.service_scenario!="adc_timeout" or group==0:
                self.adc_pending=(self.virtual_us+25,channel,raw)
            return
        super().write(cpu,access,address,size,value,user)

    def motor_report(self):
        return struct.unpack("<32I",self.cpu.mem_read(self.symbols["motor_report"],128))

    def irq_priority(self,vector):
        assert vector in (15,43),vector
        return (self.get(0xe000ed20 if vector==15 else 0xe000e418)>>24)&0xc0

    def handler(self,vector):
        assert self.reason=="foreground_boundary" and not self.cpu.reg_read(base.UC_ARM_REG_PRIMASK)
        previous=self.in_handler
        active=self.cpu.reg_read(base.UC_ARM_REG_XPSR)&0x1ff
        if previous:
            assert self.irq_priority(vector)<self.irq_priority(active)
            self.nested_irq_count+=1
        if vector==15:
            assert self.get(ST)&7==7
            self.pending_tick=False;self.put(ICSR,self.get(ICSR)&~PENDING)
        else:assert vector==43 and self.get(0xe000e100)&(1<<27)
        saved=self.cpu.context_save()
        sp=self.cpu.reg_read(base.UC_ARM_REG_SP)-32
        self.cpu.reg_write(base.UC_ARM_REG_MSP,sp)
        self.cpu.reg_write(base.UC_ARM_REG_XPSR,0x01000000|vector)
        self.cpu.reg_write(base.UC_ARM_REG_LR,base.SENTINEL|1)
        self.in_handler=True;self.irq_depth+=1
        self.max_irq_depth=max(self.max_irq_depth,self.irq_depth)
        entry=self.get(self.get(0xe000ed08)+4*vector)&~1
        while True:
            self.execute(entry,100000)
            if self.reason!="nested_irq_boundary":break
            nested=self.irq_vector;self.reason="foreground_boundary"
            self.handler(nested)
            if self.reason=="fatal":break
            entry=self.cpu.reg_read(base.UC_ARM_REG_PC)
        assert self.reason in ("handler_return","fatal"),self.reason
        if self.reason=="handler_return":
            self.cpu.context_restore(saved);self.reason="foreground_boundary"
        self.irq_depth-=1;self.in_handler=previous
        if vector==15:self.ticks+=1
        else:self.uart_irqs+=1

    def step(self,elapsed=125):
        if self.reason=="fatal":
            return
        if self.reason=="irq_boundary":
            vector=self.irq_vector
            self.reason="foreground_boundary"
            self.handler(vector)
            self.advance_us(10)
        elif self.reason=="foreground_boundary":
            self.advance_us(elapsed)
            self.skip_boundary=True
        else:
            raise AssertionError(self.reason)
        self.execute(self.cpu.reg_read(base.UC_ARM_REG_PC))

    def checks(self):
        assert self.off() and self.selectors_low()
        assert bytes(self.cpu.mem_read(0x08000000,0x20000))==self.flash
        assert bytes(self.cpu.mem_read(0x20006c00,32))==self.canary
        assert not self.get(RCC+0x30)&1
        trace=None
        if "w600_motor_trace" in self.symbols:
            address=self.symbols["w600_motor_trace"]
            header=struct.unpack("<8I",self.cpu.mem_read(address,32))
            if not self.runtime_zero:
                assert header==(0xa5a5a5a5,)*8,header
            elif header[0]:
                assert header[:3]==(0x57365452,1,2256),header
                assert header[4] in (1,2) and header[5]<=2256 and header[6:]==(0,0),header
                records=[struct.unpack("<2I",self.cpu.mem_read(address+32+8*i,8)) for i in range(header[5])]
                assert all(0<((b[0]-a[0])&0xffffffff)<1000 for a,b in zip(records,records[1:])),records
                report=self.motor_report()
                if report[5]:
                    assert header[4:6]==(report[14],report[5]),(header,report)
                    assert records[-1][1]==report[6],(records[-1],report)
                trace={"header":header,"samples":len(records),"last":records[-1] if records else None}
        return {"virtual_us":self.virtual_us,"instructions":self.instructions,
                "lowest_sp":hex(self.lowest_sp),"systick_handlers":self.ticks,
                "uart_handlers":self.uart_irqs,"missed_ticks":self.missed_ticks,
                "motor_report":self.motor_report()[:17],"app_report":self.report()[:19],
                "motor_time_report":self.motor_report()[17:23],
                "timer_runs":self.timer_runs,"sampling_windows":len(self.clamps),
                "adc_conversions":len(self.adc_launches),"radio_during_motion":self.radio_during_motion,
                "tx_hex":self.tx.hex(),"display_hex":self.display_wire.hex(),
                "traffic_received_at_us":self.traffic_received_at,
                "idle_cpu_sleep_count":self.idle_sleep_count,"idle_cpu_sleep_us":self.idle_sleep_us,
                "motor_trace":trace,"hardware_tested":False,"cycle_accurate":False}


def run(data,symbols,direction,scenario):
    m=ServiceCore(data,symbols,direction,scenario)
    if scenario=="bad_factory": m.put(0x1fff03c0,0)
    m.boot()
    for _ in range(30000):
        if m.reason=="fatal": break
        if scenario in ("display_fault","radio_error","clock_fault") and m.virtual_us>=300000 and not m.injected:
            m.injected=True
            if scenario=="display_fault": m.put(display.UART2+0x0c,2499)
            elif scenario=="radio_error": m.uart_error=8
            else: m.put(ST+4,47998)
        if scenario in ("foreground_stall","cpu_stall","missed_window") and m.timer_runs and not m.injected and m.virtual_us>=270000:
            m.injected=True
            if scenario=="cpu_stall":
                m.advance_us(45001000)
                assert m.get(PB+0x10)&0xc000==0xc000 and not m.get(TIM)&1
            elif scenario=="foreground_stall":
                until=m.virtual_us+110000
                while m.virtual_us<until:
                    m.advance_us(1000)
                    assert not m.in_handler and m.reason in ("foreground_boundary","irq_boundary")
                    m.pending_tick=False; m.put(ICSR,m.get(ICSR)&~PENDING)
                    m.reason="foreground_boundary"; m.handler(15)
                assert m.off()
            else:
                m.advance_us(2500)
        m.step(200 if not m.timer_runs else 125)
        report=m.motor_report()
        if report[2] in (7,8): break
    else:
        raise AssertionError((scenario,"No bounded motor result",m.motor_report(),m.virtual_us))
    if scenario=="radio_error":
        assert m.reason=="fatal" and m.report()[3]==7 and m.report()[4]==1
        assert m.motor_report()[2:5]==(8,9,1) and m.motor_report()[10:13]==(0,0,0)
        assert not m.get(RCC+0x38) and not m.get(RCC+0x34)&4
        return m.checks()
    assert m.reason!="fatal",(scenario,m.report())
    report=m.motor_report()
    assert report[0:2]==(0x57364d53,2) and report[4]==1
    assert report[10:13]==(0,0,0) and report[13]==1
    assert report[23:]==(0,)*9
    if report[3]: assert report[17:23]==(0,)*6
    else:
        assert report[22]==1 and 0<report[18]<=report[19]<450000,report
        assert report[20]<=report[18] and report[21]<=report[19]
    assert m.get(RCC+0x34)==(0x40000 if scenario=="display_fault" else 0x60000)
    assert not m.get(RCC+0x38) and m.get(RCC+8)==0x12
    if scenario in ("normal","traffic","held_button") or scenario.startswith("temperature"):
        assert report[2:4]==(7,0) and report[7]==3 and report[5]==23,report
        assert len(m.timer_runs)==1 and m.radio_during_motion>10 and not m.missed_ticks
        assert m.clamps and len(m.clamps)==23
    else:
        assert report[2]==8 and report[3],report
    if scenario=="traffic":
        assert m.traffic_sent and m.traffic_received_at and any(at<=m.traffic_received_at<at+1250 for at in m.clamps)
        # Let the last accepted frame's nonblocking UART/display sends finish.
        for _ in range(2000):
            m.step()
            if len(m.tx)>=9 and m.display_report()[3:5]==(2,2): break
        assert base.decode(bytes(m.tx))==base.Frame(0x42,3,bytes.fromhex("000700"))
        assert m.display_report()[3:5]==(2,2)
        assert bytes(m.display_wire)==bytes.fromhex("080000000018ff08377b001018ff")
    if scenario=="held_button":
        # Exercise a real long hold without wasting foreground polls after motion.
        for _ in range(10010):
            m.advance_us(1000); m.pending_tick=False; m.put(ICSR,m.get(ICSR)&~PENDING)
            m.reason="foreground_boundary"; m.handler(15)
        m.step()
        assert m.report()[6:8]==(0,0)
    if scenario=="compare_failure": assert len(m.clamps)==1 and report[3]==4
    if scenario=="adc_boundary": assert m.injected and report[3]==5 and report[5]==0
    if scenario.startswith("temperature"):
        return run_temperature(m,scenario)
    return m.checks()


def run_temperature(m,scenario):
    def frames():
        result=[]; pos=0
        while pos<len(m.tx):
            if len(m.tx)-pos<8: return None
            size=10+int.from_bytes(m.tx[pos+5:pos+7],"big")
            if len(m.tx)-pos<size: return None
            result.append(base.decode(bytes(m.tx[pos:pos+size]))); pos+=size
        return result
    def reports_done(count):
        for _ in range(3000):
            m.step()
            decoded=frames()
            if (decoded is not None and sum(f.kind==5 for f in decoded)==count and
                m.get(PA+0x14)&(1<<11)):
                reports=[f for f in decoded if f.kind==5]
                assert all(f.payload==bytes.fromhex("0706040992") for f in reports),reports
                assert [f.sequence for f in reports]==list(range(1,count+1)),reports
                return decoded
        raise AssertionError((scenario,"No completed local-temperature report",bytes(m.tx).hex()))
    def acquire(sequence):
        for _ in range(10000):
            m.step()
            n=len(m.temperature_launches)
            if n and scenario=="temperature_traffic" and not m.traffic_sent:
                m.rx.extend(base.encode(base.Frame(0x43,2,bytes.fromhex("0705040834"))))
                m.traffic_sent=True
            if n>=2 and not m.injected:
                if scenario=="temperature_display_fault":
                    m.put(display.UART2+0x0c,2499); m.injected=True
                elif scenario=="temperature_radio_error":
                    m.uart_error=8; m.injected=True
                elif scenario=="temperature_sensor_pin_fault":
                    m.put(PB+0x0c,m.get(PB+0x0c)|(1<<26)); m.injected=True
            r=m.report()
            if m.reason=="fatal" or (r[23]>=sequence and (r[22] or r[20]==7)):
                return r
        raise AssertionError((scenario,"No bounded temperature result",m.report()))
    r=acquire(1)
    assert m.off() and m.selectors_low() and len(m.timer_runs)==1
    if scenario=="temperature_radio_error":
        assert m.reason=="fatal" and r[2:5]==(0xe0,7,1)
        assert not m.get(RCC+0x38) and m.get(RCC+8)==0x12
    else:
        assert m.reason!="fatal" and r[19]==1 and r[23]==1
        if scenario in ("temperature_adc_timeout","temperature_bad_raw","temperature_display_fault","temperature_sensor_pin_fault","temperature_release_failure"):
            expected={"temperature_adc_timeout":6,"temperature_bad_raw":9,
                      "temperature_display_fault":11,"temperature_sensor_pin_fault":4,
                      "temperature_release_failure":10}[scenario]
            assert r[20:23]==(7,expected,0),r
            assert not m.cpu.mem_read(m.symbols["radio"]+4,1)[0]&8
        else:
            assert r[20:28]==(6,0,1,1,r[24],2048,245,245),r
            assert len(m.temperature_launches)==5
            assert m.get(m.symbols["radio"]+4)&8
            assert struct.unpack("<H",m.cpu.mem_read(m.symbols["radio"]+6,2))[0]==2450
        if scenario=="temperature_release_failure":
            assert r[31]==1 and m.get(RCC+0x38)&0x100000
        else:
            assert not r[31] and not m.get(RCC+0x38) and m.get(RCC+8)==0x12
            assert m.get(RCC+0x34)==(0x40000 if scenario=="temperature_display_fault" else 0x60000)
            assert m.get(PB)&(3<<18)==3<<18 and not m.get(PB+0x0c)&(3<<18)
        if r[22]: reports_done(1)
        else: assert not any(f.kind==5 for f in (frames() or []))
    if scenario=="temperature_traffic":
        m.rx.extend(base.encode(base.Frame(0x44,1,bytes.fromhex("07050706"))))
        for _ in range(3000):
            m.step()
            if m.report()[9]==2: break
        decoded=reports_done(1)
        responses=[f for f in decoded if f.kind!=5]
        assert responses==[base.Frame(0x43,3,bytes.fromhex("000705")),
                           base.Frame(0x44,4,bytes.fromhex("070500040834070600040992"))],decoded
        assert m.get(m.symbols["radio"]+4)&0xffff==0x010e  # External source retained.
    if scenario=="temperature_repeat":
        first=r[24]
        for _ in range(60001):
            m.advance_us(1000); m.pending_tick=False; m.put(ICSR,m.get(ICSR)&~PENDING)
            m.reason="foreground_boundary"; m.handler(15)
        m.reason="foreground_boundary"
        r=acquire(2)
        assert r[20:24]==(6,0,1,2) and r[24]-first>=60000 and len(m.temperature_launches)==10
        assert len(m.timer_runs)==1 and not m.get(RCC+0x38) and m.get(RCC+8)==0x12
        reports_done(2)
    result=m.checks()
    result.update(temperature_report=m.report()[19:],temperature_conversions=len(m.temperature_launches),
                  temperature_tx_frames=sum(f.kind==5 for f in (frames() or [])),
                  temperature_hardware_accuracy_validated=False)
    return result


def main():
    out=Path(sys.argv[1]); manifest=json.loads((out/"manifest.json").read_text())
    data=(out/"replacement-core.bin").read_bytes()
    assert hashlib.sha256(data).hexdigest()==manifest["bin_sha256"]
    assert manifest["motor_service_bench"] and manifest["display_enabled"]
    scenarios=sys.argv[2:] or SCENARIOS
    sources={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in
        (Path(__file__),Path(display.__file__),Path(probe.__file__),Path(base.__file__))}
    result_path=out/"service-thumb-results.json"
    previous=json.loads(result_path.read_text()) if result_path.exists() else {}
    results=previous.get("cases",{}) if previous.get("bin_sha256")==manifest["bin_sha256"] and previous.get("model_sources")==sources else {}
    for scenario in scenarios:
        results[scenario]=run(data,manifest["symbols"],manifest["motor_direction"],scenario)
        print("PASS combined actual-linked",manifest["motor_direction"],scenario,flush=True)
        result_path.write_text(json.dumps({"bin_sha256":manifest["bin_sha256"],"model_sources":sources,
            "hardware_tested":False,"cycle_accurate":False,"complete":set(SCENARIOS)<=set(results),
            "cases":results},indent=2)+"\n")


if __name__=="__main__": main()
