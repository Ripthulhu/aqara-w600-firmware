"""Execute the FLASH-linked development application on the existing modeled bus.

No device access, RF, visible-screen or electrical timing claim. The preserved
bootloader's observed 48 MHz handoff is supplied, not emulated here.
"""
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from itertools import repeat
from multiprocessing import get_context
from pathlib import Path
import sys
from unicorn.arm_const import UC_ARM_REG_R1, UC_ARM_REG_R2

import service_thumb as service

base=service.base
RCC=service.RCC
SCENARIOS=("startup_mode", "temperature_repeat", "held_button",
           "mount_cancel", "mount_heat_reject", "mount_invalid_reject", "mount_no_motion",
           "idle_sleep", "idle_uart_wake", "idle_button_wake",
           "calibration_cancel", "calibration_duplicate", "calibration_no_motion",
           "calibration_timing_temperature",
           "idle_temperature", "traffic",
           "bad_clock", "bad_pll", "bad_latency", "bad_watchdog", "bad_iopen",
           "bad_irq", "bad_apb", "bad_trim", "temperature_adc_timeout",
           "temperature_bad_raw", "temperature_display_fault", "temperature_radio_error",
           "vbat", "vbat_timeout", "vbat_bad_range", "vbat_radio_error", "vbat_display_fault")


class ApplicationCore(service.ServiceCore):
    def __init__(self,data,symbols,scenario):
        self.direction_buttons=0
        self.idle_wake_armed=False
        self.idle_wake_at=None
        super().__init__(data,symbols,1,scenario)
        self.put(RCC,0x03000500)
        self.put(RCC+8,0x12)
        self.put(RCC+0x0c,0x30260)
        self.put(0x40022000,2)

    def dial_bias_ready(self):
        return (self.get(service.PA)&0x3000==0x1000 and
                bool(self.get(service.PA+4)&0x40) and
                self.get(service.PA+0x0c)&0x3000==0x2000 and
                not self.get(service.PA+0x14)&0x40)

    def boot(self):
        super().boot()
        if self.report()[2]==2:
            assert self.dial_bias_ready(), "Missing PA6 open-drain LOW dial bias"

    def levels(self,port,mode=None):
        super().levels(port,mode)
        mode=self.get(port) if mode is None else mode
        # Dial contacts can pull low only through their PA6 common return.
        connected=self.dial_bias_ready()
        for bank,pin,mask in ((service.PA,2,4),(service.PB,1,2)):
            if port==bank and (mode>>(2*pin))&3==0:
                value=self.get(port+0x10)
                value=value&~(1<<pin) if connected and self.direction_buttons&mask else value|(1<<pin)
                self.put(port+0x10,value)

    def code(self,cpu,address,size,user):
        if self.vbat_launches and not self.injected:
            if self.service_scenario=='vbat_radio_error':
                self.uart_error=1; self.injected=True
            elif self.service_scenario=='vbat_display_fault':
                self.put(service.display.UART2+0x1c,self.get(service.display.UART2+0x1c)|8); self.injected=True
        if self.idle_wake_armed and address in self.wfi_sites:
            self.idle_wake_armed=False; self.idle_wake_at=self.virtual_us
            if self.service_scenario=='idle_uart_wake':
                self.rx.extend(base.encode(base.Frame(0x61,2,bytes.fromhex('0700040866'))))
            elif self.service_scenario=='idle_button_wake':
                self.direction_buttons=4
                self.levels(service.PA)
            else: raise AssertionError(self.service_scenario)
        if address in (self.symbols.get("w600_motor_service_begin",-1),
                       self.symbols.get("w600_motor_service_begin_bounded",-1)):
            self.direction=cpu.reg_read(UC_ARM_REG_R1)
            if address==self.symbols.get("w600_motor_service_begin_bounded",-1):
                assert cpu.reg_read(UC_ARM_REG_R2)==self.motor_limit_ms
        super().code(cpu,address,size,user)

    def write(self,cpu,access,address,size,value,user):
        super().write(cpu,access,address,size,value,user)
        if (self.service_scenario in ("calibration_no_motion","mount_no_motion") and address==service.ADC and
                value==4 and self.get(service.ADC+0x1c)==0x10000):
            at,channel,_=self.adc_launches[-1]
            self.adc_launches[-1]=(at,channel,1)
            if self.adc_pending is not None:
                self.adc_pending=(self.adc_pending[0],channel,1)


class NominalCore(ApplicationCore):
    """Functional tests poll at 0.5 ms; timing sweeps retain ApplicationCore."""
    def step(self,elapsed=125):
        super().step(max(elapsed,500))

    def code(self,cpu,address,size,user):
        if address==self.symbols['w600_app_clock']:self.advance_us(20)
        super().code(cpu,address,size,user)

    def checks(self):
        return dict(super().checks(),foreground_clock_step_us=25,foreground_dispatch_us=500,
                    cycle_accurate=False)


def frames(m,allow_partial=False):
    result=[]; pos=0
    while pos<len(m.tx):
        if len(m.tx)-pos<8: return result if allow_partial else None
        size=10+int.from_bytes(m.tx[pos+5:pos+7],"big")
        if len(m.tx)-pos<size: return result if allow_partial else None
        result.append(base.decode(bytes(m.tx[pos:pos+size]))); pos+=size
    return result


def reply_count(m):
    """Count command responses, excluding startup and status announcements."""
    return sum(frame.kind in (3,4) for frame in frames(m,allow_partial=True))


def until(m,predicate,limit=20000):
    for _ in range(limit):
        if predicate(): return
        assert m.reason!="fatal",m.report()
        m.step(125)
    raise AssertionError(("No bounded application result",m.report(),m.virtual_us))


def idle_ticks(m,count):
    # Keep foreground paused but execute every real SysTick handler, as in V5.
    assert m.reason=="foreground_boundary" and not m.pending_tick
    for _ in range(count):
        m.advance_us(1000)
        m.pending_tick=False; m.put(service.ICSR,m.get(service.ICSR)&~service.PENDING)
        m.reason="foreground_boundary"; m.handler(15)


def run(data,symbols,scenario):
    precise=scenario.startswith('idle_') or scenario=='calibration_timing_temperature'
    m=(ApplicationCore if precise else NominalCore)(data,symbols,scenario)
    corrupt={"bad_clock":(RCC+8,0), "bad_pll":(RCC+0x0c,0x20260),
             "bad_latency":(0x40022000,0), "bad_watchdog":(0x40003004,7),
             "bad_iopen":(RCC+0x2c,8), "bad_irq":(0xe000e100,1),
             "bad_apb":(RCC+0x34,1), "bad_trim":(0x1fff03cc,0)}
    if scenario in corrupt: m.put(*corrupt[scenario])
    m.boot()
    if scenario.startswith('mount_'):
        at=symbols['mounting_report']
        def mount(): return tuple(m.get(at+4*i) for i in range(4))
        until(m,lambda:m.virtual_us>=250000)
        assert mount()==(0,0,0,0) and not m.timer_runs and m.off()
        if scenario=='mount_heat_reject':
            m.rx.extend(base.encode(base.Frame(0x60,2,bytes.fromhex('07030001'))))
            until(m,lambda:reply_count(m)==1 and frames(m) is not None)
        m.put(at,1 if scenario=='mount_invalid_reject' else 0xa7)
        if scenario.endswith('_reject'):
            until(m,lambda:mount()[2]==3)
            assert mount()==(0,1,3,0) and not m.timer_runs and m.off()
            m.rx.extend(base.encode(base.Frame(0x61,2,bytes.fromhex('07030000'))))
            until(m,lambda:reply_count(m)>=(2 if scenario=='mount_heat_reject' else 1))
        else:
            until(m,lambda:mount()[2]==1 and len(m.timer_runs)==1)
            assert m.direction==1 and mount()[1]==1
            m.put(at,0xa7)  # Duplicate cannot restart or extend the run.
            if scenario=='mount_cancel':
                until(m,lambda:m.motor_report()[5]>=3)
                m.rx.extend(base.encode(base.Frame(0x62,2,bytes.fromhex('07030000'))))
            until(m,lambda:mount()[2]==4 and m.off())
            assert mount()[3] and len(m.timer_runs)==1 and m.motor_report()[4]==1
        before=len(m.timer_runs)
        m.put(at,0xa7)
        start=m.virtual_us
        until(m,lambda:m.virtual_us-start>=150000 and m.get(at)==0)
        assert len(m.timer_runs)==before and mount()[1]==1 and m.off()
        assert not m.cpu.mem_read(symbols['radio']+13,1)[0]  # No calibration claim.
        assert not m.cpu.mem_read(symbols['radio']+14,1)[0]  # No invented position.
        result=m.checks();result.update(mount_report=mount(),one_shot=True,
            startup_motion=False,calibration_claimed=False,physical_retraction_validated=False)
        return result
    if scenario.startswith('vbat'):
        def report():
            return tuple(m.get(symbols['vbat_report']+4*i) for i in range(32))
        until(m,lambda:report()[0]==0x57364241 and report()[2] in (4,5))
        r=report()
        assert r[1]==1 and r[9]==1 and r[7]>=1500 and r[8]-r[7]<100
        assert r[10:13]==(0,0,0) and not m.get(RCC+0x38)&0x100000
        assert not m.get(RCC+0x34)&0x10000000 and m.get(RCC+8)==0x12
        assert not m.timer_runs and not m.adc_launches and m.off()
        if scenario=='vbat':
            assert r[2:7]==(4,0,1,3296,1495),r
            assert len(m.vbat_launches)==6 and m.vbat_calibrations==1
            assert r[13]>=r[7] and r[14]<=r[8] and r[15:19]==(1,0,1,1)
            for i in range(3):
                assert r[19+3*i:21+3*i]==(1480,1350)
                assert r[13]<=r[21+3*i]<=r[14]
            # A later ordinary request still reaches the real decoder and reply.
            m.rx.extend(base.encode(base.Frame(0x68,1,bytes.fromhex('070b071c'))))
            until(m,lambda:reply_count(m)==1 and frames(m) is not None)
            assert frames(m)[-1]==base.Frame(0x68,4,bytes.fromhex('070b000000071c0100'))
            start=m.virtual_us
            until(m,lambda:m.virtual_us-start>=100000)
            assert report()==r and len(m.vbat_launches)==6
        else:
            assert r[2]==5 and r[3]!=0 and not r[4],r
            if scenario=='vbat_bad_range': assert r[3]==7 and r[5]>3600
            elif scenario=='vbat_timeout': assert r[3]==3 and len(m.vbat_launches)==1
            else: assert r[3]==9 and m.injected
        f=frames(m) or []
        assert not any(bytes.fromhex('2004') in x.payload or bytes.fromhex('2005') in x.payload for x in f)
        result=m.checks(); result.update(vbat_report=r,vbat_conversions=len(m.vbat_launches),
            battery_publication=False,hardware_voltage_accuracy_validated=False)
        return result
    if scenario in ('idle_sleep','idle_uart_wake','idle_button_wake'):
        until(m,lambda:m.idle_sleep_count>=4)
        before=m.idle_sleep_count
        if scenario=='idle_sleep':
            start=m.virtual_us
            until(m,lambda:m.virtual_us-start>=250000)
            assert m.idle_sleep_count>before+200 and m.idle_sleep_us>=200000
        else:
            m.rx.extend(base.encode(base.Frame(0x60,2,bytes.fromhex('07000407d0'))))
            seed_ack=base.Frame(0x60,3,bytes.fromhex('000700'))
            until(m,lambda:seed_ack in (frames(m) or []) and m.get(service.PA+0x14)&(1<<11))
            before_replies=reply_count(m)
            m.idle_wake_armed=True
            if scenario=='idle_uart_wake':
                until(m,lambda:reply_count(m)>=before_replies+1 and
                    base.Frame(0x61,3,bytes.fromhex('000700')) in (frames(m) or []) and m.get(service.PA+0x14)&(1<<11))
                assert base.Frame(0x61,3,bytes.fromhex('000700')) in frames(m)
                assert m.display_report()[5]==0x10000
            else:
                until(m,lambda:m.display_report()[5]==2050)
                assert 0<=m.virtual_us-m.idle_wake_at<=3000, \
                    (m.virtual_us-m.idle_wake_at,m.display_report(),m.report())
                until(m,lambda:m.display_report()[2]==0)
                assert m.virtual_us-m.idle_wake_at<=50000
            assert m.idle_wake_at is not None
        assert not m.timer_runs and m.off() and not m.missed_ticks
        result=m.checks(); result.update(idle_wake_case=scenario,
            unchanged_systick=True,automatic_motor_motion=False)
        return result
    if scenario=="startup_mode":
        def mode_reports():
            return [f for f in (frames(m) or []) if f.kind==5 and b'\x07\x03' in f.payload]
        until(m,lambda:mode_reports() and m.get(service.PA+0x14)&(1<<11))
        assert mode_reports()[-1].payload==bytes.fromhex('071c03ffff07030000')
        for sequence,value in ((0x60,1),(0x61,0)):
            before=len(mode_reports())
            m.rx.extend(base.encode(base.Frame(sequence,2,bytes.fromhex('070300')+bytes([value]))))
            until(m,lambda:len(mode_reports())>before and m.get(service.PA+0x14)&(1<<11))
            assert mode_reports()[-1].payload==bytes.fromhex('071c03ffff070300')+bytes([value])
        assert not m.timer_runs and m.off() and m.motor_report()[2:5]==(0,0,1)
        result=m.checks();result.update(startup_off_reported=True,mode_changes_reported_without_motion=True,
            target_or_position_invented=False,calibration_proven=False,startup_false_failure_omitted=True,
            startup_reference_unknown_reported=True)
        return result
    if scenario=="calibration_timing_temperature":
        until(m,lambda:m.display_report()[3:5]==(1,1))
        m.rx.extend(base.encode(base.Frame(0x60,2,bytes.fromhex('07090001'))))
        until(m,lambda:len(m.clamps)==1)
        m.advance_us(2500)
        until(m,lambda:m.motor_report()[2]==8)
        failure=m.motor_report()
        assert failure[3]==5 and failure[4]==1 and failure[10:13]==(0,0,0)
        while m.pending_tick: m.step()
        idle_ticks(m,1001)
        until(m,lambda:m.report()[23]==1 and m.report()[20]==6)
        until(m,lambda:frames(m) is not None and any(f.kind==5 for f in frames(m)))
        while m.pending_tick: m.step()
        idle_ticks(m,60001)
        until(m,lambda:m.report()[23]==2 and m.report()[20]==6)
        assert m.motor_report()[2:5]==failure[2:5] and len(m.timer_runs)==1 and m.off()
        assert len(m.temperature_launches)==10 and m.report()[22]==1
        result=m.checks()
        result.update(temperature_after_motor_failure=True,automatic_motor_retries=0)
        return result
    if scenario=="calibration_no_motion":
        until(m,lambda:m.display_report()[3:5]==(1,1))
        m.rx.extend(base.encode(base.Frame(0x60,2,bytes.fromhex("07090001"))))
        # Reject motionless opening preparation without starting another action.
        until(m,lambda:len(m.timer_runs)==1 and m.motor_report()[2]==7 and m.off(),limit=50000)
        start=m.virtual_us
        until(m,lambda:m.virtual_us-start>=150000 and frames(m) is not None and
            any(f.kind==5 and f.payload[:4]==bytes.fromhex('070b0000') for f in frames(m)) and
            m.get(service.PA+0x14)&(1<<11))
        assert len(m.timer_runs)==1 and m.motor_report()[3]==0 and m.direction==1
        m.rx.extend(base.encode(base.Frame(0x61,1,bytes.fromhex("070b071c"))))
        until(m,lambda:reply_count(m)==2 and frames(m) is not None and m.get(service.PA+0x14)&(1<<11))
        assert frames(m)[-1]==base.Frame(0x61,4,bytes.fromhex("070b000000071c0100"))
        result=m.checks()
        result.update(calibration_completed=False,no_motion_failed=True,automatic_retries=0,
            command_transport="real UART decoder, endpoint, controller, driver and handlers")
        return result
    if scenario in ("calibration_cancel","calibration_duplicate"):
        until(m,lambda:m.display_report()[3:5]==(1,1))
        # Drain the spaced startup reports before classifying post-command reports.
        until(m,lambda:frames(m) is not None and any(f.kind==5 and
            f.payload==bytes.fromhex('071c03ffff07030000') for f in frames(m)) and
            m.get(service.PA+0x14)&(1<<11))
        before_command=len(frames(m))
        command=base.Frame(0x60,2,bytes.fromhex("07090001"))
        m.rx.extend(base.encode(command))
        until(m,lambda:len(m.timer_runs)==1 and m.motor_report()[5]>=3)
        until(m,lambda:frames(m) is not None and
            any(f.kind==5 for f in frames(m)[before_command:]) and m.get(service.PA+0x14)&(1<<11))
        busy_reports=[f for f in frames(m)[before_command:] if f.kind==5]
        assert busy_reports and all(f.payload==bytes.fromhex('07030000') for f in busy_reports),busy_reports
        if scenario=="calibration_duplicate":
            m.rx.extend(base.encode(command))
            until(m,lambda:reply_count(m)>=2)
            assert len(m.timer_runs)==1
        m.rx.extend(base.encode(base.Frame(0x61,2,bytes.fromhex("07030000"))))
        expected_replies=3 if scenario=="calibration_duplicate" else 2
        until(m,lambda:m.motor_report()[2]==7 and reply_count(m)>=expected_replies and
            frames(m) is not None and m.get(service.PA+0x14)&(1<<11))
        before=len(m.timer_runs)
        start=m.virtual_us
        until(m,lambda:m.virtual_us-start>=150000)
        assert len(m.timer_runs)==before==1 and m.motor_report()[2:4]==(7,0)
        assert m.off() and m.motor_report()[4]==1 and m.motor_report()[10:13]==(0,0,0)
        replies=[f for f in frames(m) if f.kind==3]
        assert all(f.payload in (bytes.fromhex("000709"),bytes.fromhex("000703")) for f in replies)
        assert len(replies)==expected_replies
        until(m,lambda:frames(m) is not None and m.get(service.PA+0x14)&(1<<11) and
            any(f.kind==5 and f.payload==bytes.fromhex('070b000007030000') for f in frames(m)))
        reports=[f for f in frames(m) if f.kind==5 and f.payload[:2]!=bytes.fromhex('2002')]
        assert reports[-1].payload==bytes.fromhex("070b000007030000")
        assert all(f.payload in (bytes.fromhex('071c03ffff07030000'),bytes.fromhex('07030000'),
            bytes.fromhex('070b000007030000')) for f in reports),reports
        result=m.checks()
        result.update(command_transport="real UART decoder, endpoint, controller, driver and handlers",
            calibration_completed=False,cancelled=True,radio_replies=len(replies),
            busy_reports_omit_completion_flag=True,
            duplicate_did_not_restart=scenario=="calibration_duplicate")
        return result
    if scenario in corrupt:
        assert m.reason=="fatal" and m.report()[2:5]==(0xe0,2 if scenario=="bad_trim" else 1,1)
        assert m.runtime_zero==(scenario=="bad_trim")
        assert not m.tx and not m.display_wire
    else:
        assert m.reason!="fatal" and m.runtime_zero
        until(m,lambda:m.display_report()[3:5]==(1,1))
        assert bytes(m.display_wire)==bytes.fromhex("080000000018ff")
        if scenario=="traffic":
            m.rx.extend(base.encode(base.Frame(0x42,2,bytes.fromhex("07000407d0"))))
            until(m,lambda:reply_count(m)==1 and frames(m) is not None)
            assert [f for f in frames(m) if f.kind==3]==[base.Frame(0x42,3,bytes.fromhex("000700"))]
            assert bytes(m.display_wire)==bytes.fromhex("080000000018ff")
            m.rx.extend(base.encode(base.Frame(0x43,2,bytes.fromhex("0705040834"))))
            until(m,lambda:reply_count(m)==2 and frames(m) is not None)
        if scenario=="held_button":
            # A button held from power-up must not erase pairing credentials.
            until(m,lambda:frames(m) is not None and len(frames(m))==2 and
                  m.get(service.PA+0x14)&(1<<11))
            while m.pending_tick: m.step()
            idle_ticks(m,10010)
            m.step()
            until(m,lambda:m.report()[5]>=10000)
            assert m.report()[6:8]==(0,0)
            revision,control=frames(m)
            assert revision.sequence==1 and revision.kind==5 and len(revision.payload)==5
            assert revision.payload[:3]==bytes.fromhex('200203')
            assert control==base.Frame(2,5,bytes.fromhex('071c03ffff07030000'))
        else:
            injected=False
            for _ in range(20000):
                r=m.report()
                if m.reason=="fatal" or (r[23] and r[20] in (6,7)): break
                if len(m.temperature_launches)>=2 and not injected:
                    if scenario=="temperature_display_fault":
                        m.put(service.display.UART2+0x0c,2499); injected=True
                    elif scenario=="temperature_radio_error":
                        m.uart_error=8; injected=True
                m.step()
            else: raise AssertionError((scenario,"No temperature result",m.report()))
            r=m.report()
            if scenario=="temperature_radio_error":
                assert m.reason=="fatal" and r[2:5]==(0xe0,7,1)
                assert m.motor_report()[10:13]==(0,0,0) and not m.get(RCC+0x38)
            else:
                assert m.reason!="fatal"
                if scenario in ("temperature_adc_timeout","temperature_bad_raw","temperature_display_fault"):
                    error={"temperature_adc_timeout":6,"temperature_bad_raw":9,"temperature_display_fault":11}[scenario]
                    assert r[20:24]==(7,error,0,1),r
                    assert not any(f.kind==5 and f.payload[:2]==b'\x07\x06' for f in (frames(m) or []))
                else:
                    assert r[20:24]==(6,0,1,1) and r[25:28]==(2048,245,245),r
                    until(m,lambda:frames(m) is not None and sum(f.kind==5 and f.payload[:2]==b'\x07\x06' for f in frames(m))==1 and m.get(service.PA+0x14)&(1<<11))
                    assert [f.payload for f in frames(m) if f.kind==5 and f.payload[:2]==b'\x07\x06']==[bytes.fromhex("0706040992")]
                    if scenario=="traffic":
                        m.rx.extend(base.encode(base.Frame(0x44,1,bytes.fromhex("07050706"))))
                        until(m,lambda:reply_count(m)==3 and frames(m) is not None)
                        assert frames(m)[-1]==base.Frame(0x44,4,bytes.fromhex("070500040834070600040992"))
                    if scenario=="temperature_repeat":
                        first=r[24]
                        while m.pending_tick: m.step()
                        idle_ticks(m,60001)
                        until(m,lambda:m.report()[23]==2 and m.report()[20]==6)
                        until(m,lambda:frames(m) is not None and sum(f.kind==5 and f.payload[:2]==b'\x07\x06' for f in frames(m))==2 and m.get(service.PA+0x14)&(1<<11))
                        assert m.report()[24]-first>=60000 and len(m.temperature_launches)==10
                        assert [f.sequence for f in frames(m) if f.kind==5 and f.payload[:2]==b'\x07\x06']==[3,4],frames(m)
                assert not r[31] and not m.get(RCC+0x38) and m.get(RCC+8)==0x12
    assert not m.timer_runs and not m.clamps and not m.adc_launches
    assert m.begin_us is None
    if m.runtime_zero:
        motor=m.motor_report()
        if scenario=="bad_trim":
            # Board initialization fails before the motor report is initialized.
            assert motor==(0xa5a5a5a5,)*32
        else:
            expected=(8,9,1) if scenario=="temperature_display_fault" else (0,0,1)
            if m.reason!="fatal": assert motor[2:5]==expected,motor
            assert not motor[13]
    result=m.checks()
    result.update(runtime_initialized=m.runtime_zero,temperature_report=m.report()[19:],
                  temperature_conversions=len(m.temperature_launches),
                  bootloader_emulated=False,normal_controls_complete=False)
    return result


def main():
    out=Path(sys.argv[1]); manifest=json.loads((out/"manifest.json").read_text())
    data=(out/"replacement-core.bin").read_bytes()
    assert manifest["flash_application"] and not manifest["automatic_startup_motion"]
    assert manifest["code_address"]==0x08002000
    assert hashlib.sha256(data).hexdigest()==manifest["bin_sha256"]
    sources={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in
             (Path(__file__),Path(service.__file__),Path(service.display.__file__),
              Path(service.probe.__file__),Path(base.__file__))}
    path=out/"application-thumb-results.json"
    previous=json.loads(path.read_text()) if path.exists() else {}
    cases=previous.get("cases",{}) if previous.get("bin_sha256")==manifest["bin_sha256"] and previous.get("model_sources")==sources else {}
    scenarios=tuple(sys.argv[2:]) or SCENARIOS
    assert all(scenario in SCENARIOS for scenario in scenarios)
    with ProcessPoolExecutor(max_workers=3,mp_context=get_context("spawn")) as pool:
        results=pool.map(run,repeat(data),repeat(manifest["symbols"]),scenarios)
        for scenario,result in zip(scenarios,results):
            cases[scenario]=result
            print("PASS FLASH application actual-linked",scenario,flush=True)
            path.write_text(json.dumps({"complete":set(SCENARIOS)<=set(cases),
                "bin_sha256":manifest["bin_sha256"],"model_sources":sources,
                "hardware_tested":False,"bootloader_emulated":False,"cases":cases},indent=2)+"\n")


if __name__=="__main__": main()
