"""Run the real slow-clock checker on a modeled RCL/LPTIM2, never hardware."""
import hashlib
import json
from pathlib import Path
import struct
import sys
import application_thumb as app

service=app.service
base=app.base
RCC=service.RCC
LP=0x40009000
GATE=1<<30


class ClockCore(app.ApplicationCore):
    def __init__(self,data,symbols,scenario,hz=32000):
        self.rcl_ready_at=None;self.low_started_at=None;self.low_enabled_at=None
        self.low_running=False;self.low_fraction=0;self.low_hz=hz;self.low_writes=[]
        super().__init__(data,symbols,scenario)
        self.put(RCC+0x44,0x1c000000);self.put(RCC+0x50,9)
        self.put(0x1fff03d0,0xfff60009)
        self.reset_low()

    def reset_low(self):
        for offset in range(0,0x20,4): self.put(LP+offset,0)
        self.put(LP+0x18,1)
        self.low_running=False;self.low_fraction=0

    def advance_us(self,elapsed):
        old=self.virtual_us
        super().advance_us(elapsed)
        if self.rcl_ready_at is not None and self.virtual_us>=self.rcl_ready_at:
            self.put(RCC+0x44,self.get(RCC+0x44)|2);self.rcl_ready_at=None
        if self.low_running:
            assert self.get(RCC+0x34)&GATE and self.get(RCC+0x3c)==0x100000
            assert self.get(RCC+0x44)&3==3 and self.get(LP+0x10)&1
            active=max(0,self.virtual_us-max(old,self.low_started_at))
            count,self.low_fraction=divmod(self.low_fraction+active*self.low_hz,1000000)
            total=self.get(LP+0x1c)+count
            if total>65535:self.put(LP,self.get(LP)|2)
            self.put(LP+0x1c,total&65535)

    def read(self,cpu,access,address,size,value,user):
        if LP<=address<LP+0x20:
            assert size==4 and self.get(RCC+0x34)&GATE
            return
        super().read(cpu,access,address,size,value,user)

    def write(self,cpu,access,address,size,value,user):
        if address==RCC+0x44:
            assert size==4 and not (value^self.get(address))&~0x10001
            if value&1:self.rcl_ready_at=self.virtual_us+100
            else:
                self.rcl_ready_at=None
                self.pending.append((address,value&~2))
            return
        if address==RCC+0x3c:
            assert size==4 and value in (0,0x100000)
            return
        if address==RCC+0x34 and (value^self.get(address))&GATE:
            assert size==4 and not (value^self.get(address))&~GATE
            return
        if address==RCC+0x24 and (value^self.get(address))&GATE:
            assert size==4 and not (value^self.get(address))&~GATE
            if value&GATE:self.reset_low()
            return
        if LP<=address<LP+0x20:
            assert size==4 and self.get(RCC+0x34)&GATE
            assert address-LP in (8,12,16,24)
            self.low_writes.append((self.virtual_us,address,value))
            if address==LP+16:
                if value==1:self.low_enabled_at=self.virtual_us
                if value&4:
                    self.low_running=True;self.low_fraction=0
                    self.low_started_at=self.virtual_us+64
                    self.pending.append((address,value&~4))
                if not value&1:self.low_running=False
            if address==LP+24:
                assert self.get(LP+16)&1 and self.virtual_us-self.low_enabled_at>=1000
            return
        if address==service.TIM and value&1:
            assert not self.get(RCC+0x34)&GATE and not self.get(RCC+0x44)&1
        super().write(cpu,access,address,size,value,user)

    def clock_report(self):
        return struct.unpack('<13I',self.cpu.mem_read(self.symbols['sleep_clock'],52))


def released(m):
    c=m.clock_report()
    assert c[2:4]==(0,0) and m.get(RCC+0x34)==0x60000
    assert m.get(RCC+0x3c)==0 and m.get(RCC+0x44)&3==0 and not m.low_running


def run(data,symbols,scenario):
    m=ClockCore(data,symbols,scenario,70000 if scenario=='rate_fault' else 32000)
    if scenario=='retained_rcl':
        m.put(RCC+0x44,0x1c000003);m.put(RCC+0x40,0x200008)
    m.boot()
    if scenario in ('command','calibration'):
        app.until(m,lambda:m.clock_report()[0]==4)
        payload=bytes.fromhex('07000407d0') if scenario=='command' else bytes.fromhex('07090001')
        m.rx.extend(base.encode(base.Frame(0x60,2,payload)))
        if scenario=='command':
            app.until(m,lambda:app.reply_count(m)==1 and app.frames(m) is not None and
                      m.get(service.PA+0x14)&(1<<11))
            released(m)
            assert m.clock_report()[:2]==(0,0) and not m.timer_runs
        else:
            app.until(m,lambda:len(m.timer_runs)==1 and m.motor_report()[5]>=3)
            assert not m.get(RCC+0x34)&GATE and not m.get(RCC+0x44)&1
            m.rx.extend(base.encode(base.Frame(0x61,2,bytes.fromhex('07030000'))))
            app.until(m,lambda:m.motor_report()[2]==7 and m.motor_report()[10:13]==(0,0,0) and
                      app.frames(m) is not None and m.get(service.PA+0x14)&(1<<11))
            released(m)
            assert len(m.timer_runs)==1 and m.off()
    else:
        app.until(m,lambda:m.clock_report()[0] in (5,6))
        c=m.clock_report();released(m)
        if scenario in ('healthy','retained_rcl'):
            assert c[0:4]==(5,0,0,0) and c[10]==2 and 31900<=c[9]<=32100
            app.until(m,lambda:m.report()[23]>=1 and m.report()[20]==6)
            assert m.report()[22]==1 and not m.timer_runs
        else:
            assert scenario=='rate_fault' and c[:4]==(6,6,0,0) and c[9]==0
            m.rx.extend(base.encode(base.Frame(0x60,2,bytes.fromhex('07000407d0'))))
            app.until(m,lambda:app.reply_count(m)==1 and app.frames(m) is not None and
                      m.get(service.PA+0x14)&(1<<11))
            assert m.report()[2:4]==(2,0) and not m.timer_runs
    result=m.checks()
    result.update(slow_clock_case=scenario,slow_clock_report=m.clock_report(),
                  modeled_rcl_hz=m.low_hz,slow_clock_hardware_tested=False,
                  low_timer_writes=len(m.low_writes))
    return result


if __name__=='__main__':
    out=Path(sys.argv[1]);manifest=json.loads((out/'manifest.json').read_text())
    data=(out/'replacement-core.bin').read_bytes()
    assert manifest['slow_clock_rate_check_enabled'] and not manifest['deep_sleep_enabled']
    assert hashlib.sha256(data).hexdigest()==manifest['bin_sha256']
    cases={}
    expected={'healthy','retained_rcl','command','calibration','rate_fault'}
    for scenario in tuple(sys.argv[2:]) or ('healthy','retained_rcl','command','calibration','rate_fault'):
        assert scenario in expected
        cases[scenario]=run(data,manifest['symbols'],scenario)
        print('PASS actual-linked slow-clock checker',scenario,flush=True)
        (out/'sleep-clock-thumb-results.json').write_text(json.dumps(dict(
            complete=set(cases)==expected,
            bin_sha256=manifest['bin_sha256'],cases=cases,hardware_tested=False,
            model_sources={Path(p).name:hashlib.sha256(Path(p).read_bytes()).hexdigest()
                for p in (__file__,app.__file__,service.__file__,service.display.__file__,
                          service.probe.__file__,base.__file__)}),indent=2)+'\n')
