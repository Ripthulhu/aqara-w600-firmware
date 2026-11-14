"""Offline ARM integration test for repeated commands; no hardware access.

Relinks existing objects with the public driver APIs retained, then calls those
real functions as a serialized Thread caller. Normal app/IRQ code keeps running.
This is not evidence of radio-command wiring or a mounted calibration.
"""
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

import application_thumb as app

service=app.service
base=app.base


class RepeatCore(app.ApplicationCore):
    def __init__(self,data,symbols,scenario):
        super().__init__(data,symbols,scenario)
        self.move_adc_start=0
        self.drive_intervals=[]; self.drive_start=None; self.move_drive_base=0

    def track_drive(self):
        pads=self.get(service.PB+0x10)&0xc000
        driving=pads in (0x4000,0x8000)
        if driving and self.drive_start is None: self.drive_start=self.virtual_us
        if not driving and self.drive_start is not None:
            self.drive_intervals.append((self.drive_start,self.virtual_us)); self.drive_start=None

    def advance_us(self,elapsed):
        super().advance_us(elapsed)
        self.track_drive()

    def drive_us(self):
        return sum(end-start for start,end in self.drive_intervals)+(self.virtual_us-self.drive_start if self.drive_start is not None else 0)

    def write(self,cpu,access,address,size,value,user):
        super().write(cpu,access,address,size,value,user)
        self.track_drive()
        if address==service.ADC and value==4 and self.get(service.ADC+0x1c)==0x10000:
            group=(len(self.adc_launches)-1-self.move_adc_start)//6
            raw=1 if group==0 or group>=21 else 1700
            at,channel,_=self.adc_launches[-1]
            self.adc_launches[-1]=(at,channel,raw)
            if self.adc_pending is not None:
                self.adc_pending=(self.adc_pending[0],channel,raw)

    def call(self,name,*args):
        while self.reason=="irq_boundary": self.step()
        assert self.reason=="foreground_boundary" and not self.in_handler
        saved=self.cpu.context_save()
        self.cpu.reg_write(base.UC_ARM_REG_MSP,self.cpu.reg_read(base.UC_ARM_REG_SP)-64)
        self.cpu.reg_write(base.UC_ARM_REG_PRIMASK,1)
        self.cpu.reg_write(base.UC_ARM_REG_XPSR,0x01000000)
        self.cpu.reg_write(base.UC_ARM_REG_LR,base.SENTINEL|1)
        from unicorn.arm_const import UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3
        assert len(args)<=4
        for register,value in zip((UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3),args):
            self.cpu.reg_write(register,value)
        self.in_handler=True
        self.execute(self.symbols[name])
        assert self.reason=="handler_return"
        result=self.cpu.reg_read(UC_ARM_REG_R0)
        self.in_handler=False
        self.cpu.context_restore(saved)
        self.reason="foreground_boundary"
        return result

    def begin(self,direction):
        self.direction=direction
        self.move_adc_start=len(self.adc_launches)
        self.move_drive_base=self.drive_us()
        assert self.call("w600_motor_service_begin",self.symbols["motor"],direction)==1
        # The foreground report still contains the previous command until polled.
        app.until(self,lambda:1<=self.motor_report()[2]<=5 and self.motor_report()[14]==direction)

    def stopped_wait(self):
        start=self.virtual_us
        app.until(self,lambda:self.virtual_us-start>=102000)

    def time_snapshot(self,valid=True):
        import struct
        accepted=self.call("w600_motor_service_time",self.symbols["motor"],0x20006b00)
        words=struct.unpack("<5I",self.cpu.mem_read(0x20006b00,20))
        if not valid:
            assert not accepted and words==(0,)*5
            return None
        assert accepted
        _,lower,upper,sample_lower,sample_upper=words
        actual=self.drive_us()-self.move_drive_base
        assert lower*100<=actual<=upper*100,(words,actual,self.drive_intervals)
        assert sample_lower<=lower and sample_upper<=upper
        return words


def run(data,symbols,fault=False):
    m=RepeatCore(data,symbols,"adc_timeout" if fault else "normal")
    m.boot()
    app.until(m,lambda:m.virtual_us>=250000)
    motor=symbols["motor"]
    m.time_snapshot(False)
    assert not m.call("w600_motor_service_rearm",motor)
    m.begin(2)
    app.until(m,lambda:m.motor_report()[2] in (7,8))
    first_trace=m.checks().get("motor_trace")
    if fault:
        assert m.motor_report()[2]==8 and m.motor_report()[3]
        before=bytes(m.cpu.mem_read(motor,256))
        assert not m.call("w600_motor_service_rearm",motor)
        assert bytes(m.cpu.mem_read(motor,256))==before and len(m.timer_runs)==1
        m.time_snapshot(False)
    else:
        assert m.motor_report()[2:4]==(7,0) and m.motor_report()[7]==3
        first_time=m.time_snapshot()
        assert first_time[2]*100<m.virtual_us-m.timer_runs[0]["started_at_us"]
        assert not m.call("w600_motor_service_rearm",motor)
        m.stopped_wait()
        assert m.call("w600_motor_service_rearm",motor)==1
        m.time_snapshot(False)
        m.begin(1)
        app.until(m,lambda:len(m.clamps)>=27)
        m.time_snapshot()
        assert not m.call("w600_motor_service_rearm",motor)
        assert m.call("w600_motor_service_finish",motor)==1
        assert m.off() and m.selectors_low()
        app.until(m,lambda:m.motor_report()[2]==7)
        second_trace=m.checks().get("motor_trace")
        if first_trace is not None:
            assert second_trace is not None and second_trace["header"][3]!=first_trace["header"][3]
            assert second_trace["header"][4]==1 and second_trace["samples"]<first_trace["samples"]
        m.time_snapshot()
        assert m.motor_report()[3]==0 and m.motor_report()[7]!=3
        assert not m.call("w600_motor_service_finish",motor)
        m.stopped_wait()
        assert m.call("w600_motor_service_rearm",motor)==1
        m.begin(2)
        app.until(m,lambda:m.motor_report()[2] in (7,8))
        assert m.motor_report()[2:4]==(7,0) and m.motor_report()[7]==3,m.motor_report()
        m.time_snapshot()
        assert len(m.timer_runs)==3
    result=m.checks()
    result["three_commands_completed"]=not fault
    result["failed_attempt_not_rearmed"]=fault
    result["radio_command_integration_tested"]=False
    result["last_command_modeled_drive_us"]=m.drive_us()-m.move_drive_base
    result["commanded_on_time_brackets_checked"]=not fault
    return result


def main():
    root=Path(__file__).resolve().parent
    out=Path(sys.argv[1])
    objects=sorted(out.glob("*.o")); assert objects
    elf=out/"motor-api-test.elf"; binary=out/"motor-api-test.bin"
    exports=("w600_motor_service_begin","w600_motor_service_finish","w600_motor_service_rearm","w600_motor_service_time")
    subprocess.check_call(["arm-none-eabi-gcc","-mcpu=cortex-m0plus","-mthumb","-nostdlib","-nostartfiles",
        "-Wl,--build-id=none","-Wl,--gc-sections","-Wl,--fatal-warnings",
        *("-Wl,--undefined="+name for name in exports),"-T",str(root/"flash-app.ld"),
        *map(str,objects),"-lgcc","-o",str(elf)])
    subprocess.check_call(["arm-none-eabi-objcopy","-O","binary",str(elf),str(binary)])
    symbols={}
    for line in subprocess.check_output(["arm-none-eabi-nm","-n",str(elf)],text=True).splitlines():
        match=re.fullmatch(r"([0-9a-fA-F]+)\s+\w\s+(\S+)",line)
        if match: symbols[match[2]]=int(match[1],16)
    assert all(name in symbols for name in exports)
    data=binary.read_bytes()
    results={}
    for name,fault in (("repeat_commands",False),("latched_feedback_fault",True)):
        results[name]=run(data,symbols,fault)
        print("PASS actual-linked motor API",name,flush=True)
    sources=(Path(__file__),Path(app.__file__),Path(service.__file__),
             Path(service.display.__file__),Path(service.probe.__file__),Path(base.__file__))
    (out/"motor-api-thumb-results.json").write_text(json.dumps({"complete":True,
        "bin_sha256":hashlib.sha256(data).hexdigest(),"binary_bytes":len(data),
        "objects_sha256":{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in objects},
        "model_sources":{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
        "test_link_only":True,"hardware_tested":False,"cases":results},indent=2)+"\n")


if __name__=="__main__": main()
