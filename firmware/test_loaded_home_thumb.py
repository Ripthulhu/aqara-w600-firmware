"""Actual linked app: nonzero load candidates, confirmation, UART cancellation.

No position/state seeding. Synthetic ADC waveform, not mechanical proof.
"""
import hashlib,json,sys
from pathlib import Path
from unicorn.arm_const import UC_ARM_REG_R1,UC_ARM_REG_R2
from sampling_stress import CostedCore
import application_thumb as app

class LoadedCore(CostedCore):
    def __init__(self,data,symbols):
        self.move_adc_start=0;self.group=0
        super().__init__(data,symbols,3)
    def code(self,cpu,address,size,user):
        if address==self.symbols['w600_motor_service_begin_bounded']:
            index=len(self.timer_runs)
            assert index<4
            direction,cap=((2,45000),(1,2000),(2,5000),(1,45000))[index]
            assert cpu.reg_read(UC_ARM_REG_R1)==direction and cpu.reg_read(UC_ARM_REG_R2)==cap
            self.direction=direction;self.motor_limit_ms=cap
            self.move_adc_start=len(self.adc_launches);self.group=0
        super().code(cpu,address,size,user)
    def write(self,cpu,access,address,size,value,user):
        index=len(self.timer_runs)
        threshold=100 if index==1 else 90
        if address in (app.service.TIM+0x34,app.service.TIM+0x38) and value==self.motor_limit_ms:
            assert index not in (1,3) or self.group<threshold+10,'Drive restored after sustained loaded candidate'
        super().write(cpu,access,address,size,value,user)
        if address==app.service.ADC and value==4 and self.get(app.service.ADC+0x1c)==0x10000:
            self.group=(len(self.adc_launches)-1-self.move_adc_start)//6
            raw=1 if self.group==0 else 1700
            if index in (1,3) and self.group>=threshold:raw=1300
            at,channel,_=self.adc_launches[-1];self.adc_launches[-1]=(at,channel,raw)
            if self.adc_pending is not None:self.adc_pending=(self.adc_pending[0],channel,raw)

def main():
    directory=Path(sys.argv[1]);mfest=json.loads((directory/'manifest.json').read_text())
    data=(directory/'replacement-core.bin').read_bytes();assert hashlib.sha256(data).hexdigest()==mfest['bin_sha256']
    m=LoadedCore(data,mfest['symbols']);m.boot();app.until(m,lambda:m.virtual_us>=250000)
    m.rx.extend(app.base.encode(app.base.Frame(0x61,2,bytes.fromhex('07090001'))))
    app.until(m,lambda:len(m.timer_runs)==4,limit=90000)
    layout=mfest['test_confirmation_layout'];field=lambda i:m.get(mfest['symbols']['valve']+layout[i])
    assert [x['arr'] for x in m.timer_runs]==[44999,1999,4999,44999]
    assert field(11)==1 and field(17)>0 and field(18)>100 and not(field(22)&255)
    replies=m.report()[9]
    m.rx.extend(app.base.encode(app.base.Frame(0x62,2,bytes.fromhex('07030000'))))
    app.until(m,lambda:m.off() and m.motor_report()[2]==7 and m.report()[9]==replies+1)
    at=m.virtual_us;app.until(m,lambda:m.virtual_us>=at+250000)
    assert m.off() and m.motor_report()[3:5]==(0,1) and len(m.timer_runs)==4
    result=dict(complete=True,bin_sha256=mfest['bin_sha256'],confirmed_home_then_full_leg=True,
                return_mv=field(18),calibration_applied=False,stopped=True,uart_off_accepted=True,
                hard_caps=[45000,2000,5000,45000],virtual_us=m.virtual_us,instructions=m.instructions,
                state_seeding=False,hardware_tested=False,cycle_accurate=False)
    (directory/'loaded-home-result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)
if __name__=='__main__':main()
