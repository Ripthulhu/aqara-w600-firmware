"""Execute the linked app through ambiguous-probe recovery; no device access."""
import hashlib
import json
from pathlib import Path
import sys

from unicorn.arm_const import UC_ARM_REG_R1
import application_thumb as app


class FallbackCore(app.ApplicationCore):
    def __init__(self,data,symbols,both_ambiguous):
        self.start_adc=0;self.direction=0;self.both_ambiguous=both_ambiguous
        super().__init__(data,symbols,'normal')

    def code(self,cpu,address,size,user):
        if address in (self.symbols.get('w600_motor_service_begin',-1),
                       self.symbols.get('w600_motor_service_begin_bounded',-1)):
            self.start_adc=len(self.adc_launches)
            self.direction=cpu.reg_read(UC_ARM_REG_R1)
        super().code(cpu,address,size,user)

    def write(self,cpu,access,address,size,value,user):
        super().write(cpu,access,address,size,value,user)
        if address==app.service.ADC and value==4 and self.get(app.service.ADC+0x1c)==0x10000:
            group=(len(self.adc_launches)-1-self.start_adc)//6
            raw=1 if group==0 else 550 if self.both_ambiguous or self.direction==2 else 1550
            at,channel,_=self.adc_launches[-1]
            self.adc_launches[-1]=(at,channel,raw)
            if self.adc_pending is not None:
                self.adc_pending=(self.adc_pending[0],channel,raw)


def run(directory,both_ambiguous):
    manifest=json.loads((directory/'manifest.json').read_text())
    data=(directory/'replacement-core.bin').read_bytes()
    assert hashlib.sha256(data).hexdigest()==manifest['bin_sha256']
    previous=json.loads((Path(__file__).parent/'slow-start-v12-qualified-20261109/manifest.json').read_text())
    for name in ('firmware/valve_calibration.h','replacement-control/motor_calibration.h'):
        assert manifest['sources'][name]==previous['sources'][name]
    m=FallbackCore(data,manifest['symbols'],both_ambiguous);m.boot()
    app.until(m,lambda:m.virtual_us>=250000)
    m.rx.extend(app.base.encode(app.base.Frame(0x71,2,bytes.fromhex('07090001'))))
    state=lambda:m.get(manifest['symbols']['valve']+0x14c)&0xffffff
    app.until(m,lambda:state()&255==4 or (state()&0xffff==0x0301 and len(m.timer_runs)==3),limit=50000)
    observed=state()
    assert observed==(0x080204 if both_ambiguous else 0x0301),hex(observed)
    expected_actions=2 if both_ambiguous else 3
    assert len(m.timer_runs)==expected_actions
    m.rx.extend(app.base.encode(app.base.Frame(0x72,2,bytes.fromhex('07030000'))))
    app.until(m,lambda:m.motor_report()[2] in (7,8) and m.report()[9]>=2)
    final=m.motor_report()
    assert m.off() and final[3]==0 and final[4]==1 and final[10:13]==(0,0,0),final
    assert m.report()[2:4]==(2,0) and len(m.timer_runs)==expected_actions
    assert not (m.get(manifest['symbols']['valve']+0x1a8)&255)
    return dict(bin_sha256=manifest['bin_sha256'],both_ambiguous=both_ambiguous,
        observed_calibration_word=hex(observed),motor_actions=expected_actions,
        motor_stopped=True,calibration_applied=False,hardware_tested=False,
        cycle_accurate=False,model_feedback='close ~895mV; open ~1261mV unless both ambiguous')


def main():
    directory=Path(sys.argv[1]);results=[]
    for both in (False,True):
        result=run(directory,both);results.append(result)
        print(json.dumps(result),flush=True)
    (directory/'fallback-thumb-results.json').write_text(json.dumps(dict(
        complete=True,cases=results,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),indent=2)+'\n')


if __name__=='__main__':main()
