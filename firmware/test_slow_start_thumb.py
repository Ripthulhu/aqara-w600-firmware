"""Actual linked startup probe with delayed steady feedback; no device access."""
import hashlib
import json
from pathlib import Path
import sys

import application_thumb as app


class SlowStartCore(app.ApplicationCore):
    def __init__(self,data,symbols):
        self.start_adc=0
        super().__init__(data,symbols,'normal')

    def code(self,cpu,address,size,user):
        if address in (self.symbols.get('w600_motor_service_begin',-1),
                       self.symbols.get('w600_motor_service_begin_bounded',-1)):
            self.start_adc=len(self.adc_launches)
        super().code(cpu,address,size,user)

    def write(self,cpu,access,address,size,value,user):
        super().write(cpu,access,address,size,value,user)
        if address==app.service.ADC and value==4 and self.get(app.service.ADC+0x1c)==0x10000:
            group=(len(self.adc_launches)-1-self.start_adc)//6
            # Raw550 is ~447mV, below the loaded-motion gate;1550 is ~1261mV.
            raw=1 if group==0 else 550 if group<=10 else 1550
            at,channel,_=self.adc_launches[-1]
            self.adc_launches[-1]=(at,channel,raw)
            if self.adc_pending is not None:
                self.adc_pending=(self.adc_pending[0],channel,raw)


def main():
    directory=Path(sys.argv[1]);reject=sys.argv[2:]==['--expect-reject']
    assert sys.argv[2:] in ([],['--expect-reject'])
    manifest=json.loads((directory/'manifest.json').read_text())
    data=(directory/'replacement-core.bin').read_bytes()
    assert hashlib.sha256(data).hexdigest()==manifest['bin_sha256']
    old=json.loads((Path(__file__).parent/'uart-split-v11-20261109/manifest.json').read_text())
    for name in ('firmware/valve_calibration.h','replacement-control/motor_calibration.h'):
        assert manifest['sources'][name]==old['sources'][name]
    m=SlowStartCore(data,manifest['symbols']);m.boot()
    app.until(m,lambda:m.virtual_us>=250000)
    m.rx.extend(app.base.encode(app.base.Frame(0x71,2,bytes.fromhex('07090001'))))
    state=lambda:m.get(manifest['symbols']['valve']+0x14c)&0xffffff
    app.until(m,lambda:state()&255==4 or (state()&0xffff==0x0301 and len(m.timer_runs)==2),limit=50000)
    observed=state();report=m.motor_report()
    if reject:
        assert observed==0x080104 and len(m.timer_runs)==1,(hex(observed),report)
    else:
        assert observed==0x0301 and len(m.timer_runs)==2,(hex(observed),report)
    m.rx.extend(app.base.encode(app.base.Frame(0x72,2,bytes.fromhex('07030000'))))
    app.until(m,lambda:m.motor_report()[2] in (7,8) and m.report()[9]>=2)
    final=m.motor_report()
    assert m.off() and final[3]==0 and final[4]==1 and final[10:13]==(0,0,0),final
    assert m.report()[2:4]==(2,0) and len(m.timer_runs)==(1 if reject else 2)
    result=dict(bin_sha256=manifest['bin_sha256'],expected_old_rejection=reject,
        observed_calibration_word=hex(observed),probe_qualified=not reject,
        motor_actions=len(m.timer_runs),motor_stopped=True,driver_error=final[3],
        model_feedback='~895mV through200ms, then~1261mV',
        model_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        hardware_tested=False,cycle_accurate=False)
    (directory/'slow-start-thumb-results.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)


if __name__=='__main__':main()
