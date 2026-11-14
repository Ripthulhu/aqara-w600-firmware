"""Check actual linked probe duration before reversal, then cancel cleanly."""
import hashlib
import json
from pathlib import Path
import sys

import application_thumb as app


class FreeProbeCore(app.ApplicationCore):
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
            raw=1 if group==0 else 1700
            at,channel,_=self.adc_launches[-1]
            self.adc_launches[-1]=(at,channel,raw)
            if self.adc_pending is not None:
                self.adc_pending=(self.adc_pending[0],channel,raw)


def main():
    directory=Path(sys.argv[1]);short=sys.argv[2:]==['--expect-short']
    assert sys.argv[2:] in ([],['--expect-short'])
    manifest=json.loads((directory/'manifest.json').read_text())
    data=(directory/'replacement-core.bin').read_bytes()
    assert hashlib.sha256(data).hexdigest()==manifest['bin_sha256']
    m=FreeProbeCore(data,manifest['symbols']);m.boot()
    app.until(m,lambda:m.virtual_us>=250000)
    m.rx.extend(app.base.encode(app.base.Frame(0x71,2,bytes.fromhex('07090001'))))
    app.until(m,lambda:len(m.timer_runs)==2,limit=50000)
    spacing=m.timer_runs[1]['started_at_us']-m.timer_runs[0]['started_at_us']
    assert (spacing<800000 if short else 1600000<=spacing<=2000000),spacing
    m.rx.extend(app.base.encode(app.base.Frame(0x72,2,bytes.fromhex('07030000'))))
    app.until(m,lambda:m.motor_report()[2] in (7,8) and m.report()[9]>=2)
    final=m.motor_report()
    assert m.off() and final[3]==0 and final[4]==1 and final[10:13]==(0,0,0),final
    assert m.report()[2:4]==(2,0) and len(m.timer_runs)==2
    result=dict(bin_sha256=manifest['bin_sha256'],expected_old_short_probe=short,
        first_drive_to_next_start_us=spacing,motor_actions=2,motor_stopped=True,
        driver_error=final[3],hardware_tested=False,cycle_accurate=False,
        timing_scope='model start-to-start interval, includes drive sampling gaps and stopped rest',
        model_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (directory/'probe-escape-thumb-results.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)


if __name__=='__main__':main()
