"""Actual-linked same-direction HOME, fallback, early endpoint and cancellation."""
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from itertools import repeat
from multiprocessing import get_context
from pathlib import Path
import sys

import application_thumb as app
from test_fallback_thumb import FallbackCore
from test_probe_escape_thumb import FreeProbeCore
from test_slow_start_thumb import SlowStartCore


class EarlyEndpointCore(FreeProbeCore):
    def __init__(self,data,symbols):
        self.low_batch_started=False
        super().__init__(data,symbols)

    def write(self,cpu,access,address,size,value,user):
        if address in (app.service.TIM+0x34,app.service.TIM+0x38) and value==self.motor_limit_ms:
            assert not self.low_batch_started,'Drive restored after abrupt qualified-motion low'
        super().write(cpu,access,address,size,value,user)
        if address==app.service.ADC and value==4 and self.get(app.service.ADC+0x1c)==0x10000:
            group=(len(self.adc_launches)-1-self.start_adc)//6
            raw=1700 if 0<group<22 else 1
            if group>=22:self.low_batch_started=True
            at,channel,_=self.adc_launches[-1]
            self.adc_launches[-1]=(at,channel,raw)
            if self.adc_pending is not None:
                self.adc_pending=(self.adc_pending[0],channel,raw)


def run(directory,kind):
    manifest=json.loads((directory/'manifest.json').read_text())
    data=(directory/'replacement-core.bin').read_bytes()
    assert hashlib.sha256(data).hexdigest()==manifest['bin_sha256']
    applied_at=manifest['symbols']['valve']+manifest['test_control_layout'][1]
    layout=manifest['test_confirmation_layout']
    assert len(layout)==23 and layout[0]==1 and layout[22]==manifest['test_control_layout'][1]
    cls={'steady':FreeProbeCore,'slow_start':SlowStartCore,'early_endpoint':EarlyEndpointCore}.get(kind)
    m=cls(data,manifest['symbols']) if cls else FallbackCore(data,manifest['symbols'],kind=='both_ambiguous')
    m.boot();app.until(m,lambda:m.virtual_us>=250000)
    m.rx.extend(app.base.encode(app.base.Frame(0x71,2,bytes.fromhex('07090001'))))
    state=lambda:sum(int.from_bytes(m.cpu.mem_read(manifest['symbols']['valve']+offset,1),'little')<<(8*i)
                     for i,offset in enumerate(layout[3:6]))
    if kind=='both_ambiguous':
        app.until(m,lambda:state()&255==4,limit=50000)
        assert state()==0x080204 and len(m.timer_runs)==2,(hex(state()),m.motor_report())
        expected_actions=2
    elif kind=='early_endpoint':
        app.until(m,lambda:state()&255==4,limit=50000)
        app.until(m,lambda:m.motor_report()[2]==7 and m.off())
        # An abrupt qualified-motion low stops immediately, without confirming HOME.
        assert m.motor_report()[14]==2 and not (m.get(applied_at)&255)
        assert len(m.timer_runs)==1 and m.motor_report()[5]<=24
        expected_actions=1
    else:
        expected_actions=2 if kind=='fallback' else 1
        app.until(m,lambda:state()&255==4 or state()&0xffff==0x0301,limit=50000)
        assert state()==0x0301 and len(m.timer_runs)==expected_actions,(hex(state()),m.motor_report())
        direction=1 if kind=='fallback' else 2
        assert m.motor_report()[14]==direction
        at=m.virtual_us+700000
        app.until(m,lambda:m.virtual_us>=at)
        assert state()==0x0301 and len(m.timer_runs)==expected_actions and not m.off()
    observed=state()
    m.rx.extend(app.base.encode(app.base.Frame(0x72,2,bytes.fromhex('07030000'))))
    app.until(m,lambda:m.motor_report()[2] in (7,8) and m.report()[9]>=2)
    final=m.motor_report()
    assert m.off() and final[3]==0 and final[4]==1 and final[10:13]==(0,0,0),final
    assert m.report()[2:4]==(2,0) and len(m.timer_runs)==expected_actions
    assert not (m.get(applied_at)&255)
    return dict(scenario=kind,bin_sha256=manifest['bin_sha256'],observed_calibration_word=hex(observed),
        motor_actions=expected_actions,motor_stopped=True,calibration_applied=False,
        hardware_tested=False,cycle_accurate=False)


def main():
    directory=Path(sys.argv[1]);results=[]
    kinds=('steady','slow_start','fallback','both_ambiguous','early_endpoint')
    with ProcessPoolExecutor(max_workers=3,mp_context=get_context("spawn")) as pool:
        for result in pool.map(run,repeat(directory),kinds):
            results.append(result);print(json.dumps(result),flush=True)
    sources=('test_continuous_home_thumb.py','test_fallback_thumb.py','test_probe_escape_thumb.py',
             'test_slow_start_thumb.py','application_thumb.py','service_thumb.py','display_thumb.py',
             'probe_thumb.py','test_thumb.py')
    output=dict(complete=True,cases=results,model_sources={name:hashlib.sha256(
        (Path(__file__).parent/name).read_bytes()).hexdigest() for name in sources})
    (directory/'continuous-home-thumb-results.json').write_text(json.dumps(output,indent=2)+'\n')


if __name__=='__main__':main()
