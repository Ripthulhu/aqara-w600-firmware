"""Actual-linked same-direction HOME, fallback, early endpoint and cancellation."""
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from itertools import repeat
from multiprocessing import get_context
from pathlib import Path
import sys

import application_thumb as app
from test_confirmation_fault_thumb import ConfirmationCore


class HomeCore(ConfirmationCore):
    def __init__(self,data,symbols,kind):
        self.kind=kind
        self.actions=ConfirmationCore.actions+(((1,45000),) if kind in ('fallback','both_ambiguous') else ())
        super().__init__(data,symbols)

    def sample_raw(self,group):
        if len(self.timer_runs)<=3:return super().sample_raw(group)
        if not group:return 1
        if self.kind=='slow_start':return 550 if group<=10 else 1550
        if self.kind in ('fallback','both_ambiguous'):
            return 550 if self.kind=='both_ambiguous' or self.direction==2 else 1550
        if self.kind=='early_endpoint' and group>=22:return 1
        return 1700


def run(directory,kind):
    manifest=json.loads((directory/'manifest.json').read_text())
    data=(directory/'replacement-core.bin').read_bytes()
    assert hashlib.sha256(data).hexdigest()==manifest['bin_sha256']
    applied_at=manifest['symbols']['valve']+manifest['test_control_layout'][1]
    layout=manifest['test_confirmation_layout']
    assert len(layout)==23 and layout[0]==1 and layout[22]==manifest['test_control_layout'][1]
    m=HomeCore(data,manifest['symbols'],kind)
    m.boot();app.until(m,lambda:m.virtual_us>=250000)
    m.rx.extend(app.base.encode(app.base.Frame(0x71,2,bytes.fromhex('07090001'))))
    state=lambda:sum(int.from_bytes(m.cpu.mem_read(manifest['symbols']['valve']+offset,1),'little')<<(8*i)
                     for i,offset in enumerate(layout[3:6]))
    if kind=='both_ambiguous':
        app.until(m,lambda:state()&255==4,limit=150000)
        assert state()==0x080204 and len(m.timer_runs)==5,(hex(state()),m.motor_report())
        expected_actions=5
    elif kind=='early_endpoint':
        app.until(m,lambda:state()&255==4,limit=150000)
        app.until(m,lambda:m.motor_report()[2]==7 and m.off())
        # An abrupt qualified-motion low stops immediately, without confirming HOME.
        assert m.motor_report()[14]==2 and not (m.get(applied_at)&255)
        assert len(m.timer_runs)==4 and m.motor_report()[5]<=24
        expected_actions=4
    else:
        expected_actions=5 if kind=='fallback' else 4
        app.until(m,lambda:state()&255==4 or
                  (len(m.timer_runs)==expected_actions and state()&0xffff==0x0301),limit=150000)
        assert state()==0x0301 and len(m.timer_runs)==expected_actions,(hex(state()),m.motor_report())
        direction=1 if kind=='fallback' else 2
        assert m.motor_report()[14]==direction
        at=m.virtual_us+700000
        app.until(m,lambda:m.virtual_us>=at)
        assert state()==0x0301 and len(m.timer_runs)==expected_actions and not m.off()
    observed=state()
    assert [run['arr'] for run in m.timer_runs[:3]]==[44999,3199,10199]
    assert {1,2,3}<=m.stop_proofs and m.low_batches>=2
    m.rx.extend(app.base.encode(app.base.Frame(0x72,2,bytes.fromhex('07030000'))))
    app.until(m,lambda:m.motor_report()[2] in (7,8) and app.reply_count(m)>=2)
    final=m.motor_report()
    assert m.off() and final[3]==0 and final[4]==1 and final[10:13]==(0,0,0),final
    assert m.report()[2:4]==(2,0) and len(m.timer_runs)==expected_actions
    assert not (m.get(applied_at)&255)
    return dict(scenario=kind,bin_sha256=manifest['bin_sha256'],observed_calibration_word=hex(observed),
        preparation_actions=3,motor_actions=expected_actions,motor_stopped=True,calibration_applied=False,
        foreground_clock_step_us=25,foreground_dispatch_us=500,hardware_tested=False,cycle_accurate=False)


def main():
    directory=Path(sys.argv[1]);results=[]
    kinds=('steady','slow_start','fallback','both_ambiguous','early_endpoint')
    with ProcessPoolExecutor(max_workers=3,mp_context=get_context("spawn")) as pool:
        for result in pool.map(run,repeat(directory),kinds):
            results.append(result);print(json.dumps(result),flush=True)
    sources=('test_continuous_home_thumb.py','test_confirmation_fault_thumb.py',
             'application_thumb.py','service_thumb.py','display_thumb.py',
             'probe_thumb.py','test_thumb.py')
    output=dict(complete=True,cases=results,model_sources={name:hashlib.sha256(
        (Path(__file__).parent/name).read_bytes()).hexdigest() for name in sources})
    (directory/'continuous-home-thumb-results.json').write_text(json.dumps(output,indent=2)+'\n')


if __name__=='__main__':main()
