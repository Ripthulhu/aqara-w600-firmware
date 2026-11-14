"""One-byte UART stress at the nonzero candidate stops and three rearms."""
import hashlib,json,sys
from pathlib import Path
import application_thumb as app
from test_loaded_home_thumb import LoadedCore
from uart_wire_stress import WireCore

class LoadedWire(LoadedCore,WireCore):
    def __init__(self,data,symbols):
        self.targets=[];self.target_keys=set()
        super().__init__(data,symbols)
    def code(self,cpu,address,size,user):
        actions=len(self.timer_runs)
        kind='stop' if address==self.symbols['request_stop'] and actions in (1,3) else \
             'rearm' if address==self.symbols['w600_motor_service_rearm'] and actions in (1,2,3) else None
        key=(kind,actions)
        if kind and key not in self.target_keys:
            self.target_keys.add(key);seq=0x80+len(self.targets)
            self.targets.append(dict(kind=kind,actions=actions,sequence=seq,at_us=self.virtual_us))
            self.schedule(app.base.Frame(seq,1,bytes.fromhex('070b')),self.virtual_us)
        super().code(cpu,address,size,user)

def main():
    out=Path(sys.argv[1]);manifest=json.loads((out/'manifest.json').read_text())
    data=(out/'replacement-core.bin').read_bytes();assert hashlib.sha256(data).hexdigest()==manifest['bin_sha256']
    m=LoadedWire(data,manifest['symbols']);m.boot();app.until(m,lambda:m.virtual_us>=250000)
    m.schedule(app.base.Frame(0x70,2,bytes.fromhex('07090001')),m.virtual_us+1)
    app.until(m,lambda:len(m.timer_runs)==4,limit=90000)
    assert len(m.targets)==5 and not m.overruns
    at=m.virtual_us;app.until(m,lambda:m.virtual_us>=at+100000)
    replies=app.frames(m);assert replies is not None
    received={f.sequence for f in replies if f.kind==4}
    assert {r['sequence'] for r in m.targets}<=received
    layout=manifest['test_confirmation_layout'];field=lambda i:m.get(manifest['symbols']['valve']+layout[i])
    assert field(11)==1 and field(18)>100 and not(field(22)&255)
    m.schedule(app.base.Frame(0x71,2,bytes.fromhex('07030000')),m.virtual_us+1)
    app.until(m,lambda:m.off() and m.motor_report()[2]==7)
    at=m.virtual_us;app.until(m,lambda:m.virtual_us>=at+250000)
    assert not m.overruns and m.off() and len(m.timer_runs)==4 and m.motor_report()[3:5]==(0,1)
    result=dict(complete=True,bin_sha256=manifest['bin_sha256'],targets=m.targets,overruns=m.overruns,
        max_receive_age_us=m.max_receive_age,received_sequences=sorted(received),return_mv=field(18),
        stopped=True,calibration_applied=False,state_seeding=False,hardware_tested=False,cycle_accurate=False,
        sources={name:hashlib.sha256((Path(__file__).parent/name).read_bytes()).hexdigest() for name in
                 ('test_loaded_wire_thumb.py','test_loaded_home_thumb.py','uart_wire_stress.py','sampling_stress.py',
                  'application_thumb.py','service_thumb.py','display_thumb.py','probe_thumb.py','test_thumb.py')})
    (out/'loaded-wire-result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)
if __name__=='__main__':main()
