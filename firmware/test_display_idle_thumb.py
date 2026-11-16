"""Execute screen timeout and wake paths from the actual compiled image, no hardware."""
import hashlib
import json
from pathlib import Path
import sys
import application_thumb as app

service=app.service
base=app.base


def settled(m):
    app.until(m,lambda:m.display_report()[2]==0 and not m.pending_tick and
              m.get(service.PA+0x14)&(1<<11) and app.frames(m) is not None)


def run(data,symbols,scenario):
    m=app.ApplicationCore(data,symbols,scenario)
    m.boot()
    app.until(m,lambda:m.display_report()[3:5]==(1,1))
    settled(m)
    start=m.virtual_us
    app.until(m,lambda:m.virtual_us-start>=50000)
    settled(m)
    if scenario!='unknown':
        m.rx.extend(base.encode(base.Frame(0x60,2,bytes.fromhex('07000407d0'))))
        app.until(m,lambda:m.report()[9]==1)
        settled(m)
        assert m.display_report()[5]==0x10000 and len(m.display_wire)==7
    before=len(m.display_wire)
    app.idle_ticks(m,10001)
    app.until(m,lambda:m.display_report()[5]==0x10000)
    settled(m)
    blank=bytes.fromhex('080000000018ff')
    if scenario=='unknown':
        assert len(m.display_wire)==before==7 and bytes(m.display_wire)==blank
        m.pressed=True; m.levels(service.PB)
        app.until(m,lambda:m.display_report()[5]==0x10001)
        settled(m)
        assert bytes(m.display_wire[-7:])==bytes.fromhex('080404000018ff')
        m.pressed=False; m.levels(service.PB)
        m.direction_buttons=4; m.levels(service.PA)
        app.until(m,lambda:m.display_report()[5]==2050)
        settled(m)
        assert bytes(m.display_wire[-7:])!=blank
    else:
        assert bytes(m.display_wire[-7:])==blank and len(m.display_wire)==before==7
        if scenario=='button':
            m.pressed=True; m.levels(service.PB)
            app.until(m,lambda:m.display_report()[5]==2000)
            settled(m)
            assert bytes(m.display_wire[-7:])!=blank and m.report()[6]==0
            assert m.report()[9]==1
            m.pressed=False; m.levels(service.PB)
        else:
            assert scenario=='target'
            m.rx.extend(base.encode(base.Frame(0x61,2,bytes.fromhex('0700040866'))))
            app.until(m,lambda:base.Frame(0x61,3,bytes.fromhex('000700')) in (app.frames(m) or []))
            settled(m)
            assert base.Frame(0x61,3,bytes.fromhex('000700')) in app.frames(m)
            assert m.display_report()[5]==0x10000 and len(m.display_wire)==7
            m.pressed=True; m.levels(service.PB)
            app.until(m,lambda:m.display_report()[5]==2150)
            settled(m)
            m.pressed=False; m.levels(service.PB)
            app.idle_ticks(m,9000)
            m.rx.extend(base.encode(base.Frame(0x62,2,bytes.fromhex('0700040898'))))
            app.until(m,lambda:base.Frame(0x62,3,bytes.fromhex('000700')) in (app.frames(m) or []))
            settled(m)
            assert m.display_report()[5]==2200
            app.idle_ticks(m,1001)
            app.until(m,lambda:m.display_report()[5]==0x10000)
            settled(m)
            assert bytes(m.display_wire[-7:])==blank
    assert not m.timer_runs and m.off() and not m.missed_ticks
    result=m.checks()
    result.update(screen_case=scenario,screen_timeout_ms=10000,
                  automatic_motor_motion=False,remote_updates_wake_screen=False,
                  hardware_screen_verified=False)
    return result


if __name__=='__main__':
    out=Path(sys.argv[1]); manifest=json.loads((out/'manifest.json').read_text())
    data=(out/'replacement-core.bin').read_bytes()
    assert manifest['screen_timeout_ms']==10000
    assert hashlib.sha256(data).hexdigest()==manifest['bin_sha256']
    cases={}
    for scenario in ('unknown','button','target'):
        cases[scenario]=run(data,manifest['symbols'],scenario)
        print('PASS actual-linked screen idle/wake',scenario,flush=True)
        (out/'display-idle-thumb-results.json').write_text(json.dumps(dict(
            complete=len(cases)==3,bin_sha256=manifest['bin_sha256'],
            model_sources={Path(p).name:hashlib.sha256(Path(p).read_bytes()).hexdigest()
                for p in (__file__,app.__file__,service.__file__,service.display.__file__,
                          service.probe.__file__,base.__file__)},
            cases=cases,hardware_tested=False),indent=2)+'\n')
