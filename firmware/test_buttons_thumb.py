"""Actual-linked GPIO buttons, target, display and radio path; no device/RF claim."""
import hashlib
import json
from pathlib import Path
import sys

import application_thumb as app


def settle(m,milliseconds):
    until=m.virtual_us+milliseconds*1000
    app.until(m,lambda:m.virtual_us>=until)


def press(m,mask,milliseconds=50):
    m.pressed=bool(mask&1)
    m.direction_buttons=mask&6
    settle(m,milliseconds)


def command(m,sequence,payload,kind=2):
    count=app.reply_count(m)
    m.rx.extend(app.base.encode(app.base.Frame(sequence,kind,bytes.fromhex(payload))))
    app.until(m,lambda:app.reply_count(m)==count+1 and app.frames(m) is not None and
              m.get(app.service.PA+0x14)&(1<<11))
    return next(f for f in app.frames(m) if f.sequence==sequence and f.kind==(4 if kind==1 else 3))


def target(m):
    return m.get(m.symbols['radio'])&0xffff


def target_reports(m):
    return [f.payload for f in (app.frames(m) or []) if f.kind==5 and f.payload[:2]==b'\x07\x00']


def displayed(m,raw):
    app.until(m,lambda:m.display_report()[5]==raw and
              m.display_report()[3]==m.display_report()[4] and app.frames(m) is not None)


def run(data,symbols,scenario):
    m=app.ApplicationCore(data,symbols,'buttons')
    if scenario=='held_boot':
        m.direction_buttons=6
        m.pressed=True
    m.boot()
    if scenario=='bias':
        assert m.dial_bias_ready()
        m.direction_buttons=6
        for address,mask,value in ((app.service.PA,0x3000,0x3000),
                                   (app.service.PA+4,0x40,0),
                                   (app.service.PA+0x0c,0x3000,0),
                                   (app.service.PA+0x14,0x40,0x40)):
            saved=m.get(address);m.put(address,(saved&~mask)|value)
            m.levels(app.service.PA);m.levels(app.service.PB)
            assert not m.dial_bias_ready()
            assert m.get(app.service.PA+0x10)&4 and m.get(app.service.PB+0x10)&2
            m.put(address,saved)
        m.levels(app.service.PA);m.levels(app.service.PB)
        assert not m.get(app.service.PA+0x10)&4 and not m.get(app.service.PB+0x10)&2
        m.direction_buttons=0;m.levels(app.service.PA);m.levels(app.service.PB)
        result=m.checks()
        result.update(scenario=scenario,dial_bias_required=True,
                      physical_buttons_tested=False,rf_delivery_tested=False)
        return result
    settle(m,80)
    if scenario=='pulses':
        command(m,0x61,'07000407d0070504076c07030000')
        displayed(m,0x10000)
        for index,duration in enumerate((1,5,15)):
            press(m,4,duration)
            m.levels(app.service.PA);m.levels(app.service.PB)
            assert not m.get(app.service.PA+0x10)&(1<<2) # physical lead13: clockwise
            assert m.get(app.service.PB+0x10)&(1<<1)
            press(m,0)
            displayed(m,2050)
            app.until(m,lambda:len(target_reports(m))==2*index+1)
            assert target_reports(m)[-1]==bytes.fromhex('0700040802')
            press(m,2,duration)
            m.levels(app.service.PA);m.levels(app.service.PB)
            assert not m.get(app.service.PB+0x10)&(1<<1) # physical lead20: counterclockwise
            assert m.get(app.service.PA+0x10)&(1<<2)
            press(m,0)
            displayed(m,2000)
            app.until(m,lambda:len(target_reports(m))==2*index+2)
            assert target_reports(m)[-1]==bytes.fromhex('07000407d0')
        assert not m.report()[6] and not m.report()[7]
    elif scenario=='pairing':
        press(m,1)
        while m.pending_tick:m.step()
        app.idle_ticks(m,10001)
        app.until(m,lambda:m.report()[6:8]==(1,1))
        app.until(m,lambda:app.frames(m) is not None and
                  any(f.kind==5 and f.payload==bytes.fromhex('00510001') for f in app.frames(m)))
        press(m,1,80)
        assert m.report()[6:8]==(1,1)
        press(m,0)
        assert len([f for f in app.frames(m) if f.payload==bytes.fromhex('00510001')])==1
    else:
        # Seed target/source/mode through the real decoder, never direct state writes.
        command(m,0x61,'07000407d0070504076c07030000')
        displayed(m,0x10000)
        before=command(m,0x64,'070307040705',kind=1).payload
        assert before==bytes.fromhex('0703000000070400010107050004076c')
        assert bytes(m.display_wire[-7:])==bytes.fromhex('080000000018ff')
        if scenario=='held_boot':
            while m.pending_tick:m.step()
            app.idle_ticks(m,10001)
            settle(m,50)
            assert target(m)==2000 and not target_reports(m) and m.report()[6:8]==(0,0)
            displayed(m,0x10000)
            press(m,0)
        else:
            press(m,1,15);press(m,0)
            assert target(m)==2000 and not target_reports(m)
            displayed(m,0x10000)
        press(m,2,15)
        displayed(m,1950)
        app.until(m,lambda:target_reports(m)==[bytes.fromhex('070004079e')])
        assert bytes(m.display_wire[-7:])==bytes.fromhex('08607d001118ff')
        press(m,2,80)
        assert target(m)==1950 and len(target_reports(m))==1
        press(m,0);press(m,4)
        displayed(m,2000)
        app.until(m,lambda:len(target_reports(m))==2)
        assert target_reports(m)[-1]==bytes.fromhex('07000407d0')
        assert bytes(m.display_wire[-7:])==bytes.fromhex('08377b001018ff')
        press(m,0);press(m,6);press(m,0)
        assert target(m)==2000 and len(target_reports(m))==2
        command(m,0x62,'07000401f4')
        displayed(m,500);press(m,2);press(m,0)
        assert target(m)==500 and len(target_reports(m))==2
        command(m,0x63,'0700040bb8')
        displayed(m,3000);press(m,4);press(m,0)
        assert target(m)==3000 and len(target_reports(m))==2
        assert command(m,0x65,'070307040705',kind=1).payload==before
        assert m.report()[6:8]==(0,0)
        command(m,0x66,'07180001')
        app.until(m,lambda:bytes(m.display_wire[-7:])==bytes.fromhex('08757b101018ff'))
        press(m,2);press(m,0)
        assert target(m)==3000 and len(target_reports(m))==2
        assert command(m,0x67,'0718',kind=1).payload==bytes.fromhex('0718000001')
        command(m,0x68,'07180000')
        app.until(m,lambda:bytes(m.display_wire[-7:])==bytes.fromhex('08757b001018ff'))
    assert not m.timer_runs and not m.clamps and m.off()
    result=m.checks()
    result.update(scenario=scenario,target_reports=[p.hex() for p in target_reports(m)],
                  physical_buttons_tested=False,rf_delivery_tested=False)
    return result


def main():
    directory=Path(sys.argv[1])
    manifest=json.loads((directory/'manifest.json').read_text())
    data=(directory/'replacement-core.bin').read_bytes()
    assert hashlib.sha256(data).hexdigest()==manifest['bin_sha256']
    sources={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in
             (Path(__file__),Path(app.__file__),Path(app.service.__file__),
              Path(app.service.display.__file__),Path(app.service.probe.__file__),Path(app.base.__file__))}
    path=directory/'buttons-thumb-results.json'
    previous=json.loads(path.read_text()) if path.exists() else {}
    cases=previous.get('cases',{}) if previous.get('bin_sha256')==manifest['bin_sha256'] and previous.get('model_sources')==sources else {}
    for scenario in sys.argv[2:] or ('targets','held_boot','pairing','pulses','bias'):
        assert scenario in ('targets','held_boot','pairing','pulses','bias')
        cases[scenario]=run(data,manifest['symbols'],scenario)
        print('PASS actual-linked buttons',scenario,flush=True)
        path.write_text(json.dumps(dict(complete=set(cases)=={'targets','held_boot','pairing','pulses','bias'},
            bin_sha256=manifest['bin_sha256'],model_sources=sources,cases=cases,
            hardware_tested=False,cycle_accurate=False),indent=2)+'\n')


if __name__=='__main__':main()
