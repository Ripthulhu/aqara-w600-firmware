"""Real UART -> normal controller -> driver tests with an explicit saved-reference fixture.

Reference seeding is NOT calibration evidence. No implementation functions are
replaced; real firmware, peripheral model and handlers execute afterward.
"""
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from itertools import repeat
from multiprocessing import get_context
from pathlib import Path
import sys

import test_motor_service_thumb as api
app=api.app
service=app.service
base=app.base

class ControlCore(api.RepeatCore):
    def code(self,cpu,address,size,user):
        if address==self.symbols.get("w600_motor_service_begin",-1):
            self.move_adc_start=len(self.adc_launches)
        super().code(cpu,address,size,user)

    def write(self,cpu,access,address,size,value,user):
        super().write(cpu,access,address,size,value,user)
        if address==service.ADC and value==4 and self.get(service.ADC+0x1c)==0x10000:
            group=(len(self.adc_launches)-1-self.move_adc_start)//6
            raw=1 if group==0 else 1700
            at,channel,_=self.adc_launches[-1]
            self.adc_launches[-1]=(at,channel,raw)
            if self.adc_pending is not None:
                self.adc_pending=(self.adc_pending[0],channel,raw)

def read_reply(m,sequence):
    return next((f for f in (app.frames(m) or []) if f.kind==4 and f.sequence==sequence),None)

def run(data,symbols,layout,scenario):
    m=ControlCore(data,symbols,"normal")
    m.boot()
    app.until(m,lambda:m.virtual_us>=250000)
    position=symbols["valve"]+layout[0]
    quantized=scenario=="full_quantized"
    assert m.call("motor_position_reference",position,20001 if quantized else 20000,
                  24003 if quantized else 24000,1000 if quantized else 0)==1
    m.cpu.mem_write(symbols["valve"]+layout[1],b"\x01")
    app.until(m,lambda:m.cpu.mem_read(symbols["radio"]+13,2)==b"\x01\x01")
    if scenario=="unknown_source":
        app.until(m,lambda:m.report()[23]>=1 and m.report()[20]==6)
        known=m.cpu.mem_read(symbols["radio"]+4,1)[0]
        assert known&8 and not known&4  # A valid local sample must not imply sensor selection.
    #2000/1980 =>20% request. Off is kept separate from target/sensor updates.
    initial=bytes.fromhex("07000407d007030001") if scenario=="unknown_source" else \
        bytes.fromhex("07030000") if quantized else \
        bytes.fromhex("07000407d007050407bc070300")+bytes([1 if scenario in ("partial_then_off","source_change") else 0])
    m.rx.extend(base.encode(base.Frame(0x70,2,initial)))
    if scenario=="unknown_source":
        app.until(m,lambda:app.reply_count(m)==1 and app.frames(m) is not None)
        start=m.virtual_us
        app.until(m,lambda:m.virtual_us-start>=150000)
        assert m.cpu.mem_read(symbols["radio"]+layout[2],1)==b"\x02"
        assert not m.timer_runs and m.off()
        m.rx.extend(base.encode(base.Frame(0x71,2,bytes.fromhex("07040100"))))
        app.until(m,lambda:app.reply_count(m)>=2 and app.frames(m) is not None and
            any(f.kind==5 and f.payload.endswith(bytes.fromhex("07040100")) for f in app.frames(m)))
        assert m.cpu.mem_read(symbols["radio"]+layout[2],1)==b"\x02"
        assert not m.timer_runs and m.off()
        m.rx.extend(base.encode(base.Frame(0x72,2,bytes.fromhex("07040101"))))
        app.until(m,lambda:app.reply_count(m)>=3 and app.frames(m) is not None)
        assert m.cpu.mem_read(symbols["radio"]+layout[2],1)==b"\x02"
        assert not m.timer_runs and m.off()
        m.rx.extend(base.encode(base.Frame(0x73,2,bytes.fromhex("0705040834"))))
        app.until(m,lambda:app.reply_count(m)>=4 and app.frames(m) is not None)
        assert not m.timer_runs and m.off()
    elif scenario=="off_owner":
        app.until(m,lambda:app.reply_count(m)==1 and app.frames(m) is not None)
        start=m.virtual_us
        app.until(m,lambda:m.virtual_us-start>=150000)
        assert not m.timer_runs and m.off()
        for count,source in enumerate((0,1,0),2):
            before=len(app.frames(m))
            m.rx.extend(base.encode(base.Frame(0x70+count,2,bytes.fromhex("070401")+bytes([source]))))
            app.until(m,lambda:app.reply_count(m)>=count and app.frames(m) is not None and
                any(f.kind==5 and f.payload.endswith(bytes.fromhex("070401")+bytes([source]))
                    for f in app.frames(m)[before:]))
            assert not m.timer_runs and m.off()
    elif quantized:
        #2.4003s energized plus sensing gaps exceeds the default2.5s model window.
        app.until(m,lambda:len(m.timer_runs)==1 and m.motor_report()[2] in (7,8),limit=35000)
        assert m.motor_report()[2:5]==(7,0,1) and m.off()
        m.rx.extend(base.encode(base.Frame(0x71,1,bytes.fromhex("070b071c"))))
        app.until(m,lambda:read_reply(m,0x71) is not None)
        read=read_reply(m,0x71)
        assert read.payload==bytes.fromhex("070b000001071c00030000"),read
    else:
        app.until(m,lambda:len(m.timer_runs)==1 and m.motor_report()[2]==7)
        assert m.motor_report()[3]==0 and m.off()
        m.rx.extend(base.encode(base.Frame(0x71,1,bytes.fromhex("070b071c"))))
        app.until(m,lambda:read_reply(m,0x71) is not None)
        read=read_reply(m,0x71)
        assert read.payload[:5]==bytes.fromhex("070b000001")
        opening=int.from_bytes(read.payload[-2:],"big")
        assert 199<=opening<=202,opening
        closing=bytes.fromhex("07040100") if scenario=="source_change" else bytes.fromhex("07030000")
        m.rx.extend(base.encode(base.Frame(0x72,2,closing)))
        app.until(m,lambda:len(m.timer_runs)==2 and m.motor_report()[2]==7)
        assert m.motor_report()[3]==0 and m.off()
        m.rx.extend(base.encode(base.Frame(0x73,1,bytes.fromhex("0703071c"))))
        app.until(m,lambda:read_reply(m,0x73) is not None)
        read=read_reply(m,0x73)
        expected=bytes.fromhex("0703000001071c00030000") if scenario=="source_change" else \
            bytes.fromhex("0703000000071c00030000")
        assert read.payload==expected,read
        if scenario=="source_change":
            assert m.cpu.mem_read(symbols["radio"]+layout[2],1)==b"\x02"
            assert any(f.kind==5 and f.payload.endswith(bytes.fromhex("07040100")) for f in app.frames(m))
            m.rx.extend(base.encode(base.Frame(0x74,2,bytes.fromhex("0705040834"))))
            app.until(m,lambda:app.reply_count(m)>=5 and app.frames(m) is not None and
                m.cpu.mem_read(symbols["radio"]+layout[2],1)==b"\x00")
            assert len(m.timer_runs)==2 and m.off()
    assert m.cpu.mem_read(symbols["radio"]+layout[2],1)==b"\x00"
    result=m.checks()
    result.update(saved_reference_fixture=True,calibration_proven=False,
        control_path="real UART/parser/temperature selection/position/driver/IRQs",scenario=scenario)
    return result

def main():
    out=Path(sys.argv[1]); manifest=json.loads((out/"manifest.json").read_text())
    data=(out/"replacement-core.bin").read_bytes()
    assert hashlib.sha256(data).hexdigest()==manifest["bin_sha256"]
    sources={Path(p).name:hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in
        (__file__,api.__file__,app.__file__,service.__file__,service.display.__file__,
         service.probe.__file__,base.__file__)}
    results={}
    scenarios=("off_owner","partial_then_off","full_quantized","unknown_source","source_change")
    with ProcessPoolExecutor(max_workers=5,mp_context=get_context("spawn")) as pool:
        for scenario,result in zip(scenarios,pool.map(run,repeat(data),repeat(manifest["symbols"]),
                                                     repeat(manifest["test_control_layout"]),scenarios)):
            results[scenario]=result
            print("PASS actual-linked normal valve control",scenario,flush=True)
            (out/"control-thumb-results.json").write_text(json.dumps({"complete":len(results)==5,
                "bin_sha256":manifest["bin_sha256"],"hardware_tested":False,
                "model_sources":sources,"saved_reference_fixture":True,"cases":results},indent=2)+"\n")
if __name__=="__main__": main()
