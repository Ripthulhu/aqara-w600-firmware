"""Actual linked settings path with a narrow FLASH bus model; no device access.

Executes boot restore, the foreground save adapter, SRAM worker and IRQ handlers.
Models erase/program timing and power cuts; not silicon timing or RF delivery.
"""
import hashlib
import json
from pathlib import Path
import struct
import sys
import zlib

import application_thumb as app

base=app.base
service=app.service
FP=0x40022000
PAGE=0x0800ee00
SCENARIOS=('empty','restore','restore_heat','save','traffic','low_voltage','power_cut')

def saved_page(target,generation=1,mode=0,external=1):
    record=bytearray(32)
    record[:8]=b'W6NV'+bytes((1,7,mode,external))
    struct.pack_into('<IH',record,8,generation,target)
    struct.pack_into('<II',record,24,zlib.crc32(record[:24]),0x59414b4f)
    return b'\xff'*480+record

LAYOUT=None

class SettingsCore(app.ApplicationCore):
    def __init__(self,data,symbols,pages=None,low=False,cut=None,traffic=False):
        self.busy_until=None
        self.erases=self.stores=self.busy_uart=self.busy_ticks=0
        self.key_count=0
        self.low_voltage=low
        self.cut=cut
        self.cut_reached=False
        self.send_during_save=traffic
        self.wire=[]
        self.tick_divider=0
        self.vector_changes=[]
        super().__init__(data,symbols,'settings')
        content=pages if pages is not None else b'\xff'*1024
        assert len(content)==1024
        self.cpu.mem_write(PAGE,bytes(content))
        initial=bytearray(self.flash);initial[0xee00:0xf200]=content
        self.flash=bytes(initial)
        self.put(FP+0x20,0xaa)
        for off,value in ((0x38,511),(0x3c,0),(0x40,511),(0x44,0)):
            self.put(FP+off,value)

    def state(self):
        data=bytes(self.cpu.mem_read(self.symbols['w600_settings_state'],LAYOUT[0]))
        return {'attempts':struct.unpack_from('<I',data,LAYOUT[1])[0],
                'saves':struct.unpack_from('<I',data,LAYOUT[2])[0],
                'status':int.from_bytes(data[LAYOUT[3]:LAYOUT[3]+LAYOUT[4]],'little'),
                'initialized':data[LAYOUT[5]],'dirty':data[LAYOUT[6]],'blocked':data[LAYOUT[7]]}

    def code(self,cpu,address,size,user):
        if self.cut_reached:
            self.reason='power_cut';cpu.emu_stop();return
        if self.busy_until is not None and address!=base.SENTINEL:
            assert self.symbols['__nvcode_start']<=address<self.symbols['__nvcode_end'],('FLASH execution while busy',hex(address))
        # Give the finite PVD/FLASH polling loops independent peripheral time.
        if self.get(service.PMU+4)==9:
            self.tick_divider+=1
            if self.tick_divider%100==0:self.advance_us(10)
        super().code(cpu,address,size,user)

    def advance_us(self,elapsed):
        super().advance_us(elapsed)
        if self.busy_until is not None and self.virtual_us>=self.busy_until:
            self.busy_until=None
            self.put(FP+0x10,0x1000000)
            self.put(FP+0x14,self.get(FP+0x14)&~8)
        while self.wire and self.wire[0][0]<=self.virtual_us:
            _,byte=self.wire.pop(0);self.rx.append(byte)

    def read(self,cpu,access,address,size,value,user):
        if 0x08000000<=address<0x08020000:
            assert self.busy_until is None,('FLASH data read while busy',hex(address))
        super().read(cpu,access,address,size,value,user)

    def write(self,cpu,access,address,size,value,user):
        if address==service.PMU+4:
            assert size==4 and self.get(service.RCC+0x34)&0x10000000 and value in (0,8,9)
            assert self.off() and self.selectors_low()
            # The live chip preserves level bits when a write disables PVD.
            if self.get(address)&1:
                self.pending.append((address,(value&~14)|(self.get(address)&14)))
            if value==9:self.put(service.PMU+0x14,0x800 if self.low_voltage else 0)
            return
        if address==0xe000ed08:
            assert cpu.reg_read(base.UC_ARM_REG_PRIMASK)!=0
            self.vector_changes.append(value)
        if address==FP+8:
            assert self.busy_until is None and size==4
            assert value==(0xe57a1a85 if self.key_count%2==0 else 0x7c6e8391)
            self.key_count+=1
            if self.key_count%2==0:self.put(FP+0x14,0x40000000)
            return
        if address==FP+0x10:
            assert self.busy_until is None and value==0x100001c
            self.pending.append((address,self.get(address)&~value));return
        if address==FP+0x14:
            assert self.busy_until is None and size==4
            if value&8:
                page=(value>>8)&511
                assert page in (0xee00//512,0xf000//512)
                assert value==0x4000000a|(page<<8)
                self.cpu.mem_write(0x08000000+page*512,b'\xff'*512)
                self.erases+=1;self.busy_until=self.virtual_us+2500
                self.put(FP+0x10,0x10000)
                if self.send_during_save and self.erases==1:
                    packet=base.encode(base.Frame(0x52,2,bytes.fromhex('07000408fc')))
                    self.wire=[(self.virtual_us+100+i*87,b) for i,b in enumerate(packet)]
            else:
                assert value in (0xc0000000,0x40000001,0x40000002|(0xee00//512<<8),0x40000002|(0xf000//512<<8))
            return
        if PAGE<=address<PAGE+1024:
            assert size==4 and not address&3 and self.busy_until is None
            assert self.get(FP+0x14)==0x40000001
            assert self.get(address)&value==value
            self.stores+=1;self.busy_until=self.virtual_us+50
            self.put(FP+0x10,0x10000)
            if self.cut==self.stores:self.cut_reached=True
            return
        super().write(cpu,access,address,size,value,user)

    def handler(self,vector):
        busy=self.busy_until is not None
        if busy:
            assert self.get(0xe000ed08)!=self.symbols['w600_vectors']
            if vector==43:self.busy_uart+=1
            if vector==15:self.busy_ticks+=1
        super().handler(vector)

    def checks(self):
        current=bytes(self.cpu.mem_read(0x08000000,0x20000))
        assert current[:0xee00]==self.flash[:0xee00] and current[0xf200:]==self.flash[0xf200:]
        assert self.get(0xe000ed08)==self.symbols['w600_vectors']
        assert self.get(service.RCC+0x34)==0x60000 and not self.get(service.PMU+4),(
            'PVD cleanup',hex(self.get(service.PMU+4)),self.state())
        assert self.get(FP+0x14)==0xc0000000 and self.busy_until is None
        original=self.flash;self.flash=current
        result=super().checks();self.flash=original
        result.update(state=self.state(),erases=self.erases,stores=self.stores,
                      busy_uart=self.busy_uart,busy_ticks=self.busy_ticks,
                      vector_changes=[hex(v) for v in self.vector_changes])
        return result

def until(m,predicate,limit=8000):
    for _ in range(limit):
        if predicate():return
        assert m.reason not in ('fatal','power_cut'),(m.reason,m.report(),m.state())
        m.step(1000)
    raise AssertionError(('settings timeout',m.virtual_us,m.state(),m.report()))

def change(m,target=2100):
    m.rx.extend(base.encode(base.Frame(0x51,2,b'\x07\x00\x04'+struct.pack('>H',target))))

def settle(m):
    # Do not pause foreground halfway through a real display/radio transfer.
    until(m,lambda:m.display_report()[2]==0 and m.get(service.PA+0x14)&(1<<11) and not m.pending_tick)
    app.idle_ticks(m,2001)

def run(data,symbols,name):
    assert name in SCENARIOS
    pages=saved_page(2000,4,mode=int(name=='restore_heat'))+b'\xff'*512
    m=SettingsCore(data,symbols,pages if name!='empty' else None,
                   low=name=='low_voltage',cut=3 if name=='power_cut' else None,
                   traffic=name=='traffic')
    m.boot()
    until(m,lambda:m.virtual_us>=150000)
    assert m.state()['initialized'] and not m.state()['blocked']
    if name=='empty':
        until(m,lambda:m.virtual_us>=2400000)
        assert not m.erases and not m.state()['dirty']
        return m.checks()
    # Saved settings restore without a made-up reference or old sensor sample.
    assert m.state()['status']==0
    assert not m.timer_runs
    endpoint=bytes(m.cpu.mem_read(symbols['radio'],40))
    assert endpoint[LAYOUT[8]]==5 and endpoint[LAYOUT[9]]==int(name=='restore_heat')
    assert not any(endpoint[LAYOUT[i]] for i in (10,11,12))
    assert all(endpoint[LAYOUT[i]:LAYOUT[i]+2]==b'\0\0' for i in (13,14))
    if name in ('restore','restore_heat'):return m.checks()
    change(m)
    until(m,lambda:m.state()['dirty'])
    settle(m)
    if name=='power_cut':
        for _ in range(1000):
            if m.reason=='power_cut':break
            m.step(125)
        assert m.reason=='power_cut' and m.stores==3
        stored=bytes(m.cpu.mem_read(PAGE,1024))
        restarted=SettingsCore(data,symbols,stored)
        restarted.boot();until(restarted,lambda:restarted.virtual_us>=150000)
        assert restarted.state()['status']==0 and not restarted.state()['blocked']
        assert bytes(restarted.cpu.mem_read(symbols['radio'],2))==struct.pack('<H',2000)
        result=restarted.checks();result.update(power_cut_after_words=3,old_target_restored=2000)
        return result
    until(m,lambda:m.state()['blocked'] or m.state()['saves']>=1)
    if name=='low_voltage':
        assert m.state()['blocked'] and m.state()['status']==10 and not m.erases
        return m.checks()
    assert m.erases==1 and m.stores==8 and m.busy_ticks>0
    if name=='traffic':
        until(m,lambda:not m.wire and not m.rx and m.state()['dirty'])
        assert m.busy_uart>0 and bytes(m.cpu.mem_read(symbols['radio'],2))==struct.pack('<H',2300)
        settle(m)
        until(m,lambda:m.state()['saves']==2)
        assert m.erases==2 and not m.state()['blocked']
    stored=bytes(m.cpu.mem_read(PAGE,1024))
    result=m.checks()
    restarted=SettingsCore(data,symbols,stored)
    restarted.boot();until(restarted,lambda:restarted.virtual_us>=150000)
    target=2300 if name=='traffic' else 2100
    assert bytes(restarted.cpu.mem_read(symbols['radio'],2))==struct.pack('<H',target)
    assert restarted.state()['status']==0 and not restarted.timer_runs
    result.update(restart_target=target,restart=restarted.checks())
    return result

def main():
    global LAYOUT
    directory=Path(sys.argv[1]);manifest=json.loads((directory/'manifest.json').read_text())
    LAYOUT=manifest['test_settings_layout']
    data=(directory/'replacement-core.bin').read_bytes()
    assert hashlib.sha256(data).hexdigest()==manifest['bin_sha256']
    providers=(Path(__file__),Path(app.__file__),Path(service.__file__),Path(service.display.__file__),Path(base.__file__))
    sources={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in providers}
    root=Path(__file__).resolve().parent.parent
    for p in providers:
        assert sources[str(p)]==manifest['sources'][str(p.resolve().relative_to(root))],('model source changed',str(p))
    names=sys.argv[2:] or SCENARIOS
    cases={}
    for name in names:
        cases[name]=run(data,manifest['symbols'],name);print('PASS settings actual-image',name,flush=True)
    assert sources=={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in providers},'model source changed during run'
    (directory/'settings-thumb-results.json').write_text(json.dumps({'complete':set(cases)==set(SCENARIOS),
        'bin_sha256':manifest['bin_sha256'],'model_sources':sources,'cases':cases},indent=2)+'\n')

if __name__=='__main__':main()
