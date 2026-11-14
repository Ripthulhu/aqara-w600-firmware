"""Actual linked OTA receive/staging paths on the existing narrow FLASH bus.

No device access. Invalid images must be denied; a valid synthetic image must
reach marker-last activation and restart. This is not a radio/silicon test.
"""
import binascii
from concurrent.futures import ProcessPoolExecutor, as_completed
import hashlib
import json
from pathlib import Path
import re
import struct
import subprocess
import sys
from ota_image import MAGIC,seal

import settings_thumb as settings

app=settings.app
service=app.service
base=app.base
FP=settings.FP
STAGE=0x08010800
STAGE_END=STAGE+0xce00
MARKER=0x08001e00
SCENARIOS=('receive_auth_denied','traffic','power_cut','bad_header','duplicate',
           'timeout','busy_reject','display_timeout','radio_abort','uncertain_marker','failure_transport','receive_valid')


def parse_ota_fields(text):
    """Read only named constant-offset members; reject ambiguous DIE layouts."""
    entries={};stack=[];current=None
    for line in text.splitlines():
        match=re.match(r'\s*<(\d+)><([0-9a-fA-F]+)>:\s+Abbrev Number:\s*(\d+)(?:\s+\((DW_TAG_\w+)\))?',line)
        if match:
            depth,offset,number=int(match[1]),int(match[2],16),int(match[3])
            while stack and stack[-1]['depth']>=depth:stack.pop()
            current=None
            if not number:continue
            assert match[4],line
            current={'depth':depth,'tag':match[4],'attrs':{},'children':[]}
            assert offset not in entries,('duplicate DIE',hex(offset))
            entries[offset]=current
            if stack:
                assert stack[-1]['depth']==depth-1,line
                stack[-1]['children'].append(current)
            stack.append(current)
            continue
        match=re.match(r'\s*<[0-9a-fA-F]+>\s+DW_AT_(\w+)\s*:\s*(.*?)\s*$',line)
        if match and current:current['attrs'][match[1]]=match[2]
    def name(entry):
        value=entry['attrs'].get('name','').rsplit(':',1)[-1].strip()
        return re.sub(r'^\([^)]*\)\s*','',value)
    def constant(value):
        match=re.fullmatch(r'(?:\([^)]*\)\s*)?(0x[0-9a-fA-F]+|[0-9]+)',value)
        assert match,('nonconstant DWARF field',value)
        return int(match[1],0) if match[1].startswith('0x') else int(match[1])
    def size(entry):
        seen=set()
        while 'byte_size' not in entry['attrs']:
            match=re.fullmatch(r'(?:\([^)]*\)\s*)?<0x([0-9a-fA-F]+)>',entry['attrs'].get('type',''))
            assert match,('missing DWARF type size',entry)
            offset=int(match[1],16)
            assert offset in entries and offset not in seen,('invalid DWARF type chain',offset)
            seen.add(offset);entry=entries[offset]
        return constant(entry['attrs']['byte_size'])
    matches=[]
    wanted={'sent_at':4,'pending':1,'waiting':1,'retries':1}
    for entry in entries.values():
        if entry['tag']!='DW_TAG_structure_type' or name(entry)!='w600_ota':continue
        fields={}
        for member in entry['children']:
            key=name(member)
            if member['tag']!='DW_TAG_member' or key not in wanted:continue
            offset=constant(member['attrs']['data_member_location'])
            assert 0<=offset<1024 and size(member)==wanted[key],(key,offset)
            assert key not in fields,key
            fields[key]=offset
        if fields:
            assert set(fields)==set(wanted),fields
            matches.append(fields)
    assert matches and all(fields==matches[0] for fields in matches),matches
    return matches[0]


def ota_fields(elf_path):
    text=subprocess.check_output(['arm-none-eabi-readelf','--debug-dump=info',str(elf_path)],text=True)
    return parse_ota_fields(text)


def parser_check():
    fixture=''' <0><0>: Abbrev Number: 1 (DW_TAG_compile_unit)
 <1><10>: Abbrev Number: 2 (DW_TAG_base_type)
    <11> DW_AT_byte_size : 4
 <1><20>: Abbrev Number: 2 (DW_TAG_base_type)
    <21> DW_AT_byte_size : 1
 <1><30>: Abbrev Number: 3 (DW_TAG_typedef)
    <31> DW_AT_type : <0x10>
 <1><40>: Abbrev Number: 4 (DW_TAG_structure_type)
    <41> DW_AT_name : (indirect string, offset: 0x123): w600_ota
 <2><50>: Abbrev Number: 5 (DW_TAG_member)
    <51> DW_AT_name : sent_at
    <52> DW_AT_type : <0x30>
    <53> DW_AT_data_member_location : 800
 <2><60>: Abbrev Number: 5 (DW_TAG_member)
    <61> DW_AT_name : pending
    <62> DW_AT_type : <0x20>
    <63> DW_AT_data_member_location : 804
 <2><70>: Abbrev Number: 5 (DW_TAG_member)
    <71> DW_AT_name : waiting
    <72> DW_AT_type : <0x20>
    <73> DW_AT_data_member_location : 805
 <2><80>: Abbrev Number: 5 (DW_TAG_member)
    <81> DW_AT_name : retries
    <82> DW_AT_type : <0x20>
    <83> DW_AT_data_member_location : 806
 <2><90>: Abbrev Number: 0
'''
    assert parse_ota_fields(fixture)=={'sent_at':800,'pending':804,'waiting':805,'retries':806}
    for bad in (fixture.replace('byte_size : 4','byte_size : 8'),
                fixture.replace('location : 800','location : 2 byte block: 23 01'),
                fixture.replace('waiting','unknown')):
        try:parse_ota_fields(bad)
        except AssertionError:pass
        else:raise AssertionError('bad DWARF accepted')


def age_reply_deadline(m,layout):
    at=m.symbols['ota']
    until(m,lambda:not m.cpu.mem_read(at+layout['pending'],1)[0] and
          bool(m.cpu.mem_read(at+layout['waiting'],1)[0]))
    now=m.get(m.symbols['app_ticks'])
    m.put(at+layout['sent_at'],(now-6000)&0xffffffff)


class OtaCore(settings.SettingsCore):
    def __init__(self,data,symbols,scenario,flash=None):
        self.ota_scenario=scenario
        self.worker_msp=[]
        self.stage_erases=[]
        self.display_at_takeover=None
        self.reset_requested_at=None
        super().__init__(data,symbols,cut=3 if scenario=='power_cut' else None)
        seed=bytearray(self.flash if flash is None else flash)
        if flash is None:
            seed[STAGE-0x08000000:STAGE_END-0x08000000]=b'\xff'*(STAGE_END-STAGE)
            seed[MARKER-0x08000000:MARKER-0x08000000+512]=b'\xff'*512
        self.flash=bytes(seed)
        self.cpu.mem_write(0x08000000,self.flash)

    def code(self,cpu,address,size,user):
        if address==self.symbols['w600_ota_flash_page_write']:
            sp=cpu.reg_read(base.UC_ARM_REG_MSP)
            self.worker_msp.append(sp)
            assert 0x20006e00<=sp<=0x20007000 and not sp&7,('OTA worker caller stack',hex(sp))
        if self.get(self.symbols['ota_owned'])&255 and self.display_at_takeover is None:
            self.display_at_takeover=len(self.display_wire)
        super().code(cpu,address,size,user)

    def write(self,cpu,access,address,size,value,user):
        if address==0xe000ed0c:
            assert size==4 and value>>16==0x05fa and value&4,('unexpected AIRCR write',hex(value))
            assert not self.in_handler and cpu.reg_read(base.UC_ARM_REG_PRIMASK)==1
            assert self.off() and self.selectors_low() and self.busy_until is None
            assert bytes(cpu.mem_read(MARKER,512))==b'\xff'*512,'reset with uncertain activation marker'
            self.reset_requested_at=self.virtual_us
            self.reason='reset';cpu.emu_stop();return
        if address==FP+0x14 and value&8:
            page=(value>>8)&511
            target=0x08000000+page*512
            if STAGE<=target<STAGE_END:
                assert self.busy_until is None and size==4
                assert value==0x4000000a|(page<<8)
                assert self.off() and self.selectors_low()
                self.cpu.mem_write(target,b'\xff'*512)
                self.erases+=1;self.stage_erases.append(target)
                self.busy_until=self.virtual_us+2500
                self.put(FP+0x10,0x10000)
                if self.ota_scenario=='traffic' and self.erases==1:
                    packet=base.encode(base.Frame(0x67,1,bytes.fromhex('0700')))
                    self.wire=[(self.virtual_us+100+i*87,b) for i,b in enumerate(packet)]
                return
        if address==FP+0x14 and value&0xff==2:
            page=(value>>8)&511
            if STAGE<=0x08000000+page*512<STAGE_END:
                assert self.busy_until is None and size==4 and value==0x40000002|(page<<8)
                return
        if STAGE<=address<STAGE_END:
            assert size==4 and not address&3 and self.busy_until is None
            assert self.get(FP+0x14)==0x40000001
            assert self.get(address)&value==value
            assert self.off() and self.selectors_low()
            self.stores+=1;self.busy_until=self.virtual_us+50
            self.put(FP+0x10,0x10000)
            if self.cut==self.stores:self.cut_reached=True
            return
        super().write(cpu,access,address,size,value,user)

    def unchanged_protected(self):
        current=bytes(self.cpu.mem_read(0x08000000,0x20000))
        first=STAGE-0x08000000;last=STAGE_END-0x08000000
        assert current[:first]==self.flash[:first], 'bootloader, active or settings changed'
        assert current[last:]==self.flash[last:], 'descriptors or other protected flash changed'
        assert current[0x1e00:0x2000]==b'\xff'*512, 'OTA marker armed'
        return current

    def ota_checks(self):
        current=self.unchanged_protected()
        assert self.off() and self.selectors_low() and not self.timer_runs
        assert self.get(0xe000ed08)==self.symbols['w600_vectors']
        assert self.get(service.RCC+0x34)==0x60000 and not self.get(service.PMU+4)
        assert self.get(FP+0x14)==0xc0000000 and self.busy_until is None
        old=self.flash;self.flash=current
        result=app.ApplicationCore.checks(self);self.flash=old
        result.update(stage_pages=[hex(x) for x in self.stage_erases],erases=self.erases,
                      stores=self.stores,busy_uart=self.busy_uart,busy_ticks=self.busy_ticks,
                      worker_entry_msp=[hex(x) for x in self.worker_msp],
                      ota_owned=bool(self.get(self.symbols['ota_owned'])&255),
                      activation_marker_armed=False,protected_flash_unchanged=True,
                      production_authorization_enabled=True)
        return result


class SuccessfulOtaCore(OtaCore):
    def __init__(self,*args,**kwargs):
        self.commit_pages=[]
        super().__init__(*args,**kwargs)

    def write(self,cpu,access,address,size,value,user):
        pages=(MARKER,0x0801fc00,0x0801fe00)
        if address==0xe000ed0c:
            assert size==4 and value>>16==0x05fa and value&4
            assert not self.in_handler and cpu.reg_read(base.UC_ARM_REG_PRIMASK)==1
            assert self.off() and self.selectors_low() and self.busy_until is None
            assert self.commit_pages==[0x0801fc00,0x0801fe00,MARKER]
            assert bytes(cpu.mem_read(MARKER,512))==struct.pack('<I',0x67890123)+b'\xff'*508
            self.reset_requested_at=self.virtual_us;self.reason='reset';cpu.emu_stop();return
        if address==FP+0x14:
            target=0x08000000+((value>>8)&511)*512
            if target in pages and value&0xff in (2,10):
                assert self.busy_until is None and size==4 and self.off() and self.selectors_low()
                assert value==(0x40000000|((target-0x08000000)//512<<8)|(value&0xff))
                if value&8:
                    self.commit_pages.append(target)
                    self.cpu.mem_write(target,b'\xff'*512)
                    self.erases+=1;self.busy_until=self.virtual_us+2500;self.put(FP+0x10,0x10000)
                return
        if any(p<=address<p+512 for p in pages):
            assert size==4 and not address&3 and self.busy_until is None
            assert self.get(FP+0x14)==0x40000001 and self.get(address)&value==value
            assert self.off() and self.selectors_low()
            self.stores+=1;self.busy_until=self.virtual_us+50;self.put(FP+0x10,0x10000);return
        super().write(cpu,access,address,size,value,user)


def until(m,predicate,limit=24000):
    for _ in range(limit):
        if predicate():return
        assert m.reason not in ('fatal','power_cut','reset'),(m.reason,m.report(),m.virtual_us)
        m.step(500)
    raise AssertionError(('OTA model timeout',m.virtual_us,m.report(),m.tx[-40:].hex()))


def frames(m):
    return app.frames(m)


def packet(m,frame):
    m.rx.extend(base.encode(frame))


def wait_frame(m,predicate,after=0):
    found=[]
    def ready():
        decoded=frames(m)
        if decoded is None:return False
        for index,frame in enumerate(decoded[after:],after):
            if predicate(frame):found.append((index,frame));return True
        return False
    until(m,ready)
    return found[0]


def start(m):
    packet(m,base.Frame(0x80,2,bytes.fromhex('40000001')))
    index,ack=wait_frame(m,lambda f:f.sequence==0x80 and f.kind==3)
    assert ack.payload==bytes.fromhex('004000'),ack
    index,request=wait_frame(m,lambda f:f.kind==6 and f.payload[:2]==bytes.fromhex('4001'),index+1)
    assert request.payload==bytes.fromhex('40010c0207000000000180'),request
    assert m.get(m.symbols['ota_owned'])&255
    return index,request


def image_and_header():
    image=bytearray((i*17+3)&255 for i in range(768))
    struct.pack_into('<II',image,0,0x20007000,0x08002081)
    header=bytearray(128)
    struct.pack_into('<I',header,0,0x6c756d69)
    struct.pack_into('<I',header,8,128)
    struct.pack_into('<III',header,32,len(image),binascii.crc_hqx(image,0),29)
    struct.pack_into('<I',header,124,binascii.crc_hqx(header[:124],0))
    return bytes(image),bytes(header)


def respond(m,request,data):
    assert len(data)<=224
    packet(m,base.Frame(request.sequence,7,bytes.fromhex('4001000b')+bytes((len(data),))+data))


def receive(m,index,request,duplicate=False):
    image,header=image_and_header()
    respond(m,request,header)
    index,request=wait_frame(m,lambda f:f.kind==6 and f.payload[:2]==bytes.fromhex('4001'),index+1)
    while True:
        offset=int.from_bytes(request.payload[5:9],'big')-128
        size=request.payload[10]
        assert 0<=offset<len(image) and size==min(224,len(image)-offset)
        old=request
        respond(m,request,image[offset:offset+size])
        index,request=wait_frame(m,lambda f:f.kind==6 and f.payload[0]==0x40,index+1)
        if duplicate:
            respond(m,old,image[offset:offset+size])
            elapsed=m.virtual_us
            until(m,lambda:m.virtual_us-elapsed>=10000)
            assert m.reason!='fatal'
        if request.payload[:2]==bytes.fromhex('4004'):
            assert request.payload==bytes.fromhex('40040122'),request
            break
        assert request.payload[:2]==bytes.fromhex('4001'),request
    assert bytes(m.cpu.mem_read(STAGE,len(image)))==image
    assert bytes(m.cpu.mem_read(STAGE+len(image),256))==b'\xff'*256
    assert m.erases==2 and m.worker_msp and m.busy_ticks>0
    return index


def recover_failure(m,data,symbols,transport_failure=False):
    result=m.ota_checks()
    failed_at=m.virtual_us
    if transport_failure:m.uart_error=1
    until(m,lambda:m.reason=='reset')
    assert m.reset_requested_at is not None and 0<=m.reset_requested_at-failed_at<=2100000
    current=m.unchanged_protected()
    restarted=OtaCore(data,symbols,'receive_auth_denied',flash=current)
    restarted.boot();until(restarted,lambda:restarted.virtual_us>=200000)
    assert not restarted.get(symbols['ota_owned'])&255 and restarted.off() and not restarted.timer_runs
    endpoint=symbols['radio']
    for field in (10,11):
        assert restarted.cpu.mem_read(endpoint+settings.LAYOUT[field],1)==b'\0','reference survived reset'
    result.update(failed_update_reset_requested=True,reset_after_failure_frame_us=m.reset_requested_at-failed_at,
                  old_application_restarted=True,post_reset_ota_owned=False,post_reset_reference_valid=False,
                  recovery_after_transport_fault=transport_failure,
                  post_reset=restarted.ota_checks())
    return result


def run(data,symbols,name,ota_layout):
    assert name in SCENARIOS
    m=(SuccessfulOtaCore if name=='receive_valid' else OtaCore)(data,symbols,name)
    m.boot()
    until(m,lambda:m.virtual_us>=1800000 and m.display_report()[2]==0)
    assert m.off() and not m.timer_runs and not m.erases
    assert m.get(symbols['authorize'])==symbols['ota_authorize']|1,'OTA backend policy not connected'
    if name=='busy_reject':
        m.put(symbols['mounting_report'],0xa7)
        until(m,lambda:len(m.timer_runs)==1 and not m.off())
        packet(m,base.Frame(0x80,2,bytes.fromhex('40000001')))
        index,ack=wait_frame(m,lambda f:f.kind==3 and f.sequence==0x80)
        assert ack.payload==bytes.fromhex('044000'),ack
        assert not m.get(symbols['ota_owned'])&255 and not m.erases
        packet(m,base.Frame(0x81,2,bytes.fromhex('07030000')))
        until(m,lambda:m.off() and m.motor_report()[4]==1)
        # This scenario intentionally ran a modeled mounting movement.
        old=m.timer_runs;m.timer_runs=[]
        result=m.ota_checks();m.timer_runs=old
        result.update(busy_initial_rejected=True,modeled_motor_runs=len(old))
        return result
    if name=='display_timeout':
        packet(m,base.Frame(0x71,2,bytes.fromhex('07000407d0')))
        wait_frame(m,lambda f:f.sequence==0x71 and f.kind==3)
        before_wake=len(m.display_wire)
        m.direction_buttons=4;m.levels(service.PB)
        until(m,lambda:m.display_report()[5] in (2000,2050) and m.display_report()[2]==0 and
              bool(m.cpu.mem_read(symbols['display_awake'],1)[0]) and len(m.display_wire)>before_wake)
        wake_target=m.display_report()[5]
        wake_bytes=len(m.display_wire)-before_wake
        m.direction_buttons=0;m.levels(service.PB)
        until(m,lambda:m.virtual_us>=1900000 and m.display_report()[2]==0)
    index,request=start(m)
    if name=='receive_valid':
        revision=m.get(symbols['w600_ota_identity']+20)+1
        image,header=image_and_header();image=bytearray(image)
        image[-64:]=struct.pack('<16s4I32s',MAGIC,1,revision,0,1,bytes(32))
        image=seal(image,revision);header=bytearray(header)
        struct.pack_into('<III',header,32,len(image),binascii.crc_hqx(image,0),revision)
        struct.pack_into('<I',header,124,binascii.crc_hqx(header[:124],0))
        respond(m,request,header)
        while True:
            index,request=wait_frame(m,lambda f:f.kind==6 and f.payload[0]==0x40,index+1)
            if request.payload[:2]==bytes.fromhex('4002'):break
            assert request.payload[:2]==bytes.fromhex('4001'),request
            offset=int.from_bytes(request.payload[5:9],'big')-128;n=request.payload[10]
            respond(m,request,image[offset:offset+n])
        assert m.commit_pages==[] and not m.timer_runs
        packet(m,base.Frame(request.sequence,7,bytes.fromhex('40020007')+struct.pack('>I',100)))
        until(m,lambda:m.reset_requested_at is not None)
        after=bytes(m.cpu.mem_read(0x08000000,0x20000))
        assert after[:0x1e00]==m.flash[:0x1e00] and after[0x2000:0x10800]==m.flash[0x2000:0x10800]
        assert after[0x1d600:0x1fc00]==m.flash[0x1d600:0x1fc00]
        assert after[STAGE-0x08000000:STAGE-0x08000000+len(image)]==image
        assert after[0x1fc00:0x1fe00]==after[0x1fe00:]
        descriptor=struct.unpack_from('<5I',after,0x1fc00)
        assert descriptor[:3]==(0,STAGE,len(image)) and descriptor[3]&0xffff==binascii.crc_hqx(image,0)
        assert descriptor[4]>>16==binascii.crc_hqx(after[0x1fc00:0x1fc12],0)
        return dict(valid_update_accepted=True,activation_order=[hex(p) for p in m.commit_pages],
                    compiled_application_requested_restart=True,active_image_unchanged_until_bootloader=True,
                    target_revision=revision,motor_runs=m.timer_runs,hardware_tested=False)
    if name in ('bad_header','uncertain_marker','failure_transport'):
        if name=='uncertain_marker':
            m.cpu.mem_write(MARKER+508,b'\x00')
            m.flash=bytes(m.cpu.mem_read(0x08000000,0x20000))
        _,header=image_and_header();header=bytearray(header);header[124]^=1
        respond(m,request,bytes(header))
        _,failure=wait_frame(m,lambda f:f.kind==6 and f.payload[:2]==bytes.fromhex('4004'),index+1)
        assert failure.payload==bytes.fromhex('40040123') and not m.erases
        if name=='uncertain_marker':
            then=m.virtual_us
            until(m,lambda:m.virtual_us-then>=2500000)
            assert m.reset_requested_at is None and m.off() and not m.timer_runs
            assert bytes(m.cpu.mem_read(0x08000000,0x20000))==m.flash
            return {'uncertain_marker_prevents_reset':True,'observed_after_failure_us':m.virtual_us-then,
                    'timer_runs':m.timer_runs,'protected_flash_unchanged':True}
        return recover_failure(m,data,symbols,transport_failure=name=='failure_transport')
    if name=='radio_abort':
        packet(m,base.Frame(request.sequence,7,bytes.fromhex('400400')))
        _,failure=wait_frame(m,lambda f:f.kind==6 and f.payload[:2]==bytes.fromhex('4004'),index+1)
        assert failure.payload==bytes.fromhex('40040122') and not m.erases
        return recover_failure(m,data,symbols)
    if name=='timeout':
        original=request
        for _ in range(3):
            age_reply_deadline(m,ota_layout)
            index,retry=wait_frame(m,lambda f:f.kind==6 and f.payload[:2]==bytes.fromhex('4001'),index+1)
            assert retry==original
        age_reply_deadline(m,ota_layout)
        _,failure=wait_frame(m,lambda f:f.kind==6 and f.payload[:2]==bytes.fromhex('4004'),index+1)
        assert failure.payload==bytes.fromhex('40040122') and not m.erases
        result=recover_failure(m,data,symbols)
        result.update(deadline_fault_injection={'field':'w600_ota.sent_at','age_ms':6000,
                      'retry_frames_checked':3,'final_timeout_checked':True},
                      full_26_second_elapsed_run=False)
        return result
    if name=='display_timeout':
        assert m.cpu.mem_read(symbols['display_awake'],1)[0]
        m.put(symbols['display_activity_at'],(m.get(symbols['app_ticks'])-10000)&0xffffffff)
        length=len(m.display_wire);then=m.virtual_us
        until(m,lambda:m.virtual_us-then>=20000)
        assert len(m.display_wire)==length,'display woke or blanked during OTA ownership'
        assert m.get(symbols['ota_owned'])&255
        index=len(frames(m))-1
    if name=='power_cut':
        image,header=image_and_header();respond(m,request,header)
        index,request=wait_frame(m,lambda f:f.kind==6 and f.payload[:2]==bytes.fromhex('4001'),index+1)
        for offset in (0,224,448):
            respond(m,request,image[offset:offset+min(224,768-offset)])
            if offset!=448:
                index,request=wait_frame(m,lambda f:f.kind==6 and f.payload[:2]==bytes.fromhex('4001'),index+1)
        for _ in range(10000):
            if m.reason=='power_cut':break
            assert m.reason!='fatal',m.report()
            m.step(125)
        assert m.reason=='power_cut' and m.stores==3 and m.erases==1
        current=m.unchanged_protected()
        restarted=OtaCore(data,symbols,'receive_auth_denied',flash=current)
        restarted.boot();until(restarted,lambda:restarted.virtual_us>=200000)
        assert not restarted.get(symbols['ota_owned'])&255 and restarted.off()
        result=restarted.ota_checks()
        result.update(power_cut_after_words=3,old_application_restarted=True,
                      interrupted_image_activated=False,worker_entry_msp=[hex(x) for x in m.worker_msp])
        return result
    receive(m,index,request,duplicate=name=='duplicate')
    if name=='traffic':
        wait_frame(m,lambda f:f.kind==4 and f.sequence==0x67)
        assert m.busy_uart>0 and not m.wire and not m.rx
    result=recover_failure(m,data,symbols);result['authorization_denied_after_complete_image']=True
    if name=='display_timeout':
        result.update(deadline_fault_injection={'field':'display_activity_at','age_ms':10000,
                      'subsequent_foreground_us':20000},full_10_second_elapsed_run=False,
                      physical_wake_target=wake_target,physical_wake_display_bytes=wake_bytes)
    return result


def run_case(data,symbols,name,layout,ota_layout):
    settings.LAYOUT=layout
    return run(data,symbols,name,ota_layout)


def main():
    parser_check()
    directory=Path(sys.argv[1]);manifest=json.loads((directory/'manifest.json').read_text())
    settings.LAYOUT=manifest['test_settings_layout']
    data=(directory/'replacement-core.bin').read_bytes()
    assert hashlib.sha256(data).hexdigest()==manifest['bin_sha256']
    elf_path=directory/'replacement-core.elf'
    elf_hash=hashlib.sha256(elf_path.read_bytes()).hexdigest()
    assert elf_hash==manifest['elf_sha256']
    ota_layout=ota_fields(elf_path)
    providers=(Path(__file__),Path(settings.__file__),Path(app.__file__),Path(service.__file__),
               Path(service.display.__file__),Path(base.__file__))
    sources={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in providers}
    root=Path(settings.__file__).resolve().parent.parent
    # Supplementary model is recorded by its own hash; inherited providers must
    # still match the frozen application's original build manifest.
    for p in providers[1:]:
        assert sources[str(p)]==manifest['sources'][str(p.resolve().relative_to(root))],('model source changed',str(p))
    names=sys.argv[2:] or SCENARIOS
    cases={}
    with ProcessPoolExecutor(max_workers=min(3,len(names))) as pool:
        futures={pool.submit(run_case,data,manifest['symbols'],name,settings.LAYOUT,ota_layout):name for name in names}
        for future in as_completed(futures):
            name=futures[future];cases[name]=future.result()
            print('PASS OTA actual-image',name,flush=True)
    assert sources=={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in providers}
    (directory/'ota-thumb-results.json').write_text(json.dumps({'complete':set(cases)==set(SCENARIOS),
        'bin_sha256':manifest['bin_sha256'],'elf_sha256':elf_hash,'dwarf_ota_fields':ota_layout,
        'model_sources':sources,'cases':cases},indent=2)+'\n')


if __name__=='__main__':main()
