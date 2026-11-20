"""Execute linked STOP/clock-resume instructions on a supplied register model.

No device access. Wake sources, oscillator lock and timer counts are supplied;
this cannot prove current draw, silicon STOP entry or radio wake lead time.
"""
import hashlib
import json
from pathlib import Path
import struct
import sys
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_MCLASS, UC_MODE_THUMB
from unicorn import UC_HOOK_CODE, UC_HOOK_MEM_READ, UC_HOOK_MEM_WRITE
from unicorn.arm_const import (UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC,
    UC_ARM_REG_XPSR, UC_ARM_REG_PRIMASK, UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2)

RCC,LP,EXTI,ST,SCB=0x40021000,0x40009000,0x40021800,0xe000e010,0xe000ed00
NVIC,PA,PB,PMU,UART=0xe000e100,0x50000000,0x50000400,0x40007000,0x40004800
STATE,TICKS,REPORT,END=0x20005000,0x20005100,0x20005200,0x20008000
LINES=(1<<1)|(1<<2)|(1<<8)|(1<<10)
IRQS=(1<<5)|(1<<6)|(1<<7)|(1<<30)
CASES=('timer_match','timer_masked','timer_wrap','radio','primary','decrease','increase',
       'pending_wfi','pll_failure','switch_failure','already_low','rx_pending','radio_during_setup')


def run(data,symbols,case):
    cpu=Uc(UC_ARCH_ARM,UC_MODE_MCLASS|UC_MODE_THUMB)
    for address,size in ((0x20000000,0x10000),(0x08000000,0x20000),
                         (0x40000000,0x30000),(0x50000000,0x1000),(0xe000e000,0x2000)):
        cpu.mem_map(address,size)
    cpu.mem_write(0x08002000,data)
    def put(a,v):cpu.mem_write(a,struct.pack('<I',v&0xffffffff))
    def get(a):return struct.unpack('<I',cpu.mem_read(a,4))[0]
    initial_imr=0xefc10000&(~(1<<23) if case=='timer_masked' else 0xffffffff)
    for a,v in {RCC:0x03000500,RCC+8:0x12,RCC+12:0x30260,RCC+0x44:3,
        RCC+0x34:0x40060000,RCC+0x3c:0x100000,RCC+0x2c:15,
        0x40022000:2,0x40022024:0x7121,NVIC:1<<27,
        PA:1<<22,PA+0x10:(1<<2)|(1<<10)|(1<<11),PB+0x10:(1<<1)|(1<<8),
        UART+0x1c:0x40,LP+0x18:15999,LP+0x1c:40,LP+0x10:1,
        ST:7,ST+4:47999,ST+8:35654,
        EXTI+0x50:0x12345008,EXTI+0x54:0x87654030,EXTI:0x10002,
        EXTI+4:0x10004,EXTI+0x70:initial_imr,EXTI+0x74:0x20000,TICKS:100}.items():put(a,v)
    state=[7,0,3,0,0,0,0,0,0,32000,2,0,0,1,TICKS]
    cpu.mem_write(STATE,struct.pack('<15I',*state))
    if case=='already_low':
        put(PA+0x10,get(PA+0x10)&~(1<<10))
        put(REPORT+12,1<<2) # Stale dial edge must not be replayed on a skipped sleep.
    if case=='rx_pending':put(UART+0x1c,get(UART+0x1c)|0x20)
    pending=[];writes=[];waits=0;elapsed=0;instructions=0;lowest=0x20007000

    def read(cpu,access,a,size,value,user):
        if PMU<=a<PMU+12:assert get(RCC+0x34)&(1<<28)

    def write(cpu,access,a,size,value,user):
        assert not (0x08000000<=a<0x08020000),'FLASH write'
        if 0x20000000<=a<0x20010000:
            assert STATE<=a<STATE+60 or a==TICKS or REPORT<=a<REPORT+36 or 0x20006c20<=a<0x20007000
            return
        assert size==4
        writes.append((a,value))
        if a==NVIC:pending.append((a,get(a)|value))
        elif a==NVIC+0x80:pending.append((NVIC,get(NVIC)&~value))
        elif a==NVIC+0x180:pending.append((NVIC+0x100,get(NVIC+0x100)&~value))
        elif a==EXTI+0x70 and case=='radio_during_setup' and value&LINES:
            pending.extend(((EXTI+8,1<<10),(NVIC+0x100,1<<7)))
        elif a==EXTI+8:
            assert not value&~0x3ffff,'reserved EXTI pending bit write'
            pending.append((a,get(a)&~value))
        elif a==LP+4:pending.append((LP,get(LP)&~value))
        elif a==RCC:
            if case!='pll_failure':pending.append((a,value|0x02000000))
        elif a==RCC+8:
            if case!='switch_failure':pending.append((a,value|0x10))
        elif a==SCB+4:
            assert value==1<<25;pending.append((a,get(a)&~(1<<26)))
        elif a==ST and value==7:pending.append((ST+8,47999))
        else:
            assert a in (RCC+0x34,RCC+12,ST,ST+8,SCB+0x10,LP+8,
                         EXTI,EXTI+4,EXTI+0x50,EXTI+0x54,EXTI+0x70),hex(a)

    def code(cpu,a,size,user):
        nonlocal waits,elapsed,instructions,lowest
        instructions+=1
        while pending:
            address,value=pending.pop(0);put(address,value)
        lowest=min(lowest,cpu.reg_read(UC_ARM_REG_SP))
        assert lowest>=0x20006c20
        if a==END:cpu.emu_stop();return
        if bytes(cpu.mem_read(a,2))!=b'\x30\xbf':return
        assert get(SCB+0x10)==4 and get(ST)==0 and cpu.reg_read(UC_ARM_REG_PRIMASK)==1
        assert get(NVIC)==((1<<27)|IRQS) and get(LP+8)==2
        waits+=1
        if case!='pending_wfi':
            put(RCC,0x500);put(RCC+8,0);put(RCC+12,0x20260)
        if case in ('timer_match','timer_masked'):
            put(LP+0x1c,15999);put(LP,2);elapsed=15959;put(NVIC+0x100,1<<30)
        elif case=='timer_wrap':
            put(LP+0x1c,16);put(LP,2);elapsed=15976;put(NVIC+0x100,1<<30)
        else:
            put(LP+0x1c,360);elapsed=320
            if case in ('radio','primary','decrease','increase'):
                bank,pin,irq={'radio':(PA,10,7),'primary':(PB,8,7),'decrease':(PB,1,5),'increase':(PA,2,6)}[case]
                put(bank+0x10,get(bank+0x10)&~(1<<pin));put(EXTI+8,1<<pin);put(NVIC+0x100,1<<irq)
        cpu.reg_write(UC_ARM_REG_PC,(a+2)|1)

    cpu.hook_add(UC_HOOK_CODE,code);cpu.hook_add(UC_HOOK_MEM_READ,read);cpu.hook_add(UC_HOOK_MEM_WRITE,write)
    cpu.reg_write(UC_ARM_REG_SP,0x20007000);cpu.reg_write(UC_ARM_REG_XPSR,0x01000000)
    cpu.reg_write(UC_ARM_REG_PRIMASK,1);cpu.reg_write(UC_ARM_REG_LR,END|1)
    for r,v in ((UC_ARM_REG_R0,STATE),(UC_ARM_REG_R1,TICKS),(UC_ARM_REG_R2,REPORT)):cpu.reg_write(r,v)
    try:
        cpu.emu_start(symbols['w600_stop_mode']|1,END,count=1000000)
    except Exception as error:
        pc=cpu.reg_read(UC_ARM_REG_PC)
        raise AssertionError((case,hex(pc),bytes(cpu.mem_read(pc,8)).hex(),instructions)) from error
    assert cpu.reg_read(UC_ARM_REG_PC)==END,(case,'did not return',instructions)
    result=cpu.reg_read(UC_ARM_REG_R0);report=struct.unpack('<9I',cpu.mem_read(REPORT,36))
    assert get(NVIC)==1<<27 and not get(NVIC+0x100)&IRQS and not get(SCB+0x10)
    assert get(EXTI+0x50)==0x12345008 and get(EXTI+0x54)==0x87654030
    assert get(EXTI)==0x10002 and get(EXTI+4)==0x10004 and get(EXTI+0x70)==initial_imr
    assert get(EXTI+0x74)==0x20000 and not get(LP+8) and get(RCC+0x34)==0x40060000
    assert cpu.reg_read(UC_ARM_REG_PRIMASK)==1 and get(LP+0x10)==1
    if case in ('already_low','rx_pending'):
        assert result==0 and not waits and not writes and not report[3] and get(TICKS)==100 and get(ST)==7
    elif case=='radio_during_setup':
        assert result==0 and not waits and report[:5]==(0,0,0,1<<10,0)
        assert report[5:]==(1,0,0,100) and get(TICKS)==100 and get(ST)==7
    elif case in ('pll_failure','switch_failure'):
        assert result==2 and waits==1 and report[2]==4 and get(TICKS)==100 and not get(ST)
    else:
        numerator=elapsed*48000000+12345*32000;divisor=32000*48000
        expected=100+(numerator+divisor-1)//divisor
        assert result==1 and waits==1 and report[:3]==(1,1,0) and report[4]==elapsed
        assert get(TICKS)==expected and get(ST)==7 and get(ST+8)==47999
        assert get(RCC+8)==0x12 and get(RCC+12)==0x30260
        assert report[5:8]==(int(case=='radio'),int(case in ('primary','decrease','increase')),
                              int(case in ('timer_match','timer_masked','timer_wrap')))
        assert report[8]==(expected if case=='radio' else 0)
    if result!=1 and case!='radio_during_setup':assert report[5:]==(0,0,0,0)
    return dict(result=result,report=report,ticks=get(TICKS),instructions=instructions,
                lowest_sp=lowest,hardware_tested=False)


if __name__=='__main__':
    out=Path(sys.argv[1]);manifest=json.loads((out/'manifest.json').read_text())
    data=(out/'replacement-core.bin').read_bytes()
    assert hashlib.sha256(data).hexdigest()==manifest['bin_sha256']
    cases={}
    for case in tuple(sys.argv[2:]) or CASES:
        assert case in CASES
        cases[case]=run(data,manifest['symbols'],case)
        print('PASS actual-linked STOP/clock resume',case,flush=True)
    (out/'stop-mode-thumb-results.json').write_text(json.dumps(dict(
        complete=set(cases)==set(CASES),bin_sha256=manifest['bin_sha256'],cases=cases,
        model_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        hardware_tested=False),indent=2)+'\n')
