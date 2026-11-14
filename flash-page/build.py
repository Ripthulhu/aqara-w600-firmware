"""Offline actual-source tests and SRAM-only Cortex-M0+ linkage; never a device."""
import hashlib
import json
from pathlib import Path
import re
import subprocess

HERE=Path(__file__).resolve().parent
BUILD=HERE/'build'

def run(args):
    try: return subprocess.check_output(args,cwd=HERE,text=True,stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError as error:
        print(error.output)
        raise

def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    BUILD.mkdir(exist_ok=True)
    results=[]
    for name,extra in [('strict',[]),('sanitized',['-fsanitize=address,undefined','-fno-omit-frame-pointer'])]:
        binary=BUILD/f'test-{name}'
        print(run(['gcc','-std=c11','-O2','-g','-Wall','-Wextra','-Werror','-DFLASH_PAGE_HOST_TEST',
                   *extra,'flash_page.c','test_flash_page.c','-o',str(binary)]),end='')
        results.append(run([str(binary)]).strip()); print(results[-1])
    sdk=HERE.parent/'vendor-sdk'/'CIU32L071 EclipseExampleProject'/'Drivers'/'CMSIS'
    if not sdk.exists(): sdk=Path('/srv/ai-agent/work/aqara-w600-hardware/sdk-build/vendor-sdk/CIU32L071 EclipseExampleProject/Drivers/CMSIS')
    if not sdk.exists(): raise FileNotFoundError('Vendor CMSIS required for register static assertions')
    includes=['-I'+str(sdk/'Device'),'-I'+str(sdk/'Core')]
    flags=['-mcpu=cortex-m0plus','-mthumb','-std=c11','-Os','-g','-ffreestanding','-fno-builtin',
           '-fno-common','-fno-unwind-tables','-fno-asynchronous-unwind-tables','-fstack-usage',
           '-Wall','-Wextra','-Werror']
    print(run(['arm-none-eabi-gcc',*flags,*includes,'-c','verify_regs.c','-o',str(BUILD/'verify_regs.o')]),end='')
    print(run(['arm-none-eabi-gcc',*flags,'-c','flash_page.c','-o',str(BUILD/'flash_page.o')]),end='')
    elf=BUILD/'flash_page.elf'; binary=BUILD/'flash_page.bin'
    print(run(['arm-none-eabi-gcc','-mcpu=cortex-m0plus','-mthumb','-nostdlib','-Wl,--build-id=none',
               '-Wl,-Map='+str(BUILD/'flash_page.map'),'-T','flash_page.ld',str(BUILD/'flash_page.o'),'-o',str(elf)]),end='')
    print(run(['arm-none-eabi-objcopy','-O','binary',str(elf),str(binary)]),end='')
    dis=run(['arm-none-eabi-objdump','-d',str(elf)]); (BUILD/'flash_page.dis').write_text(dis)
    symbols=run(['arm-none-eabi-nm','-n',str(elf)]); (BUILD/'symbols.txt').write_text(symbols)
    assert not run(['arm-none-eabi-nm','-u',str(elf)]).strip()
    assert not re.search(r'\b(blx|bkpt|cpsid|cpsie|wfi|wfe)\b',dis)
    for target in re.findall(r'\bbl\s+([0-9a-f]+)\b',dis):
        assert 0x20002000<=int(target,16)<0x20002000+binary.stat().st_size
    su=(BUILD/'flash_page.su').read_text(); (BUILD/'stack-usage.txt').write_text(su)
    sizes=[int(line.split('\t')[1]) for line in su.splitlines()]
    # Conservative sum of every static frame remains below the guarded512B.
    assert sum(sizes)<512, su
    sources=['flash_page.c','flash_page.h','flash_regs.h','flash_page.ld','verify_regs.c','test_flash_page.c','build.py']
    manifest={'offline_only':True,'not_a_runner':True,'base':'0x20002000','size':binary.stat().st_size,
              'bin_sha256':sha(binary),'elf_sha256':sha(elf),'sources':{n:sha(HERE/n) for n in sources},
              'tests':results,'conservative_sum_static_frames':sum(sizes),
              'limits':['No hardware test','No safe takeover/deployment/recovery implementation',
                        'SysTick and iteration budgets cannot preempt an AHB stall or stopped CPU',
                        'Caller owns power, SRAM call chain/vectors/stack and all excluded writers']}
    (BUILD/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(manifest,indent=2))

if __name__=='__main__': main()
