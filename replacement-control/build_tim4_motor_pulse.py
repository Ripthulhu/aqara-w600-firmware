"""Offline actual-source host tests and Cortex-M0+ compile; no device tools."""
import hashlib,json,os,pathlib,subprocess
here=pathlib.Path(__file__).resolve().parent
sdk=pathlib.Path(os.environ.get('CIU32_SDK', '/srv/ai-agent/work/aqara-w600-hardware/sdk-build/vendor-sdk/CIU32L071 EclipseExampleProject'))
includes=['-isystem',str(sdk/'Drivers/CMSIS/Device'),'-isystem',str(sdk/'Drivers/CMSIS/Core')]
out=pathlib.Path(os.environ.get('BUILD_OUT',str(here/'build-motor-stop')));out.mkdir(exist_ok=True)

def run(args):
    print(' '.join(map(str,args)),flush=True)
    subprocess.run(list(map(str,args)),check=True,cwd=here)
flags=['-std=c11','-Wall','-Wextra','-Werror','-pedantic']
profiles={}
for profile,defines in [('',[]),('-display',['-DW600_DISPLAY_BENCH'])]:
    sources=[('test_tim4_motor_pulse.c','test-tim4-pulse')]
    if not profile:sources.append(('../startup/test_motor_stop.c','test-motor-stop'))
    for source,name in sources:
        for suffix,extra in [('',[]),('-sanitize',['-fsanitize=address,undefined','-fno-omit-frame-pointer','-g'])]:
            exe=out/(name+profile+suffix)
            run(['gcc',*flags,*includes,*defines,*extra,source,'-o',exe]);run([exe])
    objects=[]
    for source in ['tim4_motor_pulse.c','../startup/w600_motor_stop.c']:
        obj=out/(pathlib.Path(source).stem+profile+'.o');objects.append(obj)
        run(['arm-none-eabi-gcc',*flags,*includes,*defines,'-mcpu=cortex-m0plus','-mthumb','-Os',
             '-ffreestanding','-fno-builtin','-fstack-usage','-c',source,'-o',obj])
    combined=out/('pulse-with-motor-stop'+profile+'.o')
    run(['arm-none-eabi-gcc','-mcpu=cortex-m0plus','-mthumb','-nostdlib','-r',*objects,'-o',combined])
    undefined=subprocess.check_output(['arm-none-eabi-nm','-u',str(combined)],text=True).strip()
    assert not undefined,undefined
    with (out/('tim4_motor_pulse'+profile+'.dis')).open('w') as f:
        subprocess.run(['arm-none-eabi-objdump','-dr',str(combined)],check=True,stdout=f)
    profiles[profile or 'default']=hashlib.sha256(combined.read_bytes()).hexdigest()
manifest={'scope':'offline component tests/compile only; no device validation','undefined_symbols':undefined,
          'profile_objects':profiles,
          'sources':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in
             [here/'tim4_motor_pulse.c',here/'tim4_motor_pulse.h',here/'test_tim4_motor_pulse.c',
              here/'../startup/w600_motor_stop.c',here/'../startup/w600_motor_stop.h',
              here/'../startup/test_motor_stop.c',pathlib.Path(__file__)]}}
(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
