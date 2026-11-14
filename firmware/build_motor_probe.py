"""Offline actual-source service tests/ARM compile only; never contacts a device."""
import hashlib,json,os,pathlib,subprocess
here=pathlib.Path(__file__).resolve().parent
root=here.parent
sdk=pathlib.Path(os.environ.get('CIU32_SDK','/srv/ai-agent/work/aqara-w600-hardware/sdk-build/vendor-sdk/CIU32L071 EclipseExampleProject'))
out=here/'build-motor-probe';out.mkdir(exist_ok=True)
inc=['-isystem',str(sdk/'Drivers/CMSIS/Device'),'-isystem',str(sdk/'Drivers/CMSIS/Core')]
flags=['-std=c11','-Wall','-Wextra','-Werror','-Wshadow','-pedantic']
units=[here/'motor_probe.c',root/'replacement-control/motor_probe_sequence.c',
       root/'replacement-control/tim4_motor_pulse.c',root/'startup/w600_motor_stop.c',
       root/'startup/w600_feedback_board.c',root/'startup/w600_feedback.c']
def run(args):
    print(' '.join(map(str,args)),flush=True)
    subprocess.run(list(map(str,args)),check=True,cwd=here)
for name,extra in [('strict',['-O2']),('sanitized',['-O1','-g','-fsanitize=address,undefined',
                                          '-fno-omit-frame-pointer','-fno-sanitize-recover=all'])]:
    objs=[]
    for source in units:
        obj=out/(source.stem+'-'+name+'.o');objs.append(obj)
        conversion=['-Wconversion'] if source.name in ('motor_probe.c','motor_probe_sequence.c') else []
        run(['gcc',*flags,*conversion,*inc,*extra,'-DMOTOR_PROBE_TEST_HEADER_ONLY',
             '-include',here/'test_motor_probe.c','-c',source,'-o',obj])
    exe=out/('test-motor-probe-'+name)
    run(['gcc',*flags,*inc,*extra,here/'test_motor_probe.c',*objs,'-o',exe]);run([exe])
arm=[]
for source in units:
    obj=out/(source.stem+'-arm.o');arm.append(obj)
    run(['arm-none-eabi-gcc',*flags,*inc,'-mcpu=cortex-m0plus','-mthumb','-Os','-ffreestanding',
         '-fno-builtin','-fstack-usage','-c',source,'-o',obj])
combined=out/'motor-probe-combined.o'
run(['arm-none-eabi-gcc','-mcpu=cortex-m0plus','-mthumb','-nostdlib','-r',*arm,'-o',combined])
undefined=subprocess.check_output(['arm-none-eabi-nm','-u',str(combined)],text=True).strip()
assert all(line.split()[-1] in ('memset','memcpy') for line in undefined.splitlines()),undefined
with (out/'motor-probe.dis').open('w') as f:
    subprocess.run(['arm-none-eabi-objdump','-dr',str(combined)],check=True,stdout=f)
files=units+[here/'motor_probe.h',here/'test_motor_probe.c',pathlib.Path(__file__),
    root/'replacement-control/motor_probe_sequence.h',root/'replacement-control/tim4_motor_pulse.h',
    root/'startup/w600_motor_stop.h',root/'startup/w600_feedback.h',root/'startup/w600_feedback_board.h']
(out/'manifest.json').write_text(json.dumps({'scope':'offline source integration, not linked app/live qualification',
    'undefined_symbols':undefined,'sources':{str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()
    for p in files}},indent=2)+'\n')
