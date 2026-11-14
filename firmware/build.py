"""Build RAM bench applications or a persistent application development image."""
import argparse
import bisect
from concurrent.futures import ThreadPoolExecutor, as_completed
import functools
import hashlib
import json
import os
from pathlib import Path
import re
import struct
import subprocess
import sys

ROOT=Path(__file__).resolve().parent
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument("--raw-probe-bench",action="store_true")
parser.add_argument("--display-probe-bench",action="store_true")
parser.add_argument("--motor-service-bench",action="store_true")
parser.add_argument("--flash-application",action="store_true")
parser.add_argument("--development-link-only",action="store_true",help="Compile/test/link a FLASH development image; leave runtime qualification pending")
parser.add_argument("--controller-revision",type=int,default=46)
parser.add_argument("--motor-direction",type=int,choices=(1,2),default=2)
args=parser.parse_args()
flash=args.flash_application
if not 1<=args.controller_revision<=65535:parser.error("Controller revision outside BE16 range")
if flash and not 46<=args.controller_revision<=255:parser.error("OTA-enabled application revision must be46..255")
if args.development_link_only and not flash:parser.error("Development link-only requires --flash-application")
service=args.motor_service_bench or flash
if flash and (args.raw_probe_bench or args.display_probe_bench or args.motor_service_bench):
    parser.error("The persistent application is not an automatic bench motor run")
if service and (args.raw_probe_bench or args.display_probe_bench):
    parser.error("Motor service replaces, rather than combines with, the raw probe")
display=args.display_probe_bench or service
probe=args.raw_probe_bench or args.display_probe_bench
OUT=Path(os.environ.get("W600_BUILD_DIR",str(ROOT/("build-application" if flash else "build-service" if service else "build-display" if display else "build-probe" if probe else "build-stop"))))
OUT.mkdir(exist_ok=True)
SDK=Path(os.environ.get("CIU32_SDK","/srv/ai-agent/work/aqara-w600-hardware/sdk-build/vendor-sdk/CIU32L071 EclipseExampleProject"))
PREFIX=os.environ.get("ARM_TOOL_PREFIX","arm-none-eabi-")
warnings=["-std=gnu11","-Wall","-Wextra","-Werror"]
control=ROOT.parent/"replacement-control"
tests=[(ROOT/"test_app.c",[control/"w600_buttons.c",control/"w600_endpoint.c"],[]),(ROOT/"test_radio.c",
        [ROOT/"radio.c",control/"w600_frame.c",control/"w600_endpoint.c",control/"w600_pairing.c"],
        ["-DW600_RADIO_TEST"])]
tests.append((ROOT/"test_controller_revision.c",[control/"w600_endpoint.c"],[]))
for name,extra in (("frame",["w600_frame.c"]),("endpoint",["w600_frame.c","w600_endpoint.c"]),
                   ("pairing",["w600_frame.c","w600_pairing.c"]),
                   ("temperature",["w600_temperature.c"]),("buttons",["w600_buttons.c"]),
                   ("settings",["w600_settings.c"]),
                   ("settings_store",["w600_settings.c","w600_settings_store.c"]),
                   ("settings_service",["w600_settings.c","w600_settings_store.c","w600_settings_service.c"])):
    tests.append((control/("test_w600_"+name+".c"),[control/p for p in extra],[]))
if display:
    tests += [(ROOT/"test_app.c",[control/"w600_display.c",control/"w600_buttons.c",control/"w600_endpoint.c"],["-DW600_DISPLAY_BENCH=1"]),
              (ROOT/"test_display_tx.c",[ROOT/"display_tx.c"],["-DW600_DISPLAY_TX_TEST=1"]),
              (control/"test_w600_display.c",[control/"w600_display.c"],[]),
              (ROOT.parent/"startup/test_display_board.c",[],
               ["-isystem",str(SDK/"Drivers/CMSIS/Core"),
                "-isystem",str(SDK/"Drivers/CMSIS/Device")])]
if service:
    includes=["-isystem",str(SDK/"Drivers/CMSIS/Core"),"-isystem",str(SDK/"Drivers/CMSIS/Device")]
    tests += [(ROOT/"test_app_clock.c",[],includes),
              (ROOT.parent/"startup/test_feedback.c",[],includes),
              (ROOT/"test_motor_service.c",[control/"motor_homing.c",control/"motor_guard.c",ROOT/"motor_trace.c"],includes),
              (ROOT/"test_motor_trace.c",[ROOT/"motor_trace.c"],[]),
              (ROOT/"test_temperature_adc.c",[control/"w600_temperature.c"],includes),
              (ROOT/"test_temperature_adc.c",[control/"w600_temperature.c"],includes+["-DW600_DISPLAY_BENCH=1"])]
    for name in ("motor_calibration","motor_position"):
        tests.append((control/("test_"+name+".c"),[control/(name+".c"),
            *([control/"motor_homing.c",control/"motor_guard.c",control/"motor_slowdown.c"] if name=="motor_calibration" else [])],[]))
    tests.append((control/"test_motor_slowdown.c",[control/"motor_slowdown.c"],[]))
if flash:
    tests.append((ROOT/"test_ota_auth.c",[ROOT/"ota_auth.c"],["-DW600_OTA_AUTH_TEST=1"]))
    tests.append((ROOT/"test_ota_receiver.c",[ROOT/"ota_receiver.c"],[]))
    tests.append((ROOT/"test_ota_chain.c",[ROOT/"ota_receiver.c",ROOT/"ota_auth.c",ROOT/"ota_flash.c"],
                  ["-DW600_OTA_AUTH_TEST=1","-DW600_OTA_FLASH_TEST=1"]))
    tests.append((ROOT/"test_ota_flash.c",[ROOT/"ota_flash.c"],["-DW600_OTA_FLASH_TEST=1"]))
    tests.append((ROOT/"test_radio.c",[ROOT/"radio.c",ROOT/"ota_receiver.c",control/"w600_frame.c",
                  control/"w600_endpoint.c",control/"w600_pairing.c"],
                  ["-DW600_RADIO_TEST","-DW600_OTA_RECEIVER=1"]))
    tests.append((ROOT/"test_vbat_adc.c",[],includes))
    tests.append((ROOT/"test_vbat_adc.c",[],includes+["-DW600_DISPLAY_BENCH=1"]))
    tests.append((ROOT/"test_sleep_clock.c",[],includes))
    tests.append((ROOT/"test_stop_mode.c",[],includes))
    tests.append((ROOT/"test_settings_flash.c",[ROOT/"settings_flash.c"],["-DW600_NV_TEST=1"]))
    tests.append((control/"test_motor_confirmation.c",[control/"motor_calibration.c",
        control/"motor_homing.c",control/"motor_guard.c",control/"motor_slowdown.c"],[]))
    tests.append((ROOT/"test_motion_end_rounding.c",[control/"motor_calibration.c",control/"motor_homing.c",
        control/"motor_guard.c",control/"motor_position.c",control/"w600_endpoint.c",control/"motor_slowdown.c"],[]))
    tests.append((ROOT/"test_valve_calibration.c",[control/"motor_calibration.c",
        control/"motor_position.c",control/"motor_homing.c",control/"motor_guard.c",
        control/"w600_endpoint.c",control/"motor_slowdown.c"],[]))
results=[]
for source,extra_sources,test_flags in tests:
    for mode,flags in (("strict",[]),("sanitized",["-fsanitize=address,undefined",
                            "-fno-omit-frame-pointer","-fno-sanitize-recover=all"])):
        label=source.stem+("-display" if "-DW600_DISPLAY_BENCH=1" in test_flags else "")+(
            "-ota" if "-DW600_OTA_RECEIVER=1" in test_flags else "")
        binary=OUT/(label+"-"+mode)
        subprocess.check_call(["gcc",*warnings,"-O1","-g",*flags,
                               "-DW600_CONTROLLER_REVISION="+str(args.controller_revision),*test_flags,str(source),
                               *map(str,extra_sources),"-o",str(binary)])
        result=subprocess.check_output([str(binary)],text=True)
        print(result.strip())
        results.append({"test":label,"mode":mode,"result":result.strip()})

units=[ROOT/"reset.S",ROOT/"app.c",ROOT/"radio.c",
       ROOT.parent/"startup/runtime/w600_runtime.c",
       ROOT.parent/"startup/ciu32_startup.c",ROOT.parent/"startup/w600_board.c",
       ROOT.parent/"startup/w600_motor_stop.c",
       control/"w600_frame.c",control/"w600_endpoint.c",control/"w600_pairing.c",control/"w600_buttons.c"]
if probe:
    units += [ROOT/"motor_probe.c",control/"motor_probe_sequence.c",
              control/"tim4_motor_pulse.c",ROOT.parent/"startup/w600_feedback.c",
              ROOT.parent/"startup/w600_feedback_board.c"]
if display:
    units += [ROOT/"display_tx.c",control/"w600_display.c",
              ROOT.parent/"startup/w600_display_board.c"]
if service:
    units += [ROOT/"app_clock.c",ROOT/"motor_service.c",ROOT/"motor_trace.c",control/"motor_homing.c",control/"motor_guard.c",
              ROOT/"temperature_adc.c",control/"w600_temperature.c",
              ROOT.parent/"startup/w600_feedback.c",ROOT.parent/"startup/w600_feedback_board.c"]
if flash:
    units += [ROOT/"ota_auth.c",ROOT/"ota_receiver.c",ROOT/"ota_flash.c",ROOT/"vbat_adc.c",ROOT/"sleep_clock.c",ROOT/"stop_mode.c",ROOT/"valve_calibration.c",control/"motor_calibration.c",control/"motor_position.c",control/"motor_slowdown.c",ROOT/"settings_flash.c",
              ROOT/"settings_app.c",control/"w600_settings.c",control/"w600_settings_store.c",control/"w600_settings_service.c"]
linker=ROOT/("flash-app.ld" if flash else "ram-service.ld" if service else "ram-display.ld" if display else "ram-probe.ld" if probe else "ram.ld")
code_start=0x08002000 if flash else 0x20000c00 if service else 0x20001000 if display else 0x20002000
ram_data_limit=0x20006000 if flash else code_start
flags=[*warnings,"-mcpu=cortex-m0plus","-mthumb","-Os","-g3","-ffreestanding",
       "-fno-builtin","-fno-common","-fno-stack-protector","-fno-jump-tables",
       "-ffunction-sections","-fdata-sections","-fno-unwind-tables",
       "-fno-asynchronous-unwind-tables","-fstack-usage",
       "-isystem",str(SDK/"Drivers/CMSIS/Core"),"-isystem",str(SDK/"Drivers/CMSIS/Device")]
if probe: flags.append("-DW600_RAW_PROBE_BENCH=1")
if display: flags.append("-DW600_DISPLAY_BENCH=1")
if service: flags.append("-DW600_MOTOR_SERVICE=1")
if args.motor_service_bench: flags += ["-DW600_MOTOR_SERVICE_BENCH=1","-DW600_MOTOR_DIRECTION="+str(args.motor_direction)]
if flash: flags += ["-DW600_FLASH_APPLICATION=1","-DW600_OTA_RECEIVER=1"]
flags += ["-DW600_CONTROLLER_REVISION="+str(args.controller_revision)]
objects=[]
for unit in units:
    obj=OUT/(unit.stem+".o")
    subprocess.check_call([PREFIX+"gcc",*flags,"-c",str(unit),"-o",str(obj)])
    objects.append(obj)
if flash:
    subprocess.check_call([PREFIX+'gcc',*flags,'-c',str(ROOT/'verify_settings_flash_regs.c'),
                           '-o',str(OUT/'verify-settings-regs.o')])
control_layout=None
confirmation_layout=None
settings_layout=None
radio_diagnostics_layout=None
if flash:
    layout_obj=OUT/"test-control-layout.o"
    layout_bin=OUT/"test-control-layout.bin"
    subprocess.check_call([PREFIX+"gcc",*flags,"-c",str(ROOT/"test_control_layout.c"),"-o",str(layout_obj)])
    subprocess.check_call([PREFIX+"objcopy","-O","binary","-j",".rodata.w600_test_control_layout",str(layout_obj),str(layout_bin)])
    control_layout=list(struct.unpack("<3I",layout_bin.read_bytes()))
    confirmation_layout_bin=OUT/"test-confirmation-layout.bin"
    subprocess.check_call([PREFIX+"objcopy","-O","binary","-j",".rodata.w600_test_confirmation_layout",str(layout_obj),str(confirmation_layout_bin)])
    confirmation_layout=list(struct.unpack("<23I",confirmation_layout_bin.read_bytes()))
    settings_layout_bin=OUT/"test-settings-layout.bin"
    subprocess.check_call([PREFIX+"objcopy","-O","binary","-j",".rodata.w600_test_settings_layout",str(layout_obj),str(settings_layout_bin)])
    settings_layout=list(struct.unpack("<15I",settings_layout_bin.read_bytes()))
    radio_layout_bin=OUT/"test-radio-diagnostics-layout.bin"
    subprocess.check_call([PREFIX+"objcopy","-O","binary","-j",".rodata.w600_test_radio_diagnostics_layout",str(layout_obj),str(radio_layout_bin)])
    radio_diagnostics_layout=list(struct.unpack("<10I",radio_layout_bin.read_bytes()))
    assert radio_diagnostics_layout[0]==1 and radio_diagnostics_layout[2]==36
elf=OUT/"replacement-core.elf"
subprocess.check_call([PREFIX+"gcc","-mcpu=cortex-m0plus","-mthumb","-nostdlib","-nostartfiles",
                       "-Wl,--build-id=none","-Wl,--gc-sections","-Wl,--fatal-warnings",
                       "-Wl,-Map="+str(OUT/"replacement-core.map"),"-T",str(linker),
                       *map(str,objects),*(["-lgcc"] if service else []),"-o",str(elf)])
assert not subprocess.check_output([PREFIX+"nm","-u",str(elf)],text=True).strip()
symbols={}
for line in subprocess.check_output([PREFIX+"nm","-n",str(elf)],text=True).splitlines():
    m=re.fullmatch(r"([0-9a-fA-F]+)\s+\w\s+(\S+)",line)
    if m: symbols[m[2]]=int(m[1],16)
binary=OUT/"replacement-core.bin"
subprocess.check_call([PREFIX+"objcopy","-O","binary",str(elf),str(binary)])
if flash:
    from ota_image import seal
    sealed=seal(binary.read_bytes(),args.controller_revision)
    identity=OUT/'ota-identity.bin'
    identity.write_bytes(sealed[-64:])
    subprocess.check_call([PREFIX+'objcopy','--update-section','.ota_identity='+str(identity),str(elf)])
    subprocess.check_call([PREFIX+'objcopy','-O','binary',str(elf),str(binary)])
    assert binary.read_bytes()==sealed
data=binary.read_bytes()
vectors=struct.unpack_from("<48I",data)
assert vectors[0]==0x20007000 and vectors[1]==symbols["W600_Reset_Handler"]|1
assert vectors[15]==symbols["W600_SysTick_Handler"]|1
assert vectors[43]==symbols["W600_UART3_4_Handler"]|1
assert all(v in (0,symbols["W600_Default_Handler"]|1) for i,v in enumerate(vectors) if i not in (0,1,15,43))
assert symbols["__bss_end"]<=ram_data_limit and symbols["app_report"]==0x20006a00
assert symbols["w600_vectors"]==code_start
code_limit=0x0800ee00 if flash else 0x20006980 if service else 0x20006800 if probe else 0x20006a00
assert symbols["__data_load"]+symbols["__data_size"]<=code_limit
assert len(data)<=code_limit-code_start
if probe: assert symbols["probe_report"]==0x20006800
if service: assert symbols["motor_report"]==0x20006980
image=elf.read_bytes()
header=struct.unpack_from("<HHIIIIIHHHHHH",image,16)
assert image[:7]==b"\x7fELF\x01\x01\x01" and header[0]==2 and header[1]==40
assert header[3]==vectors[1] and header[8]==32
segments=[]
for i in range(header[9]):
    kind,offset,vaddr,paddr,filesz,memsz,permissions,align=struct.unpack_from("<8I",image,header[4]+32*i)
    if kind!=1: continue
    assert filesz<=memsz
    assert (0x20000000<=vaddr and vaddr+memsz<=ram_data_limit) or (code_start<=vaddr and vaddr+memsz<=(code_limit if flash else 0x20006c00)) or (flash and 0x20006000<=vaddr and vaddr+memsz<=0x20006c00)
    if filesz: assert code_start<=paddr and paddr+filesz<=code_limit
    segments.append({"vaddr":vaddr,"paddr":paddr,"filesz":filesz,"memsz":memsz,"flags":permissions})
assert segments
dis=subprocess.check_output([PREFIX+"objdump","-d",str(elf)],text=True)
(OUT/"replacement-core.dis").write_text(dis)
assert not re.search(r"\b(bkpt|svc|wfe)\b",dis)
sleep_sites=[]; current_function=None
for line in dis.splitlines():
    head=re.match(r"^[0-9a-f]+ <([^>]+)>:$",line)
    if head: current_function=head[1]
    if re.search(r"\bwfi\b",line): sleep_sites.append(current_function)
assert sorted(sleep_sites)==(["w600_app_idle","w600_stop_mode"] if flash else []),sleep_sites
if not service: assert not re.search(r"\bblx\b",dis)
assert b"\xf0\xed\x00\xe0" not in data, "No target debug-register access"

# Count linked direct calls and cross-function branches conservatively as nested.
def normalized(name): return re.sub(r"\.(?:constprop|isra|part|cold)(?:\.\d+)?","",name.split("@",1)[0])
frames={"W600_Reset_Handler":8,"W600_Default_Handler":0}
for file in OUT.glob("*.su"):
    for line in file.read_text().splitlines():
        source,size,kind=line.split("\t")
        assert kind=="static",(file,line)
        name=normalized(source.rsplit(":",1)[-1])
        frames[name]=max(frames.get(name,0),int(size))
raw_heads=[(int(a,16),name) for a,name in re.findall(r"^([0-9a-f]+) <([^>]+)>:$",dis,re.M)]
names=[name for _,name in raw_heads]
heads=[(a,name+"@"+format(a,"x") if names.count(name)>1 else name) for a,name in raw_heads]
head_names=dict(heads)
assert len(head_names)==len(heads) and len(set(head_names.values()))==len(heads)
addresses=[a for a,_ in heads]
edges={name:set() for _,name in heads}
current=None
for line in dis.splitlines():
    m=re.match(r"^([0-9a-f]+) <([^>]+)>:$",line)
    if m: current=head_names[int(m[1],16)]; continue
    m=re.match(r"^\s*[0-9a-f]+:\s+(?:[0-9a-f]{4}\s+){1,2}(b(?:l|eq|ne|cs|cc|mi|pl|vs|vc|hi|ls|ge|lt|gt|le)?(?:\.[nw])?)\s+([0-9a-f]+)\s+<",line)
    if m:
        index=bisect.bisect_right(addresses,int(m[2],16))-1
        assert index>=0
        callee=heads[index][1]
        if callee!=current: edges[current].add(callee)
if service:
    indirect=[]; current=None
    for line in dis.splitlines():
        m=re.match(r"^[0-9a-f]+ <([^>]+)>:$",line)
        if m: current=m[1]
        if re.search(r"\bblx\b",line): indirect.append(current)
    indirect_targets={"stop":["service_stop"]}
    if flash:
        indirect_targets.update({"read_pages":["read_page"],"w600_settings_save":["replace_page"],
            "verify":["ota_quiesce","ota_stage_read","ota_authorize"],
            "w600_ota_process":["ota_quiesce","ota_stage_write"],"w600_ota_poll":["ota_commit"],
            "w600_ota_flash_commit":["ota_authorize"]})
        indirect_targets["stop"].append("calibration_stop")
        indirect_targets['w600_ota_authorize_image']=['ota_stage_read']
    assert set(indirect)==set(indirect_targets),(indirect,indirect_targets)
    expected_indirect={"stop":1}
    if flash:expected_indirect.update({"verify":4,"w600_ota_process":3,"w600_ota_poll":1,
        "w600_ota_flash_commit":2,"read_pages":2,"w600_settings_save":1})
    if flash:expected_indirect['w600_ota_authorize_image']=2
    assert {name:indirect.count(name) for name in set(indirect)}==expected_indirect,indirect
    for caller,callees in indirect_targets.items():
        for callee in callees: edges[caller].add(callee)
    for address,name in heads:
        if not name.endswith('_veneer'):continue
        body=dis.split('<'+name+'>:\n',1)[1].split('\n\n',1)[0]
        targets=re.findall(r'\.word\s+0x([0-9a-f]+)',body)
        assert len(targets)==1 and re.search(r'\bbx\s+ip\b',body),(name,body)
        target=int(targets[0],16)&~1
        assert target in head_names,(name,hex(target))
        edges[name].add(head_names[target])
        frames[name]=8
    # libgcc has no .su records: count every emitted push/sub, even on separate paths.
    for _,name in heads:
        if name not in frames and (name.startswith("__") or name.startswith(".div")):
            body=dis.split("<"+name+">:\n",1)[1].split("\n\n",1)[0]
            reserve=0
            for line in body.splitlines():
                if "push" in line:
                    regs=line.split("{",1)[1].split("}",1)[0].split(",")
                    reserve+=4*len(regs)
                m=re.search(r"sub\s+sp,\s*#(\d+)",line)
                if m: reserve+=int(m[1])
            frames[name]=reserve
visiting=set()
@functools.lru_cache(None)
def bound(name):
    assert name not in visiting,"Unexpected call/branch cycle: "+name
    assert normalized(name) in frames,"Missing stack frame: "+name
    visiting.add(name)
    child=max((bound(n) for n in edges[name]),default=(0,[]),key=lambda x:x[0])
    visiting.remove(name)
    return frames[normalized(name)]+child[0],[name]+child[1]
fg,fg_path=bound("W600_Reset_Handler")
uart,uart_path=bound("W600_UART3_4_Handler")
tick,tick_path=bound("W600_SysTick_Handler")
fault,fault_path=bound("W600_Default_Handler")
stack_bound=fg+uart+tick+fault+3*36
assert stack_bound<=1024,(stack_bound,fg_path,uart_path,tick_path,fault_path)
nv_stack=None
if flash:
    lo,hi=symbols['__nvcode_start'],symbols['__nvcode_end']
    assert lo==0x20006000 and lo<hi<=0x20006980
    for name in ('w600_settings_flash_write','w600_ota_flash_page_write','w600_settings_flash_fault','w600_settings_tick_handler',
                 'W600_UART3_4_Handler','w600_radio_irq','latch_error'):
        assert lo<=symbols[name]<hi,(name,symbols[name])
    nv_dis=subprocess.check_output([PREFIX+'objdump','-d','-j','.nvcode',str(elf)],text=True)
    for target in re.findall(r'\b(?:bl|b(?:eq|ne|cs|cc|mi|pl|vs|vc|hi|ls|ge|lt|gt|le)?(?:\.[nw])?)\s+([0-9a-f]+)\s+<',nv_dis):
        assert lo<=int(target,16)<hi,('FLASH-dependent RAM branch',target)
    assert not re.search(r'\bblx\b',nv_dis)
    assert all(x=='lr' for x in re.findall(r'\bbx\s+(\w+)',nv_dis))
    nv_fg,nv_path=bound('w600_settings_flash_write')
    ota_nv_fg,ota_nv_path=bound('w600_ota_flash_page_write')
    if ota_nv_fg>nv_fg:nv_fg,nv_path=ota_nv_fg,ota_nv_path
    nv_stack=nv_fg+uart+3*36
    assert nv_stack<=1024,(nv_stack,nv_path,uart_path)
inputs=units+[ROOT/"radio.h",linker,ROOT/"test_app.c",ROOT/"test_radio.c",
              ROOT/"radio_test_mmio.h",ROOT/"build.py",ROOT/"test_thumb.py",
              ROOT.parent/"radio_frames.py",control/"w600_frame.h",
              control/"w600_endpoint.h",control/"w600_pairing.h",control/"w600_buttons.h",
              ROOT.parent/"startup/w600_board.h",ROOT.parent/"startup/w600_motor_stop.h",
              ROOT.parent/"startup/ciu32_startup.h",
              ROOT.parent/"startup/runtime/w600_runtime.h"]
inputs += [control/"w600_temperature.c",control/"w600_temperature.h"]
inputs += [control/"w600_settings.c",control/"w600_settings.h"]
inputs += [control/"w600_settings_store.c",control/"w600_settings_store.h"]
inputs += [control/"w600_settings_service.c",control/"w600_settings_service.h"]
if probe:
    inputs += [ROOT/"motor_probe.h",ROOT/"probe_thumb.py",
               control/"motor_probe_sequence.h",control/"tim4_motor_pulse.h",
               ROOT.parent/"startup/w600_feedback.h",ROOT.parent/"startup/w600_feedback_board.h"]
if display:
    inputs += [ROOT/"display_tx.h",ROOT/"display_tx_test_mmio.h",ROOT/"display_thumb.py",
               control/"w600_display.h",ROOT.parent/"startup/w600_display_board.h"]
inputs += [source for source,_,_ in tests]
if service:
    inputs += [ROOT/"app_clock.h",ROOT/"motor_service.h",ROOT/"service_thumb.py",ROOT/"probe_thumb.py",
               ROOT/"temperature_adc.h",ROOT/"motor_trace.h",
               control/"motor_homing.h",control/"motor_guard.h",
               control/"motor_calibration.h",control/"motor_position.h",control/"motor_slowdown.h",control/"test_motor_slowdown.c",
               ROOT.parent/"startup/w600_feedback.h",ROOT.parent/"startup/w600_feedback_board.h"]
if flash: inputs += [ROOT/"ota_auth.h",ROOT/"ota_image.py",ROOT/"ota_receiver.h",ROOT/"ota_flash.h",ROOT/"vbat_adc.h",ROOT/"stop_mode.h",ROOT/"test_stop_mode_thumb.py",ROOT/"sleep_clock.h",ROOT/"test_sleep_clock_thumb.py",ROOT/"test_sleep_cancel_thumb.py",ROOT/"settings_flash.h",ROOT/"settings_app.h",ROOT/"settings_thumb.py",ROOT/'verify_settings_flash_regs.c',ROOT.parent/"flash-page/flash_regs.h",ROOT/"application_thumb.py",ROOT/"valve_calibration.h",ROOT/"test_display_idle_thumb.py",
    control/"test_motor_confirmation.c",
    control/"motor_calibration.h",control/"motor_position.h",control/"motor_slowdown.h",ROOT/"control_thumb.py",
    ROOT/"test_control_layout.c",ROOT/"test_motor_service_thumb.py",ROOT/"sampling_stress.py",
    ROOT/"uart_wire_stress.py",ROOT/"test_continuous_home_thumb.py",ROOT/"test_buttons_thumb.py",
    ROOT/"test_confirmation_fault_thumb.py",
    ROOT/"ota_thumb.py",
    ROOT/"test_fallback_thumb.py",ROOT/"test_probe_escape_thumb.py",ROOT/"test_slow_start_thumb.py"]
manifest={"kind":"persistent application development image, normal controls incomplete" if flash else "feedback motor-service + display/radio bench application, RAM-only" if service else "raw-probe bench application, motor UNPLUGGED, RAM-only" if probe else "motor-disabled replacement application core, RAM-only",
          "hardware_tested":False,"flash_programming":False,"motor_control_enabled":probe or service,
          "flash_application":flash,"automatic_startup_motion":args.motor_service_bench,
          "motor_service_bench":args.motor_service_bench,"motor_direction":args.motor_direction if args.motor_service_bench else None,
          "linked_service_model_pending":service,
          "development_link_only":args.development_link_only,
          "settings_flash_worker_linked":flash,"settings_storage_enabled":flash,
          "ota_receiver_enabled":flash,"ota_commit_authorization_enabled":flash,
          "controller_revision":args.controller_revision,"ota_update_policy":"paired Zigbee network; newer W600 image with retained OTA capability and SHA256 integrity" if flash else None,
          "ota_radio_package_acceptance_validated":False,"ota_hardware_validated":False,
          "settings_worker_stack_bound_bytes":nv_stack,
          "raw_probe_bench":probe,"radio_controlled_motion_enabled":flash,
          "radio_thermostat_regulation_enabled":flash,
          "radio_calibration_enabled":flash,
          "critical_motor_sampling_isolated":flash,
          "motor_sample_off_window_us":1500 if service else None,
          "display_probe_bench":args.display_probe_bench,
          "display_enabled":display,
          "screen_timeout_ms":10000 if display else None,
          "idle_cpu_sleep_enabled":flash,"deep_sleep_enabled":False,
          "deep_sleep_bench_path_linked":flash,"deep_sleep_bench_gate_default":0,
          "battery_runtime_validated":False,
          "internal_vbat_diagnostic_once_per_boot":flash,
          "battery_voltage_publication_enabled":False,
          "battery_percentage_publication_enabled":False,
          "slow_clock_rate_check_enabled":flash,
          "slow_clock_hardware_validated":False,
          "onboard_temperature_acquisition_enabled":service,
          "local_temperature_reports_enabled":service,
          "local_temperature_rf_delivery_validated":False,
          "temperature_hardware_accuracy_validated":False,
          "code_address":code_start,
          "fresh_pairing_validated":False,"normal_valve_firmware_complete":False,
          "test_control_layout":control_layout,
          "test_confirmation_layout":confirmation_layout,
          "test_settings_layout":settings_layout,
          "test_radio_diagnostics_layout":radio_diagnostics_layout,
          "entry":vectors[1],"report_address":symbols["app_report"],"segments":segments,
          "motor_report_address":symbols["motor_report"] if service else None,
          "motor_report_version":2 if service else None,
          "bin_sha256":hashlib.sha256(data).hexdigest(),"binary_bytes":len(data),
          "elf_sha256":hashlib.sha256(image).hexdigest(),"symbols":symbols,"host_tests":results,
          "arm_compiler":subprocess.check_output([PREFIX+"gcc","--version"],text=True).splitlines()[0],
          "host_compiler":subprocess.check_output(["gcc","--version"],text=True).splitlines()[0],
          "stack_bound_bytes":stack_bound,"stack_reserve_bytes":1024,
          "stack_paths":{"foreground":fg_path,"uart":uart_path,"tick":tick_path,"fault":fault_path},
          "stack_scope":"All emitted static frames on linked direct/cross-branch paths plus UART, SysTick and one fault exception; not silicon latency or corrupt-SP proof.",
          "sources":{str(p.relative_to(ROOT.parent)):hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},
          "device_header_sha256":hashlib.sha256((SDK/"Drivers/CMSIS/Device/ciu32l071.h").read_bytes()).hexdigest()}
(OUT/"manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
if args.development_link_only:
    assert manifest['linked_service_model_pending'] is True
elif flash:
    # Independent processes read the pending manifest and write distinct result files.
    models=[
        ('ota_models','ota_thumb.py','ota-thumb-results.json',[]),
        ('stop_mode_models','test_stop_mode_thumb.py','stop-mode-thumb-results.json',[]),
        ('sleep_clock_models','test_sleep_clock_thumb.py','sleep-clock-thumb-results.json',[]),
        ('sleep_cancel_models','test_sleep_cancel_thumb.py','sleep-cancel-thumb-results.json',[]),
        ('confirmation_fault_models','test_confirmation_fault_thumb.py','confirmation-fault-thumb-results.json',[]),
        ('thumb_models','application_thumb.py','application-thumb-results.json',[]),
        ('continuous_home_models','test_continuous_home_thumb.py','continuous-home-thumb-results.json',[]),
        ('uart_wire_models','uart_wire_stress.py','uart-wire-results.json',['--require-clean']),
        ('sampling_stress_models','sampling_stress.py','sampling-stress-results.json',['--require-clean']),
        ('normal_control_models','control_thumb.py','control-thumb-results.json',[]),
        ('button_models','test_buttons_thumb.py','buttons-thumb-results.json',[]),
        ('settings_models','settings_thumb.py','settings-thumb-results.json',[]),
        ('display_idle_models','test_display_idle_thumb.py','display-idle-thumb-results.json',[]),
    ]
    with ThreadPoolExecutor(max_workers=3) as pool:
        futures={pool.submit(subprocess.check_call,[sys.executable,str(ROOT/script),str(OUT),*extra]):
                 (key,result_file) for key,script,result_file,extra in models}
        try:
            for future in as_completed(futures):
                future.result()
                key,result_file=futures[future]
                report=json.loads((OUT/result_file).read_text())
                assert report['complete']
                if key!='continuous_home_models':
                    assert report['bin_sha256']==manifest['bin_sha256']
                if key in ('uart_wire_models','sampling_stress_models'):
                    assert report['all_clean']
                manifest[key]=report
        except BaseException:
            for future in futures: future.cancel()
            raise
    manifest['linked_service_model_pending']=False
elif not service:
    subprocess.check_call([sys.executable,str(ROOT/("display_thumb.py" if display else "probe_thumb.py" if probe else "test_thumb.py")),str(OUT)])
    manifest["thumb_models"]=json.loads((OUT/"thumb-results.json").read_text())
else:
    model=ROOT/("application_thumb.py" if flash else "service_thumb.py")
    subprocess.check_call([sys.executable,str(model),str(OUT)])
    manifest["thumb_models"]=json.loads((OUT/("application-thumb-results.json" if flash else "service-thumb-results.json")).read_text())
    assert manifest["thumb_models"]["complete"]
    assert manifest["thumb_models"]["bin_sha256"]==manifest["bin_sha256"]
    manifest["linked_service_model_pending"]=False
(OUT/"manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
print(json.dumps({k:v for k,v in manifest.items() if k not in ("sources","symbols","host_tests","stack_paths","thumb_models")},indent=2))
print("PASS link/vectors, static stack bound and host tests. Not flashed or live-approved.")
if args.development_link_only:print("DEVELOPMENT LINK ONLY: runtime models pending; not qualified for installation.")
elif service: print("PASS combined actual-linked peripheral/handler model; real timing/RF and pinned live workflow still unverified.")
