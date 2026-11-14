"""SHA-pinned measured signals through the calibration-only candidate path.

Acquisition times/voltages are real. Energized counts use an explicitly synthetic
1.5ms off-window model; no distance, mounted force or reference is inferred.
"""
import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess
import sys

ROOT=Path(__file__).resolve().parent
EVIDENCE=Path(os.environ.get('W600_CONFIRMATION_EVIDENCE_ROOT',str(ROOT.parent)))
OUT=Path(sys.argv[1]);OUT.mkdir(parents=True,exist_ok=True)
sys.path.insert(0,str(EVIDENCE/'firmware/perturb-host'))
from decode_capture import decode

host=EVIDENCE/'firmware/perturb-host'
image=(host/'live-r5-run1/bench.bin').read_bytes()
pins={
    'r5-1':('591af93567ab75d401fdcb8458282a9b45d8ed48ca9b620beb87c489f51b3c93',19000,1),
    'r5-2':('ff9aba4ca5f87d31e418eb47da171b22f83cda276a47634753104801ae558025',18600,61),
    'r5-3':('8e0b567f5b4bf9a2ac73ac06abc1f30a730243a521a082fc08f05647103023f3',18800,1),
    'V16':('2e46f0e14f1c18cf01d9cc9514befc7dfb4a2c6f28f431a32050b52d88a4952a',435200,1),
    'V15':('4b63cd148f73e39c6dbf42bf1b6e8506dc7cf90ab53f4c7cb4a89da065d697d9',0,0),
}
traces=[]
for label,(digest,at,mv) in pins.items():
    if label.startswith('r5'):
        data=(host/f'live-r5-run{label[-1]}/sram-01.bin').read_bytes()
        assert hashlib.sha256(data).hexdigest()==digest
        decoded=decode(data,image);leg=decoded['legs'][1]
        assert decoded['complete_diagnostic'] and leg['stop_reason']=='unqualified low'
        records=[((stamp-leg['generation_ms']*10)&0xffffffff,value)
                 for stamp,value in decoded['current_samples']][1:]
        assert records[-1]==(at,mv)
    else:
        folder=EVIDENCE/f'firmware/normal-control-host/live-{label.lower()}/calibration-1/full-trace'
        data=(folder/'sram-01.bin').read_bytes()
        assert hashlib.sha256(data).hexdigest()==digest
        header=struct.unpack_from('<8I',data,0xc7c)
        assert header[:3]==(0x57365452,1,2256) and header[5]==2250 and header[6:]==(0,0)
        records=[((stamp-header[3]*10)&0xffffffff,value)
                 for stamp,value in (struct.unpack_from('<II',data,0xc7c+32+i*8) for i in range(1,2250))]
    path=OUT/(label+'.trace');path.write_bytes(b''.join(struct.pack('<II',*row) for row in records))
    traces.append((label,path,at,mv))
units=['test_motor_confirmation.c','motor_calibration.c','motor_homing.c','motor_guard.c','motor_slowdown.c']
results=[]
for mode,flags in [('strict',[]),('sanitized',['-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-sanitize-recover=all'])]:
    exe=OUT/('confirmation-trace-'+mode)
    subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-pedantic','-O1','-g',
                    *flags,*(str(ROOT/name) for name in units),'-o',str(exe)],check=True)
    for label,path,at,mv in traces:
        run=subprocess.run([str(exe),str(path),str(at),str(mv)],check=True,text=True,capture_output=True)
        assert run.stdout.count('PASS measured signal replay:')==2
        print(label,mode,run.stdout.strip(),flush=True)
        results.append(dict(label=label,mode=mode,stdout=run.stdout))
(OUT/'manifest.json').write_text(json.dumps(dict(complete=True,hardware_actions=False,
    physical_reference_proven=False,pins=pins,results=results,
    energized_counter_scope='Synthetic1.5ms off per20ms cycle; not historical measured motor-on time',
    sources={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
             for name in [*units,'test_confirmation_trace.py']}),indent=2)+'\n')
