"""Replay an audited V15/V16 signal through the production calibration sequencer."""
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys

ROOT=Path(__file__).resolve().parent
OUT=Path(sys.argv[1]);OUT.mkdir(parents=True,exist_ok=True)
VERSION=sys.argv[2] if len(sys.argv)==3 else 'V15'
assert len(sys.argv) in (2,3) and VERSION in ('V15','V16')
CAPTURE=ROOT.parent/f'firmware/normal-control-host/live-{VERSION.lower()}/calibration-1/full-trace'
pins={'audit.json':'7a04143dd8658bf638bc3f7189f4d953de53f945cd5a5e67c84959cddd3324fd',
      'sram-00.bin':'03eecd06e52888d50910a777cba50963d86f4f9761154d76ab65eceb76570716',
      'sram-01.bin':'4b63cd148f73e39c6dbf42bf1b6e8506dc7cf90ab53f4c7cb4a89da065d697d9'}
if VERSION=='V16':
    pins={'audit.json':'e6a20a163004344efad4ff6c205c72ed04219b2d8f7fbb5db84b5143c9099cfb',
          'sram-00.bin':'b3d563ea8c7f2bba2a6751e0560bc1b9967cbee61ade6f0a0365772314500e10',
          'sram-01.bin':'2e46f0e14f1c18cf01d9cc9514befc7dfb4a2c6f28f431a32050b52d88a4952a'}
for name,digest in pins.items():
    assert hashlib.sha256((CAPTURE/name).read_bytes()).hexdigest()==digest,name
audit=json.loads((CAPTURE/'audit.json').read_text())
assert audit['driver_readings_complete'] and len(audit['records'])==2250
assert audit['sram_sha256']==[pins['sram-00.bin'],pins['sram-01.bin']]
trace=b''.join(struct.pack('<II',r['elapsed_ticks'],r['millivolts']) for r in audit['records'])
# Verify the replay input against the audited SRAM, not just a copied JSON claim.
for name in ('sram-00.bin','sram-01.bin'):
    data=(CAPTURE/name).read_bytes()
    header=struct.unpack_from('<8I',data,0xc7c)
    assert list(header)==audit['header']
    records=[struct.unpack_from('<II',data,0xc7c+32+8*i) for i in range(2250)]
    expected=b''.join(struct.pack('<II',(stamp-header[3]*10)&0xffffffff,mv) for stamp,mv in records)
    assert expected==trace
path=OUT/(VERSION.lower()+'-signal.trace');path.write_bytes(trace)
units=['test_calibration_trace.c','motor_calibration.c','motor_homing.c','motor_guard.c','motor_slowdown.c']
results=[]
for mode,flags in [('strict',[]),('sanitized',['-fsanitize=address,undefined',
        '-fno-omit-frame-pointer','-fno-sanitize-recover=all'])]:
    exe=OUT/('calibration-trace-'+mode)
    subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-pedantic','-O1','-g',
                    *flags,*(str(ROOT/name) for name in units),'-o',str(exe)],check=True)
    run=subprocess.run([str(exe),str(path),VERSION],check=True,text=True,capture_output=True)
    assert run.stdout.count('PASS '+VERSION+' trace replay:2250')==2
    print(run.stdout,end='',flush=True);results.append(dict(mode=mode,stdout=run.stdout))
result=dict(complete=True,hardware_actions=False,physical_endpoint_proven=False,
    source_capture=VERSION,
    signal_scope='2250 retained measured voltages/acquisition times; first pre-drive acquisition excluded',
    sequence_scope='One captured leg supplied as an initial probe with equal direction policies; not a reconstruction of historical direction/token/fallback',
    energized_counter_scope='Synthetic 1.5ms off per window, not reconstructed measured motor-on time',
    input_sha256=pins,trace_sha256=hashlib.sha256(trace).hexdigest(),results=results,
    sources={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in [*units,'test_calibration_trace.py']})
(OUT/'manifest.json').write_text(json.dumps(result,indent=2)+'\n')
print('PASS complete '+VERSION+' signal replay; no false endpoint or renewed deadline; no device access')
