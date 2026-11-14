"""Replay real captures through the native candidate detector; no device access."""
import hashlib,json,struct,subprocess,sys
from pathlib import Path
root=Path(__file__).resolve().parent
sys.path.insert(0,str(root/'perturb-host'))
from decode_capture import decode

traces={};sources={}
for version in ('v15','v16'):
    path=root/f'normal-control-host/live-{version}/calibration-1/full-trace/audit.json'
    data=path.read_bytes();sources[str(path)]=hashlib.sha256(data).hexdigest()
    rows=json.loads(data)['records']
    traces[version]=[(r['elapsed_ticks'],r['millivolts']) for r in rows]
path=root/'confirmation-host/live-calibration-1/after-sram.bin'
b=path.read_bytes();sources[str(path)]=hashlib.sha256(b).hexdigest()
m=json.loads((root/'confirmation-host/inputs-r9/qualification.json').read_text())
offset=m['symbols']['w600_motor_trace']-0x20000000
header=struct.unpack_from('<8I',b,offset)
assert header[0]==0x57365452 and header[5]==2250 and header[6:]==(0,0)
rows=[struct.unpack_from('<2I',b,offset+32+8*i) for i in range(header[5])]
traces['v19']=[(at-rows[0][0],mv) for at,mv in rows]
image=(root/'perturb-host/live-r5-run1/bench.bin').read_bytes()
for run in (1,2,3):
    path=root/f'perturb-host/live-r5-run{run}/sram-01.bin'
    b=path.read_bytes();sources[str(path)]=hashlib.sha256(b).hexdigest()
    report=decode(b,image)
    for name in ('backoff_samples','current_samples'):
        rows=report[name];start=rows[0][0]
        traces[f'r5-{run}-{name}']=[((at-start)&0xffffffff,mv) for at,mv in rows]
results={}
for name,rows in traces.items():
    data=''.join(f'{at} {mv}\n' for at,mv in rows)
    p=subprocess.run(['wsl','-d','Ubuntu-24.04','--',
        '/srv/ai-agent/work/aqara-w600-hardware/slowdown-candidate-20261109/test','replay'],
        input=data,text=True,capture_output=True,check=True)
    results[name]=dict(samples=len(rows),result=p.stdout.strip())
report=dict(sources=sources,results=results,detector='>=200ms high>=1100mV average baseline; <=85% baseline for200ms; maxgap21ms',
            physical_endpoint_proven=False,firmware_integrated=False,hardware_tested=False,
            limitation='Only a provisional load-change candidate. Requires bounded back-off/return and travel consistency; captures do not establish a general endpoint threshold.')
(root/'slowdown-replay-20261109.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(results,indent=2))
