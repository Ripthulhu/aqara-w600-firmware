"""Test the actual homing controller against pinned bench captures; no hardware."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent
OUT = Path(sys.argv[1])
OUT.mkdir(parents=True, exist_ok=True)
fixtures = [
    ('bemf-cadence20-host/live-1794180281/run-1794180281-trace.bin', '32127995f452eeb98a43fd1e88525f177f0e6aae228a167995239152c5b2f7b4'),
    ('bemf-cadence20-in35-host/live-1794181980/run-1794181980-trace.bin', 'aee7bcc6b2cccafe3e92e15bc6a6626f1f69f6f540e21663d6e4400cbcc6871b'),
    ('bemf-cadence20-out45-host/live-1794182880/run-1794182880-trace.bin', 'd63ea9bc205ac1e17ada678e43f26721cfb93cf7053d116ecc97097f44908e58'),
    ('bemf-cadence20-out30-host/live-1794181329/run-1794181329-trace.bin', 'be7850be5789e0a8d7cc4cfdc018af531fa0a30dde4fc0633b37910b40278798'),
]
paths = [ROOT.parent / name for name, _ in fixtures]
for path, (_, digest) in zip(paths, fixtures):
    assert hashlib.sha256(path.read_bytes()).hexdigest() == digest, path
results = []
sources = [ROOT/'motor_guard.c', ROOT/'motor_homing.c']
for name in ('test_motor_homing', 'test_motor_calibration', 'test_homing_replay'):
    units = sources + ([ROOT/'motor_calibration.c',ROOT/'motor_slowdown.c'] if name == 'test_motor_calibration' else []) + [ROOT/(name+'.c')]
    for mode, flags in [('strict', []), ('sanitized', ['-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-fno-sanitize-recover=all'])]:
        exe = OUT/(name+'-'+mode)
        subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-pedantic', '-O1', '-g', *flags, *map(str, units), '-o', str(exe)], check=True)
        run = subprocess.run([str(exe), *(map(str, paths) if name == 'test_homing_replay' else [])], check=True, text=True, capture_output=True)
        print(run.stdout, end='', flush=True)
        results.append({'test': name, 'mode': mode, 'stdout': run.stdout})
subprocess.run(['arm-none-eabi-gcc', '-mcpu=cortex-m0plus', '-mthumb', '-std=c11', '-ffreestanding', '-Wall', '-Wextra', '-Werror', '-pedantic', '-c', str(ROOT/'motor_homing.c'), '-o', str(OUT/'motor_homing.o')], check=True)
tracked = ['motor_homing.c','motor_homing.h','motor_guard.c','motor_guard.h','motor_calibration.c','motor_calibration.h','motor_slowdown.c','motor_slowdown.h','test_motor_homing.c','test_motor_calibration.c','test_homing_replay.c','build_homing_replay.py']
manifest = {'hardware_actions': False, 'mounted_calibration': False, 'fixtures': dict(fixtures), 'results': results,
            'source_sha256': {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in tracked}}
(OUT/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
print('PASS homing regression, calibration caller, pinned capture replays and ARM compile; no device access')
