"""Offline portable adapter tests/compile only. No device or network actions."""
import hashlib
import json
import pathlib
import subprocess

here = pathlib.Path(__file__).resolve().parent
out = here / 'build-motor-probe-sequence'
out.mkdir(exist_ok=True)
flags = ['-std=c11', '-Wall', '-Wextra', '-Werror', '-Wconversion', '-Wshadow', '-pedantic']

def run(args):
    print(' '.join(map(str, args)), flush=True)
    subprocess.run(list(map(str, args)), cwd=here, check=True)

for suffix, extra in [('', ['-O2']), ('-sanitize', ['-O1', '-g', '-fsanitize=address,undefined',
                                               '-fno-omit-frame-pointer'])]:
    exe = out / ('test-motor-probe-sequence' + suffix)
    run(['gcc', *flags, *extra, 'motor_probe_sequence.c', 'test_motor_probe_sequence.c', '-o', exe])
    run([exe])
obj = out / 'motor_probe_sequence.o'
run(['arm-none-eabi-gcc', *flags, '-mcpu=cortex-m0plus', '-mthumb', '-Os', '-ffreestanding',
     '-fno-builtin', '-fstack-usage', '-c', 'motor_probe_sequence.c', '-o', obj])
undefined = subprocess.check_output(['arm-none-eabi-nm', '-u', str(obj)], text=True).strip()
with (out / 'motor_probe_sequence.dis').open('w') as f:
    subprocess.run(['arm-none-eabi-objdump', '-dr', str(obj)], check=True, stdout=f)
files = [here / name for name in ['motor_probe_sequence.c', 'motor_probe_sequence.h',
    'test_motor_probe_sequence.c', 'build_motor_probe_sequence.py', 'tim4_motor_pulse.h',
    '../startup/w600_feedback.h']]
(out / 'manifest.json').write_text(json.dumps({
    'scope': 'portable raw pulse/acquisition evidence only; not linked into firmware',
    'undefined_symbols': undefined,
    'sources': {str(p.relative_to(here)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
}, indent=2) + '\n')
