"""Relink the qualified image and exercise the reserved-page linker assertions."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile

assert len(sys.argv)==2,'Usage: check_settings_layout.py <qualified R2 build>'
build=Path(sys.argv[1]).resolve()
manifest=json.loads((build/'manifest.json').read_bytes())
keys=list(manifest['sources'])
units=keys[:keys.index('firmware/radio.h')]
assert len(units)==27
linker=Path(__file__).with_name('flash-app.ld')
text=linker.read_text()
for setting in ('__settings_page0 = 0x0800ee00;','__settings_page1 = 0x0800f000;'):
    assert text.count(setting)==1
command=['arm-none-eabi-gcc','-mcpu=cortex-m0plus','-mthumb','-nostdlib',
    '-nostartfiles','-Wl,--build-id=none','-Wl,--gc-sections','-Wl,--fatal-warnings',
    *[str(build/(Path(n).stem+'.o')) for n in units],'-lgcc']
with tempfile.TemporaryDirectory() as directory:
    root=Path(directory)
    elf=root/'positive.elf';binary=root/'positive.bin'
    subprocess.run([*command,'-T',str(linker),'-o',str(elf)],check=True)
    subprocess.run(['arm-none-eabi-objcopy','-O','binary',str(elf),str(binary)],check=True)
    assert hashlib.sha256(binary.read_bytes()).hexdigest()==manifest['bin_sha256']
    for i,(a,b) in enumerate(((0x08009e00,0x0800a000),(0x0800ee00,0x0800f020),
                              (0x08010600,0x08010800),(0x0800ee04,0x0800f004))):
        fixture=root/f'negative-{i}.ld'
        fixture.write_text(text.replace('__settings_page0 = 0x0800ee00;',f'__settings_page0 = 0x{a:x};').replace(
            '__settings_page1 = 0x0800f000;',f'__settings_page1 = 0x{b:x};'))
        result=subprocess.run([*command,'-T',str(fixture),'-o',str(root/f'negative-{i}.elf')],
                              capture_output=True,text=True)
        assert result.returncode and 'Settings pages overlap code/staging' in result.stderr,(i,result.stderr)
print('PASS settings linker: exact R2 binary unchanged;4 code/spacing/staging/alignment refusals; no device access')
