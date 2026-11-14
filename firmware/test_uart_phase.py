"""Check finite-register traffic with shifted wire timing; no device access."""
import hashlib
import json
from pathlib import Path
import sys

import uart_wire_stress as wire


def main():
    directory=Path(sys.argv[1])
    manifest=json.loads((directory/'manifest.json').read_text())
    data=(directory/'replacement-core.bin').read_bytes()
    assert hashlib.sha256(data).hexdigest()==manifest['bin_sha256']
    original=wire.WireCore
    results=[]
    for phase in (29,58):
        class ShiftedCore(original):
            def schedule(self,frame,at):super().schedule(frame,at+phase)
        wire.WireCore=ShiftedCore
        result=wire.run(data,manifest,'calibration')
        result['wire_shift_us']=phase
        results.append(result)
        print(json.dumps(result),flush=True)
    wire.WireCore=original
    (directory/'uart-phase-results.json').write_text(json.dumps(dict(
        complete=True,bin_sha256=manifest['bin_sha256'],cases=results,
        all_clean=all(r['clean'] for r in results),hardware_tested=False,
        cycle_accurate=False),indent=2)+'\n')
    assert all(r['clean'] for r in results),results


if __name__=='__main__':main()
