"""Check gate-off cleanup in the real application; register model, no hardware."""
import hashlib
import json
from pathlib import Path
import struct
import sys
import test_sleep_clock_thumb as clock

app = clock.app
RCC = clock.RCC

def run(data, symbols, phase):
    m = clock.ClockCore(data, symbols, 'healthy')
    m.boot()
    app.until(m, lambda: m.clock_report()[0] == 5 and m.report()[23] >= 1 and
              m.report()[20] == 6 and app.frames(m) is not None and
              m.get(app.service.PA + 0x14) & (1 << 11))
    idle_before = m.idle_sleep_count
    app.until(m, lambda: m.idle_sleep_count >= idle_before + 2)
    state = symbols['sleep_clock']
    owned = 1 if phase == 1 else 3
    # Reproduce disabling the bench gate during each asynchronous arming phase.
    m.cpu.mem_write(state, struct.pack('<15I', phase, 0, owned, 0,
        m.virtual_us // 1000, m.virtual_us // 1000, 0, 0, 0, 32000, 2,
        0, 0, 1, symbols['app_ticks']))
    m.put(symbols['w600_stop_bench_enable'], 0)
    m.put(RCC + 0x44, 0x1c000003)
    if owned == 3:
        m.put(RCC + 0x34, 0x40060000)
        m.put(RCC + 0x3c, 0x100000)
        m.put(clock.LP + 0x10, 1)
    app.until(m, lambda: m.clock_report()[2] == 0, limit=800)
    clock.released(m)
    assert m.clock_report()[0:4] == (5, 0, 0, 0)
    assert m.clock_report()[9:11] == (32000, 2)
    assert not m.timer_runs and m.off() and m.report()[2:4] == (2, 0)
    return dict(phase=phase, resources_released=True, motor_attempts=0,
                rate_check_preserved=True)

if __name__ == '__main__':
    out = Path(sys.argv[1])
    manifest = json.loads((out / 'manifest.json').read_text())
    data = (out / 'replacement-core.bin').read_bytes()
    assert hashlib.sha256(data).hexdigest() == manifest['bin_sha256']
    cases = {}
    for phase in (1, 2, 3, 7):
        cases[str(phase)] = run(data, manifest['symbols'], phase)
        print('PASS actual-linked disabled sleep gate, phase', phase, flush=True)
    (out / 'sleep-cancel-thumb-results.json').write_text(json.dumps(dict(
        complete=True, cases=cases, bin_sha256=manifest['bin_sha256'],
        hardware_tested=False), indent=2) + '\n')
