"""Boot the STOP-enabled binary and exercise the real wake-to-command/UI paths."""
import hashlib
import json
from pathlib import Path
import struct
import sys

import test_sleep_clock_thumb as clock
from uart_wire_stress import WireCore

app, service, RCC, LP = clock.app, clock.service, clock.RCC, clock.LP
EXTI, NVIC, SCR = 0x40021800, 0xe000e100, 0xe000ed10
LINES = (1 << 1) | (1 << 2) | (1 << 8) | (1 << 10)
IRQS = (1 << 5) | (1 << 6) | (1 << 7) | (1 << 30)
CASES = ('timer', 'zigbee_early', 'zigbee_late', 'button', 'clockwise', 'counterclockwise', 'opposite')
RECOVERY_CASES = ('radio_recovery', 'stop_recovery')


class DeepCore(WireCore, clock.ClockCore):
    model_deep_stop = True

    def __init__(self, data, symbols, scenario):
        self.wake = scenario
        self.stops = 0
        self.wake_at = None
        self.pll_ready_at = None
        self.rx_start_at = None
        self.restoring = False
        super().__init__(data, symbols, 3)
        self.put(0x40022024, 0x7121)
        self.put(service.PA + 0x10, self.get(service.PA + 0x10) | (1 << 10))

    def advance_us(self, elapsed):
        if self.rx_start_at is not None and self.virtual_us + elapsed >= self.rx_start_at:
            assert not self.restoring and self.get(RCC + 8) == 0x12, 'Clock not ready for the first start bit'
            self.rx_start_at = None
        super().advance_us(elapsed)
        if self.low_running:
            top, count = self.get(LP + 0x18), self.get(LP + 0x1c)
            if top != 65535 and count >= top:
                self.put(LP, self.get(LP) | 2)
                self.put(LP + 0x1c, count % (top + 1))
        if self.pll_ready_at is not None and self.virtual_us >= self.pll_ready_at:
            self.put(RCC, self.get(RCC) | (1 << 25))
            self.pll_ready_at = None

    def write(self, cpu, access, address, size, value, user):
        if address == EXTI + 8:
            self.pending.append((address, self.get(address) & ~value))
        elif address == LP + 4:
            self.pending.append((LP, self.get(LP) & ~value))
        elif address == NVIC:
            self.pending.append((NVIC, self.get(NVIC) | value))
        elif address == NVIC + 0x80:
            self.pending.append((NVIC, self.get(NVIC) & ~value))
        elif address == NVIC + 0x180:
            self.pending.append((NVIC + 0x100, self.get(NVIC + 0x100) & ~value))
        elif address == service.ICSR:
            assert value == 1 << 25 and not self.pending_tick
            self.pending.append((address, self.get(address) & ~service.PENDING))
        elif address == RCC + 0x34 and (value | self.get(address)) & clock.GATE and (value ^ self.get(address)) == 1 << 28:
            assert self.off() and not self.get(LP + 8)
        elif address == RCC and self.restoring:
            self.pll_ready_at = self.virtual_us + 55
        elif address == RCC + 8 and self.restoring:
            self.restoring = False
            self.pending.append((address, value | 0x10))
        elif address not in (SCR, EXTI, EXTI + 4, EXTI + 0x50, EXTI + 0x54, EXTI + 0x70):
            super().write(cpu, access, address, size, value, user)

    def code(self, cpu, address, size, user):
        if address not in self.wfi_sites:
            return super().code(cpu, address, size, user)
        if not self.get(SCR):
            # Shallow WFI wakes on the next UART byte, not just on SysTick.
            until_tick = 1000 - (self.virtual_us - self.systick_started) % 1000
            if self.wire and not (self.rx or self.pending_tick or self.uart_error):
                until_byte = self.wire[0][0] - self.virtual_us
                if 0 <= until_byte < until_tick:
                    assert self.get(NVIC) == 1 << 27 and self.get(service.ST) & 7 == 7
                    self.idle_sleep_count += 1
                    self.idle_sleep_us += until_byte
                    self.advance_us(until_byte)
                    cpu.reg_write(app.base.UC_ARM_REG_PC, (address + 2) | 1)
                    return
            return super().code(cpu, address, size, user)
        assert self.get(SCR) == 4 and not self.get(service.ST)
        assert self.get(NVIC) == IRQS | (1 << 27) and self.get(LP + 8) == 2
        assert self.get(EXTI + 0x70) & LINES == LINES
        assert self.get(EXTI + 4) & LINES == LINES
        assert self.get(EXTI + 0x50) & 0x770 == 0x10
        assert self.get(EXTI + 0x54) & 0x707 == 1
        assert self.off() and self.selectors_low() and self.dial_bias_ready()
        assert not self.rx and not self.restoring
        self.stops += 1
        wake = self.wake if self.stops == 1 else 'timer'
        lines, irq = {
            'zigbee_early': (1 << 10, 1 << 7), 'zigbee_late': (1 << 10, 1 << 7),
            'button': (1 << 8, 1 << 7), 'clockwise': (1 << 2, 1 << 6),
            'counterclockwise': (1 << 1, 1 << 5), 'opposite': (6, (1 << 5) | (1 << 6)),
            'timer': (0, 1 << 30),
        }[wake]
        if wake == 'timer':
            top, count = self.get(LP + 0x18), self.get(LP + 0x1c)
            self.advance_us(((top - count) * 1000000 - self.low_fraction + self.low_hz - 1) // self.low_hz)
            assert self.get(LP) & 2
        else:
            self.advance_us(15000)
        self.put(EXTI + 8, lines)
        self.put(NVIC + 0x100, irq)
        self.put(RCC, 0x500); self.put(RCC + 8, 0); self.put(RCC + 12, 0x20260)
        self.restoring = True
        self.wake_at = self.virtual_us
        if wake == 'button':
            self.pressed = True
            self.levels(service.PB)
        if wake.startswith('zigbee'):
            delay = 250 if wake == 'zigbee_early' else 12000
            self.rx_start_at = self.wake_at + delay
            self.schedule(app.base.Frame(0x65, 2, bytes.fromhex('0700040866')), self.rx_start_at + 87)
        # Both dial and radio contacts are modeled as already high on return.
        cpu.reg_write(app.base.UC_ARM_REG_PC, (address + 2) | 1)

    def stop_report(self):
        return struct.unpack('<9I', self.cpu.mem_read(self.symbols['stop_report'], 36))


def run(data, symbols, scenario):
    m = DeepCore(data, symbols, scenario)
    m.boot()
    assert m.get(symbols['w600_stop_bench_enable']) == 1
    start_target = app.base.Frame(0x60, 2, bytes.fromhex('07000407d0'))
    m.rx.extend(app.base.encode(start_target))
    app.until(m, lambda: app.reply_count(m) == 1 and m.clock_report()[0] == 5 and
              m.report()[23] >= 1 and m.get(symbols['vbat_report'] + 36) == 1 and
              m.get(service.PA + 0x14) & (1 << 11))
    m.put(symbols['app_ticks'], 10000)
    if scenario == 'timer':
        app.until(m, lambda: m.stop_report()[7] >= 3)
        assert 11400 <= m.get(symbols['app_ticks']) <= 11600
        assert m.display_report()[5] == 0x10000
    elif scenario.startswith('zigbee'):
        ack = app.base.Frame(0x65, 3, bytes.fromhex('000700'))
        try:
            app.until(m, lambda: ack in (app.frames(m) or []))
        except AssertionError as error:
            raise AssertionError((scenario, m.stop_report(), m.first_overrun, m.arrived,
                                  m.virtual_us - m.wake_at, m.report())) from error
        assert m.stops == 1 and m.stop_report()[5] == 1 and not m.overruns and m.rx_start_at is None
        assert m.arrived == len(app.base.encode(app.base.Frame(0x65, 2, bytes.fromhex('0700040866'))))
        assert m.display_report()[5] == 0x10000
        assert m.get(symbols['radio']) & 0xffff == 2150
        app.until(m, lambda: m.stop_report()[7] >= 1)
    else:
        expected = {'button': 2000, 'clockwise': 2050, 'counterclockwise': 1950, 'opposite': 2000}[scenario]
        app.until(m, lambda: m.display_report()[5] == expected and m.display_report()[2] == 0)
        assert m.stops == 1 and m.stop_report()[6] == 1
        start = m.virtual_us
        app.until(m, lambda: m.virtual_us - start >= 60000)
        assert m.display_report()[5] == expected and m.get(symbols['radio']) & 0xffff == expected
        assert not m.report()[6], 'Wake became a pairing request'
    assert not m.timer_runs and m.off() and m.report()[2:4] == (2, 0)
    assert m.stop_report()[2] == 0 and not m.get(SCR)
    clock.released(m)
    return dict(wake=scenario, stop_report=m.stop_report(), display_target=m.display_report()[5],
                uart_overruns=m.overruns, max_receive_age_us=m.max_receive_age,
                motor_attempts=len(m.timer_runs))


class RecoveryCore(DeepCore):
    def __init__(self, data, symbols, fault):
        self.fault = fault
        self.reset_requests = 0
        super().__init__(data, symbols, 'timer')

    def code(self, cpu, address, size, user):
        if address == self.symbols['fatal']:
            self.fatal_steps = -64  # Let the reset guard complete before the model stops.
        super().code(cpu, address, size, user)

    def write(self, cpu, access, address, size, value, user):
        if address == 0xe000ed0c:
            assert size == 4 and value == 0x05fa0004 and self.off() and self.selectors_low()
            assert not self.get(0x40022010) & 0x10000 and not self.timer_runs
            self.reset_requests += 1
        else:
            super().write(cpu, access, address, size, value, user)
            if self.fault == 'stop_recovery' and self.stops and address == EXTI + 0x54:
                self.pending.append((address, value ^ 1))


def recovery_run(data, symbols, fault):
    m = RecoveryCore(data, symbols, fault)
    m.boot()
    if fault == 'stop_recovery':
        app.until(m, lambda: m.clock_report()[0] == 5 and m.get(symbols['vbat_report'] + 36) == 1)
        m.put(symbols['app_ticks'], 10000)
    else:
        m.uart_error = 8
    app.until(m, lambda: m.reason == 'fatal')
    assert m.report()[2:5] == (0xe0, 17 if fault == 'stop_recovery' else 7, 1)
    assert m.reset_requests == 1
    cookie = symbols['recovery']
    assert m.get(cookie) == 0x57365243 and m.get(cookie + 4) & 255 == m.report()[3]
    # A fresh core supplies the reset bootloader/clock handoff, with only the
    # recovery record carried over. Verify it responds before injecting again.
    r = RecoveryCore(data, symbols, 'radio_recovery')
    r.put(cookie, m.get(cookie)); r.put(cookie + 4, m.get(cookie + 4))
    r.boot()
    assert r.get(symbols['w600_stop_bench_enable']) == 0 and r.report()[2:4] == (2, 0)
    r.rx.extend(app.base.encode(app.base.Frame(0x60, 2, bytes.fromhex('07000407d0'))))
    app.until(r, lambda: app.reply_count(r) == 1)
    assert not r.stops and r.off()
    r.uart_error = 8
    app.until(r, lambda: r.reason == 'fatal')
    assert r.reset_requests == 0 and r.off(), 'A repeating fault must not reboot-loop'
    return dict(cause=m.report()[3], detail=m.get(cookie + 4) >> 8, radio_reply_after_restart=True,
                deep_sleep_disabled=True, reset_requests=m.reset_requests, repeat_reset_requests=0)


if __name__ == '__main__':
    out = Path(sys.argv[1])
    manifest = json.loads((out / 'manifest.json').read_text())
    data = (out / 'replacement-core.bin').read_bytes()
    assert manifest['deep_sleep_enabled'] and hashlib.sha256(data).hexdigest() == manifest['bin_sha256']
    cases = {}
    expected = CASES + RECOVERY_CASES
    for case in tuple(sys.argv[2:]) or expected:
        assert case in expected
        cases[case] = (recovery_run if case in RECOVERY_CASES else run)(data, manifest['symbols'], case)
        print('PASS linked deep-sleep wake:', case, flush=True)
    (out / 'deep-sleep-thumb-results.json').write_text(json.dumps(dict(
        complete=set(cases) == set(expected), bin_sha256=manifest['bin_sha256'], cases=cases,
        modeled_pll_lock_us=55, hardware_tested=False), indent=2) + '\n')
