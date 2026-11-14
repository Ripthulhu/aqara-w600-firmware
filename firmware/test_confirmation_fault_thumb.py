"""Actual-linked confirmation and display faults in its rests; no device access.

The real UART command, controller, driver, cleanup and display-failure path run.
Only peripheral ADC values and one display baud fault are injected. No firmware
state/reference is seeded, and no internal struct offsets are assumed.
"""
import hashlib
import json
from concurrent.futures import ProcessPoolExecutor
from itertools import repeat
from multiprocessing import get_context
from pathlib import Path
import sys

from unicorn.arm_const import UC_ARM_REG_R1, UC_ARM_REG_R2

import application_thumb as app


class ConfirmationCore(app.ApplicationCore):
    def __init__(self, data, symbols):
        self.move_adc_start = 0
        self.stop_proofs = set()
        self.low_batch_started = False
        self.low_batches = 0
        super().__init__(data, symbols, "normal")

    def code(self, cpu, address, size, user):
        if address == self.symbols["w600_motor_service_begin_bounded"]:
            index = len(self.timer_runs)
            assert index < 4, "Unexpected additional calibration action"
            direction, cap = ((2, 45000), (1, 3200), (2, 10200), (1, 45000))[index]
            assert cpu.reg_read(UC_ARM_REG_R1) == direction
            assert cpu.reg_read(UC_ARM_REG_R2) == cap
            self.direction = direction
            self.motor_limit_ms = cap
            self.move_adc_start = len(self.adc_launches)
            self.low_batch_started = False
        if address == self.symbols["motor_calibration_confirm_stop"]:
            self.stop_proofs.add(len(self.timer_runs))
        super().code(cpu, address, size, user)

    def write(self, cpu, access, address, size, value, user):
        if address in (app.service.TIM + 0x34, app.service.TIM + 0x38) and value == self.motor_limit_ms:
            assert not self.low_batch_started, "Drive restored after the candidate low batch"
        super().write(cpu, access, address, size, value, user)
        if (address == app.service.ADC and value == 4 and
                self.get(app.service.ADC + 0x1c) == 0x10000):
            group = (len(self.adc_launches) - 1 - self.move_adc_start) // 6
            raw = 1 if group == 0 else 1700
            # HOME has fresh sustained motion, then a pre-stop declining band.
            # Backoff remains moving until its timed stop, never an endpoint.
            transition = 90 if len(self.timer_runs) == 3 else 100
            if len(self.timer_runs) != 2 and group >= transition:
                raw = (900, 600, 300, 1)[min(group - transition, 3)]
                if group >= transition + 3 and not self.low_batch_started:
                    self.low_batch_started = True
                    self.low_batches += 1
            at, channel, _ = self.adc_launches[-1]
            self.adc_launches[-1] = (at, channel, raw)
            if self.adc_pending is not None:
                self.adc_pending = (self.adc_pending[0], channel, raw)


def stopped(m):
    report = m.motor_report()
    return (m.off() and m.selectors_low() and report[2:5] == (7, 0, 1) and
            report[10:13] == (0, 0, 0) and not m.get(app.service.RCC + 0x34) & 4)


def run(data, symbols, layout, scenario):
    actions = {"candidate_rest": 1, "backoff_rest": 2, "confirmed_home": 4}[scenario]
    m = ConfirmationCore(data, symbols)
    m.boot()
    app.until(m, lambda: m.display_report()[3:5] == (1, 1))
    m.rx.extend(app.base.encode(app.base.Frame(0x60, 2, bytes.fromhex("07090001"))))
    injected_at = m.virtual_us
    if scenario == "confirmed_home":
        app.until(m, lambda: len(m.timer_runs) == 4, limit=90000)
        assert m.low_batches == 2 and {1, 2, 3} <= m.stop_proofs
        assert [run["arr"] for run in m.timer_runs] == [44999, 3199, 10199, 44999]
        field = lambda index: m.get(symbols["valve"] + layout[index])
        assert field(3) & 255 == 1 and field(4) & 255 == 4  # ACTIVE, MEASURE
        assert field(11) == 1 and field(8) == 4  # One confirmed HOME; four unique actions.
        assert field(12) & 255 == 3 and field(13) & 255 == 2
        assert field(14) == 1 and 0 < field(15) <= field(16)
        assert field(17) > 0 and field(18) <= 100
        assert field(19) & 65535 >= 2 and field(20) & 65535 >= 2
        assert not field(22) & 255
        assert m.get(symbols["display_failure_handled"]) == 0
        # The next full leg starts only after the confirmed HOME; cancel it by UART.
        replies = m.report()[9]
        injected_at = m.virtual_us
        m.rx.extend(app.base.encode(app.base.Frame(0x61, 2, bytes.fromhex("07030000"))))
        app.until(m, lambda: stopped(m) and m.report()[9] == replies + 1 and
                  app.frames(m) is not None and m.get(app.service.PA + 0x14) & (1 << 11))
        reply = next(frame for frame in app.frames(m) if frame.sequence == 0x61 and frame.kind == 3)
        assert reply.payload == bytes.fromhex("000703"), reply
    else:
        app.until(m, lambda: len(m.timer_runs) == actions and actions in m.stop_proofs and stopped(m),
                  limit=70000)
        assert m.low_batches == 1
        # The ordinary stop proof has happened, but the next action's rest has not.
        injected_at = m.virtual_us
        m.put(app.service.display.UART2 + 0x0c, 2499)
        app.until(m, lambda: m.get(symbols["display_failure_handled"]) == 1)
        app.until(m, lambda: m.display_report()[1] != 0 and app.frames(m) is not None)
        assert stopped(m) and len(m.timer_runs) == actions

        # A new transaction, not a duplicate calibration packet, must be rejected.
        replies = m.report()[9]
        m.rx.extend(app.base.encode(app.base.Frame(0x61, 2, bytes.fromhex("07090001"))))
        app.until(m, lambda: m.report()[9] == replies + 1 and app.frames(m) is not None and
                  m.get(app.service.PA + 0x14) & (1 << 11))
        reply = next(frame for frame in app.frames(m) if frame.sequence == 0x61 and frame.kind == 3)
        assert reply.payload == bytes.fromhex("030709"), reply
    app.until(m, lambda: m.virtual_us >= injected_at + 1250000)
    assert stopped(m) and len(m.timer_runs) == actions
    assert not m.get(app.service.RCC + 0x38) & 0x100000

    m.rx.extend(app.base.encode(app.base.Frame(0x62, 1, bytes.fromhex("070b071c"))))
    app.until(m, lambda: m.report()[9] == replies + 2 and app.frames(m) is not None and
              m.get(app.service.PA + 0x14) & (1 << 11))
    reply = next(frame for frame in app.frames(m) if frame.sequence == 0x62 and frame.kind == 4)
    assert reply.payload == bytes.fromhex("070b000000071c0100"), reply
    assert stopped(m) and len(m.timer_runs) == actions
    result = m.checks()
    result.update(scenario=scenario, intervention_at_us=injected_at,
                  motor_actions_before_and_after_intervention=actions,
                  new_calibration_rejected=scenario != "confirmed_home",
                  confirmed_home_then_full_leg=scenario == "confirmed_home",
                  full_leg_cancelled_by_uart=scenario == "confirmed_home",
                  reference_or_calibration_applied=False, no_restart_after_rest=True,
                  candidate_stopped_before_drive_restore=True,
                  state_seeding=False, hardware_tested=False, cycle_accurate=False)
    return result


def main():
    directory = Path(sys.argv[1])
    manifest = json.loads((directory / "manifest.json").read_text())
    data = (directory / "replacement-core.bin").read_bytes()
    assert manifest["flash_application"] and not manifest["automatic_startup_motion"]
    assert hashlib.sha256(data).hexdigest() == manifest["bin_sha256"]
    layout = manifest["test_confirmation_layout"]
    assert len(layout) == 23 and layout[0] == 1
    sources = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in
               (Path(__file__), Path(app.__file__), Path(app.service.__file__),
                Path(app.service.display.__file__), Path(app.service.probe.__file__), Path(app.base.__file__))}
    path = directory / "confirmation-fault-thumb-results.json"
    cases = {}
    scenarios = ("candidate_rest", "backoff_rest", "confirmed_home")
    with ProcessPoolExecutor(max_workers=3, mp_context=get_context("spawn")) as pool:
        results = pool.map(run, repeat(data), repeat(manifest["symbols"]), repeat(layout), scenarios)
        for scenario, result in zip(scenarios, results):
            cases[scenario] = result
            print("PASS actual-linked confirmation", scenario, flush=True)
            path.write_text(json.dumps(dict(complete=len(cases) == 3, bin_sha256=manifest["bin_sha256"],
                model_sources=sources, cases=cases, hardware_tested=False, cycle_accurate=False), indent=2) + "\n")


if __name__ == "__main__":
    main()
