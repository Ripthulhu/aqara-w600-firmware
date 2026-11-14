"""Linked app bus extension: real functions/handlers, modeled TIM4/ADC time.

Only this file extends the frozen test_thumb.Core. No device/network clients.
advance_us supplies virtual peripheral time; milliseconds manually invokes the
real SysTick once per millisecond and services real foreground code in125us
slots. This is not silicon IRQ dispatch, clock accuracy or electrical proof.
ADC conversions/calibration complete as timed register events, never C stubs.
The ordinary-off ADC path keeps PA15/PC10 LOW; no endpoint/calibration meaning.
"""
import importlib.util
import hashlib
import json
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("w600_frozen_core_bus", ROOT / "test_thumb.py")
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)

TIM, ADC, PMU = 0x40000800, 0x40012400, 0x40007000
RCC, PA, PB, PC = 0x40021000, 0x50000000, 0x50000400, 0x50000800


class ProbeCore(base.Core):
    """Narrow actual-linked bus; scenarios alter peripherals/scheduling only."""
    def __init__(self, data, symbols, scenario="normal"):
        assert scenario in ("normal", "radio", "foreground_stall", "adc_timeout", "timer_stall", "begin_failure", "power_latch_failure")
        self.virtual_us = 0
        self.timer_fraction = 0
        self.timer_runs = []
        self.timer_trace = []
        self.adc_events = []
        self.adc_launches = []
        self.adc_reads = []
        self.adc_pending = None
        self.cal_pending = None
        self.bgr_ready_at = None
        self.in_fault = False
        self.power_writes = []
        self.power_applied_at_us = None
        self.begin_calls = 0
        self.begin_readback_failed = False
        self.probe_scenario = scenario
        super().__init__(data, symbols, "quiet")
        self.put(0x1fff03c0, 0xfa2805d7)
        self.put(PA, 0xebffffff)
        self.put(PA + 0x0c, 0x24000000)
        self.put(PB, 0xffffffff)
        self.put(PC, 0xffdfffff)
        self.put(TIM + 0x2c, 0xffff)

    def selectors_low(self):
        return not ((self.get(PA + 0x14) | self.get(PA + 0x10)) & 0x8000 or
                    (self.get(PC + 0x14) | self.get(PC + 0x10)) & 0x400)

    def levels(self, port, mode=None):
        super().levels(port, mode)
        if port in (PA, PC):
            mask = 0x8000 if port == PA else 0x400
            assert not (self.get(port + 0x10) | self.get(port + 0x14)) & mask, "Selector HIGH"
        if port != PB:
            return
        mode = self.get(port) if mode is None else mode
        value = self.get(PB + 0x10)
        for pin, channel, af in ((14, 1, 5), (15, 2, 1)):
            if (mode >> (2 * pin)) & 3 != 2:
                continue
            assert self.get(PB + 0x20 + 4) >> (4 * (pin - 8)) & 15 == af, "Wrong motor AF"
            enabled = self.get(TIM + 0x20) & (1 << (4 * (channel - 1)))
            inverted = self.get(TIM + 0x20) & (2 << (4 * (channel - 1)))
            compare = self.get(TIM + 0x30 + 4 * channel)
            ccm = (self.get(TIM + 0x18) >> (8 * (channel - 1))) & 0xff
            assert enabled and inverted and ccm == 0x78, "Unmodeled PWM polarity/mode"
            # Inverted PWM2: HIGH below CCR; LOW at/above CCR, then HIGH at OPM CNT0.
            low = bool(self.get(TIM) & 1) and self.get(TIM + 0x24) >= compare
            value = value & ~(1 << pin) if low else value | (1 << pin)
        self.put(PB + 0x10, value)

    def _record_pads(self):
        self.levels(PB)
        pad = self.get(PB + 0x10) & 0xc000
        assert self.selectors_low(), "Selector HIGH is forbidden"
        if not self.timer_trace or self.timer_trace[-1][1] != pad:
            self.timer_trace.append((self.virtual_us, pad))

    def advance_us(self, elapsed):
        assert isinstance(elapsed, int) and elapsed >= 0
        old_time = self.virtual_us
        self.virtual_us += elapsed
        if self.get(RCC + 0x34) & 4 and self.get(TIM) & 1:
            assert self.get(TIM) & ~1 == 0x88 and not self.get(TIM + 8), "Unmodeled TIM4 mode"
            assert self.get(TIM + 0x28) == 479, "Unmodeled timer prescaler"
            if self.probe_scenario != "timer_stall":
                cycles = self.timer_fraction + elapsed * 48
                ticks, self.timer_fraction = divmod(cycles, 480)
                count, arr = self.get(TIM + 0x24), self.get(TIM + 0x2c)
                assert arr in (99, 1999) and count <= arr
                terminal = count + ticks > arr
                end = min(count + ticks, arr)
                flags = self.get(TIM + 0x10)
                for channel in range(1, 5):
                    compare = self.get(TIM + 0x30 + 4 * channel)
                    if count < compare <= end or terminal and compare == 0:
                        flags |= 1 << channel
                if terminal:
                    self.put(TIM + 0x24, 0)
                    self.put(TIM, self.get(TIM) & ~1)
                    self.put(TIM + 0x10, flags | 1)
                    expire = old_time + max(0, ((arr + 1 - count) * 480 -
                                               (cycles - elapsed * 48) + 47) // 48)
                    self.timer_runs[-1]["expired_at_us"] = expire
                else:
                    self.put(TIM + 0x24, count + ticks)
                    self.put(TIM + 0x10, flags)
                self._record_pads()
        if self.bgr_ready_at is not None and self.virtual_us >= self.bgr_ready_at:
            self.put(PMU + 0x14, 0x400)
            self.bgr_ready_at = None
        if self.cal_pending is not None and self.virtual_us >= self.cal_pending:
            self.put(ADC, 1)
            self.put(ADC + 0x10, self.get(ADC + 0x10) | 0x800)
            self.put(ADC + 0x30, 21)
            self.cal_pending = None
        if self.adc_pending is not None and self.virtual_us >= self.adc_pending[0]:
            _, channel, raw = self.adc_pending
            self.put(ADC, 1)
            self.put(ADC + 0x10, self.get(ADC + 0x10) | 12)
            self.put(ADC + 0x40, raw)
            self.adc_events.append((self.virtual_us, channel, raw))
            self.adc_pending = None

    def code(self, cpu, address, size, user):
        if address == self.symbols.get("motor_probe_begin"):
            self.begin_calls += 1
        if address == self.symbols.get("fatal"):
            self.in_fault = True
        super().code(cpu, address, size, user)

    def read(self, cpu, access, address, size, value, user):
        if TIM <= address < TIM + 0x60:
            assert self.get(RCC + 0x34) & 4, "Gated TIM4 read"
            return
        if PMU <= address < PMU + 0x20:
            assert self.get(RCC + 0x34) & 0x10000000, "Gated PMU read"
            return
        if ADC <= address < ADC + 0x80:
            assert self.get(RCC + 0x38) & 0x100000, "Gated ADC read"
            if address == ADC + 0x40:
                assert self.get(ADC) == 1 and self.get(ADC + 0x10) & 12 == 12
                self.adc_reads.append((self.virtual_us, self.get(ADC + 0x1c), self.get(address)))
                self.pending.append((ADC + 0x10, self.get(ADC + 0x10) & ~4))
            return
        super().read(cpu, access, address, size, value, user)
        if address == PB + 0x10 and self.probe_scenario == "begin_failure" and self.begin_calls and not self.begin_readback_failed:
            self.put(address, self.get(address) & ~0x4000)
            self.begin_readback_failed = True

    def write(self, cpu, access, address, size, value, user):
        if 0x20000000 <= address < 0x20010000:
            assert not (self.code_base <= address < self.code_base + len(self.data)), "Code self-write"
            if address == base.REPORT + 16 and value == 1:
                assert self.off(), "motor_disabled1 without complete off proof"
            if self.in_fault and base.REPORT <= address < base.REPORT + 128:
                assert self.off(), "Fault report before complete off"
            return
        if address in (RCC + 0x34, RCC + 0x38, RCC + 0x28):
            assert size == 4
            if address == RCC + 0x34:
                assert not value & ~(0x40000 | 4 | 0x10000000), "Unowned APB1 gates"
                if self.get(address) & 4 and not value & 4:
                    assert self.off(), "TIM4 gated before GPIOHH"
            elif address == RCC + 0x38:
                assert not value & ~0x100000, "Unowned APB2/debug gate"
            else:
                assert not value & ~0x100000, "Unowned peripheral reset"
                if value & 0x100000:
                    assert self.power_applied_at_us is not None and self.virtual_us - self.power_applied_at_us >= 101000, "ADC reset before HHL warmup"
                    self.adc_pending = self.cal_pending = None
                    for offset in range(0, 0x80, 4):
                        self.put(ADC + offset, 0)
            return
        if TIM <= address < TIM + 0x60:
            assert size == 4 and self.get(RCC + 0x34) & 4, "Gated TIM4 write"
            assert not self.get(TIM + 0x0c) and not self.get(RCC + 0x30) & 1, "Timer IRQ/DMA forbidden"
            if address in (TIM + 4, TIM + 8, TIM + 0x0c):
                assert value == 0, "Timer trigger/IRQ/DMA requests forbidden"
            if address == TIM:
                if self.get(TIM) & 1 and not value & 1:
                    assert self.off(), "Software timer stop before GPIOHH"
                if value & 1 and not self.get(TIM) & 1:
                    assert self.get(TIM + 0x24) == 0
                    self.timer_fraction = 0
                    self.timer_runs.append({"started_at_us": self.virtual_us,
                                            "arr": self.get(TIM + 0x2c)})
            if address == TIM + 0x14:
                assert value == 1
                self.pending.extend(((TIM + 0x24, 0),
                                     (TIM + 0x10, self.get(TIM + 0x10) | 1)))
            if address == TIM + 0x10:
                self.pending.append((address, self.get(address) & value))
            return
        if address == PMU:
            assert size == 4 and self.get(RCC + 0x34) & 0x10000000
            if value & 0x8000:
                self.bgr_ready_at = self.virtual_us + 100
            else:
                self.bgr_ready_at = None
                self.put(PMU + 0x14, 0)
            return
        if ADC <= address < ADC + 0x80:
            assert size == 4 and self.get(RCC + 0x38) & 0x100000
            assert self.power_applied_at_us is not None and self.virtual_us - self.power_applied_at_us >= 101000, "ADC access before HHL warmup"
            if address == ADC + 0x10:
                self.pending.append((address, self.get(address) & ~value))
            if address == ADC:
                if value == 1:
                    self.pending.append((address, 1))
                elif value == 0x80000000:
                    assert self.get(address) == 1
                    assert (self.get(ADC + 4), self.get(ADC + 8), self.get(ADC + 0x18), self.get(ADC + 0x0c)) == (0x4000, 0x80010101, 0x77, 0), "Unmodeled ADC configuration"
                    self.pending.append((address, 0x80000001))
                    self.cal_pending = self.virtual_us + 80
                elif value == 4:
                    assert self.off() and not self.get(RCC + 0x34) & 4, "ADC launch before off-settle/gate release"
                    assert self.get(RCC + 8) == 0x40012, "ADC clock is not modeled6MHz"
                    assert (self.get(ADC + 4), self.get(ADC + 8), self.get(ADC + 0x18), self.get(ADC + 0x0c)) == (0x4000, 0x80010101, 0x77, 0), "Unmodeled ADC configuration"
                    assert self.get(address) == 1 and not self.get(ADC + 0x10) & 0x83e
                    channel = self.get(ADC + 0x1c)
                    expected = (0x2000, 0x10000)[len(self.adc_launches) & 1]
                    assert channel == expected, "ADC channel sequence is not fresh BGR/signal pairs"
                    group, pair = divmod(len(self.adc_launches), 6)
                    raw = 1200 + group if channel == 0x2000 else 900 + group * 10 + pair // 2
                    self.adc_launches.append((self.virtual_us, channel, raw))
                    self.pending.append((address, 5))
                    if self.probe_scenario != "adc_timeout" or len(self.adc_launches) <= 6:
                        self.adc_pending = (self.virtual_us + 25, channel, raw)
                elif value in (2, 16):
                    self.adc_pending = None
                    self.pending.append((address, 0 if value == 2 else 1))
                else:
                    raise AssertionError(f"Unknown ADC command:{value:x}")
            return
        if 0x50000000 <= address < 0x50001000:
            if address & 0x3ff == 0x18:
                assert value <= 0xffff, "Unmodeled BSR upper-half reset write"
            self._power_write(address, value)
            if self.probe_scenario == "power_latch_failure" and address == PA + 0x18 and value == 0x20:
                return  # Ignore only PA5's latch effect; hardware readback stays LOW.
        if 0x50000000 <= address < 0x50001000 and address & 0x3ff == 0:
            assert size == 4 and self.get(RCC + 0x2c) & (1 << ((address - PA) // 0x400))
            port = address & ~0x3ff
            if port == PB:
                assert value & 0xf0000000 in (0x50000000, 0x60000000, 0x90000000)
            if port == PB and self.ignore_motor_mode:
                self.pending.append((address, self.get(address)))
            else:
                self.levels(port, value)
            return
        if address == 0xe000e100:
            assert value == 1 << 27, "Unmodeled peripheral IRQ enabled"
        super().write(cpu, access, address, size, value, user)

    def _power_write(self, address, value):
        """Permit only the fixed nine HHL writes; all other fields stay owned elsewhere."""
        port, offset = address & ~0x3ff, address & 0x3ff
        pin_mask = 0x20 if port == PA else 0x1001 if port == PB else 0
        field_mask = 0xc00 if port == PA else 0x03000003 if port == PB else 0
        mask = pin_mask if offset in (4, 0x14, 0x18, 0x28) else field_mask if offset in (0, 0x0c) else 0
        if offset in (0x18, 0x28):
            changed = value & mask
        else:
            changed = (value ^ self.get(address)) & mask
        steps = ((PA + 0x18, 0xffffffff, 0x20), (PB + 0x18, 0xffffffff, 1),
                 (PB + 0x28, 0xffffffff, 0x1000), (PA + 4, 0x20, 0),
                 (PB + 4, 0x1001, 0), (PA + 0x0c, 0xc00, 0x800),
                 (PB + 0x0c, 0x03000003, 0x01000002),
                 (PA, 0xc00, 0x400), (PB, 0x03000003, 0x01000001))
        step = len(self.power_writes)
        expected = steps[step] if step < len(steps) else None
        # Zero-change OTYPE writes are still part of the explicit ordered apply.
        applying = step in (3, 4) and address == expected[0]
        if not changed and not applying:
            return
        assert expected is not None and address == expected[0] and value & expected[1] == expected[2], "Unowned/reordered HHL power write"
        if offset not in (0x18, 0x28):
            assert (value ^ self.get(address)) & ~expected[1] == 0, "HHL changed unrelated fields"
        assert self.off() and self.selectors_low(), "HHL apply before motor off"
        if offset == 0:
            assert self.get(PA + 0x14) & 0x20 and self.get(PB + 0x14) & 0x1001 == 1, "HHL MODE before latch"
        self.power_writes.append((self.virtual_us, address, value))
        if len(self.power_writes) == 9:
            self.power_applied_at_us = self.virtual_us

    def milliseconds(self, count, service=True):
        for _ in range(count):
            for slot in range(8):
                self.advance_us(125)
                if slot == 7 and self.reason != "fatal":
                    self.handler(15)
                if service and self.reason != "fatal":
                    self.foreground(2)
                if self.reason == "fatal":
                    return

    def checks(self):
        assert bytes(self.cpu.mem_read(0x08000000, 0x20000)) == self.flash
        assert bytes(self.cpu.mem_read(0x20006c00, 32)) == self.canary
        assert self.selectors_low()
        assert not self.get(RCC + 0x30) & 1, "DMA never owned by this model"
        return {"virtual_us": self.virtual_us, "real_irq_dispatch": False,
                "timer_runs": self.timer_runs, "timer_pad_trace": self.timer_trace,
                "power_writes": self.power_writes,
                "adc_launches": self.adc_launches, "adc_reads": self.adc_reads,
                "instructions": self.instructions, "manual_ticks": self.ticks,
                "begin_calls": self.begin_calls,
                "tx_hex": self.tx.hex(), "app_report": self.report()[:12]}


def snapshot(model):
    """Fixed ARM ABI: one-byte enum makes feedback sample40B, not native44B."""
    address = model.symbols["probe_report"]
    assert address == 0x20006800
    data = bytes(model.cpu.mem_read(address, 312))
    names = ("version", "bytes", "state", "error", "cleanup_pending", "begin_used",
             "started_at_ms", "last_tick_ms", "finished_at_ms", "target_pulses", "samples",
             "completed_pulses", "nominal_low_ticks_100khz", "sequence_error",
             "pulse_error", "pulse_cleanup_error", "board_error", "board_owned",
             "sampler_error", "stop_confirmed", "cleanup_abort_expected")
    result = dict(zip(names, struct.unpack_from("<21I", data)))
    assert result["version"] == 1 and result["bytes"] == 312, result
    result["raw"] = []
    for i in range(3):
        at = 84 + i * 40
        pairs = [struct.unpack_from("<HHI", data, at + j * 8) for j in range(3)]
        oldest, completed, sequence = struct.unpack_from("<3I", data, at + 24)
        error, valid = struct.unpack_from("<BB", data, at + 36)
        result["raw"].append({"pair": pairs, "oldest": oldest, "completed": completed,
                              "sequence": sequence, "error": error, "valid": valid})
    result["pulse"] = [struct.unpack_from("<11I", data, 204 + i * 44) for i in range(2)]
    result["off_at"] = struct.unpack_from("<2I", data, 292)
    result["done_at"] = struct.unpack_from("<2I", data, 300)
    result["factory"] = struct.unpack_from("<I", data, 308)[0]
    return result


def run(data, symbols, scenario):
    m = ProbeCore(data, symbols, scenario)
    m.boot()
    if scenario == "power_latch_failure":
        assert m.reason == "fatal" and m.report()[2:5] == (0xE0, 9, 1)
        assert len(m.power_writes) == 7 and not m.begin_calls
        assert m.get(PA) & 0xc00 == 0xc00 and m.get(PB) & 0x03000003 == 0x03000003
        assert not m.adc_launches and not m.timer_runs
        return m.checks()
    assert len(m.power_writes) == 9 and m.power_applied_at_us == 0
    # Before the fixed250ms begin, leave foreground quiet; real ticks still run.
    m.milliseconds(200, service=False)
    m.foreground(2)
    if scenario == "radio":
        m.receive(base.encode(base.Frame(0x35, 2, bytes.fromhex("07050407d0"))))
    m.milliseconds(49, service=False)
    m.foreground(2)
    assert not m.adc_launches and not m.timer_runs, "Probe began before250ms"
    stalled = False
    for _ in range(150):
        m.milliseconds(1)
        if scenario == "foreground_stall" and not stalled and any(pad != 0xc000 for _, pad in m.timer_trace):
            stalled = True
            m.milliseconds(30, service=False)
            # No foreground polling/time writes were needed for natural OPM expiry.
            assert m.timer_runs[0]["expired_at_us"] == m.timer_runs[0]["started_at_us"] + 20000
            assert m.get(PB + 0x10) & 0xc000 == 0xc000
            m.foreground(4)
        if m.reason == "fatal":
            raise AssertionError((scenario, "unexpected app fatal", m.report()))
        report = snapshot(m)
        if report["state"] in (4, 5) and not report["cleanup_pending"]:
            break
    else:
        raise AssertionError((scenario, "No bounded clean result", snapshot(m)))
    assert m.off() and not m.get(RCC + 0x34) & 4
    assert m.get(RCC + 8) == 0x12 and m.get(RCC + 0x38) == 0
    assert m.get(RCC + 0x34) == 0x40000
    if scenario == "begin_failure":
        assert m.begin_calls == 1 and m.begin_readback_failed
        assert report["state"] == 5 and report["error"] and report["begin_used"] == 0
        assert not m.timer_runs and not m.adc_launches and not report["completed_pulses"]
        m.milliseconds(10)
        assert m.begin_calls == 1, "Early rejected begin was retried by application"
        return m.checks()
    assert m.begin_calls == 1 and report["begin_used"] == 1 and report["target_pulses"] == 2
    assert report["started_at_ms"] >= 250 and report["finished_at_ms"] - report["started_at_ms"] <= 100
    assert report["stop_confirmed"] == 1 and report["board_owned"] == 0
    assert report["nominal_low_ticks_100khz"] == 1900 * report["completed_pulses"]
    assert len(m.timer_runs) <= 4 and all(item["arr"] in (99, 1999) for item in m.timer_runs)
    if scenario in ("normal", "radio"):
        assert report["state"] == 4 and report["error"] == 0
        assert (report["samples"], report["completed_pulses"], report["nominal_low_ticks_100khz"]) == (3, 2, 3800)
        assert all(report[name] == 0 for name in ("sequence_error", "pulse_error", "pulse_cleanup_error", "board_error", "sampler_error"))
        assert [run["arr"] for run in m.timer_runs] == [1999, 99, 1999, 99]
        assert len(m.adc_reads) == len(m.adc_launches) == 18
        for i, raw in enumerate(report["raw"]):
            assert raw["sequence"] == i + 1 and raw["valid"] == 1 and raw["error"] == 0
            assert raw["oldest"] <= raw["completed"] and raw["completed"] - raw["oldest"] < 5
            assert [(bgr, signal) for bgr, signal, _ in raw["pair"]] == [(1200 + i, 900 + i * 10 + j) for j in range(3)]
            if i:
                assert raw["oldest"] >= report["done_at"][i - 1]
        for i, pulse in enumerate(report["pulse"]):
            assert pulse[:3] == (3, 0, 1) and pulse[4:6] == (2, 0)
            assert pulse[7] == 1 and 100 <= pulse[8] <= 1999 and pulse[10] == 0
            assert report["done_at"][i] >= report["off_at"][i] + 1
        for run in m.timer_runs:
            assert run["expired_at_us"] - run["started_at_us"] == (20000 if run["arr"] == 1999 else 1000)
    else:
        assert report["state"] == 5 and report["error"] != 0
        assert report["samples"] == 1
        assert report["completed_pulses"] == (1 if scenario == "adc_timeout" else 0)
        assert report["sampler_error"] == (7 if scenario == "adc_timeout" else 0)
        if scenario == "foreground_stall":
            assert stalled and report["sequence_error"] == 5
        if scenario == "timer_stall":
            assert report["sequence_error"] == 6 and not any(pad != 0xc000 for _, pad in m.timer_trace)
    if scenario == "radio":
        assert base.decode(bytes(m.tx)) == base.Frame(0x35, 3, bytes.fromhex("000705"))
    else:
        assert not m.tx
    before = len(m.timer_runs), len(m.adc_launches)
    m.milliseconds(10)
    assert before == (len(m.timer_runs), len(m.adc_launches)), "Finished attempt restarted"
    result = m.checks()
    result.update({"probe_report": report, "endpoint_validated": False,
                   "calibration_validated": False, "physical_motor_energy_measured": False})
    return result


def main():
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "build-probe"
    manifest = json.loads((out / "manifest.json").read_text())
    data = (out / "replacement-core.bin").read_bytes()
    assert hashlib.sha256(data).hexdigest() == manifest["bin_sha256"]
    cases = {scenario: run(data, manifest["symbols"], scenario) for scenario in
             ("normal", "radio", "foreground_stall", "adc_timeout", "timer_stall", "begin_failure", "power_latch_failure")}
    result = {"hardware_tested": False, "silicon_irq_dispatch": False,
              "bin_sha256": manifest["bin_sha256"], "cases": cases}
    (out / "thumb-results.json").write_text(json.dumps(result, indent=2) + "\n")
    print(f"PASS {len(cases)} actual-linked raw probe cases; virtual peripherals/manual IRQs, no physical/calibration claim")


if __name__ == "__main__":
    main()
