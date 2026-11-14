"""Actual linked display + raw-probe app; virtual peripherals, manual IRQ calls.

No device access, visible-LED claim, electrical model or calibration result.
Reuses the motor-probe bus; code spans derive from the linked vector address.
"""
import hashlib
import json
from pathlib import Path
import sys

import probe_thumb as probe

UART2 = 0x40004400
RCC, PA, PB = probe.RCC, probe.PA, probe.PB


class DisplayCore(probe.ProbeCore):
    def __init__(self, data, symbols, scenario="normal"):
        self.display_scenario = scenario
        self.display_wire = bytearray()
        self.display_writes = []
        self.display_due = None
        self.display_fault_injected = False
        super().__init__(data, symbols, "normal")

    def advance_us(self, elapsed):
        super().advance_us(elapsed)
        if self.display_due is not None and self.virtual_us >= self.display_due:
            flags = 0x80 if self.display_scenario == "missing_tc" else 0xc0
            self.put(UART2 + 0x1c, self.get(UART2 + 0x1c) | flags)
            self.display_due = None

    def execute(self, entry, count=2000000):
        # The admitted init's100000 bounded ACK polls execute real MMIO guards.
        if self.display_scenario == "missing_teack":
            count = max(count, 20000000)
        super().execute(entry, count)

    def read(self, cpu, access, address, size, value, user):
        if UART2 <= address < UART2 + 0x30:
            assert self.get(RCC + 0x34) & 0x20000, "Gated display UART read"
            return
        super().read(cpu, access, address, size, value, user)

    def write(self, cpu, access, address, size, value, user):
        if address == RCC + 0x34:
            assert size == 4 and not value & ~(0x60000 | 4 | 0x10000000)
            if self.get(address) & 4 and not value & 4:
                assert self.off(), "Timer gate removed before motor-off proof"
            if self.get(address) & 0x20000 and not value & 0x20000:
                assert self.off() and not self.get(UART2), "Display gated before off/disable"
            return
        if address == RCC + 0x24:
            assert size == 4 and not value & ~0x60000
            if value & 0x20000:
                for offset in range(0, 0x2c, 4):
                    self.put(UART2 + offset, 0)
                self.put(UART2 + 0x1c, 0xc0)
            # The frozen model also applies the UART3 reset side effects.
            super().write(cpu, access, address, size, value, user)
            return
        if UART2 <= address < UART2 + 0x30:
            assert size == 4 and self.get(RCC + 0x34) & 0x20000
            assert not self.get(0xe000e100) & (1 << 26), "Display IRQ enabled"
            assert not self.get(0xe000e200) & (1 << 26), "Display IRQ pending"
            self.display_writes.append((self.virtual_us, address, value))
            if address == UART2:
                assert value in (0, 0x1608, 0x1609)
                old = self.get(UART2)
                # Format fields are write-protected while UE was enabled.
                effective = ((value & ~0x1600) | (old & 0x1600)) if old & 1 else value
                self.pending.append((UART2, effective))
                status = 0xc0 | (0x200000 if value else 0)
                if not effective & 1:
                    status &= ~0x200000
                if value and self.display_scenario == "missing_teack":
                    status &= ~0x200000
                self.pending.append((UART2 + 0x1c, status))
            elif address == UART2 + 0x20:
                assert value in (15, 0x40)
                self.pending.append((UART2 + 0x1c, self.get(UART2 + 0x1c) & ~value))
            elif address == UART2 + 0x28:
                assert value <= 255 and self.get(UART2) == 0x1609
                assert self.get(UART2 + 0x1c) & 0x80
                assert self.get(PB) & 0x3000 == 0x2000
                assert self.get(PB + 4) & 0x40
                assert self.get(PB + 0x0c) & 0x3000 == 0x2000
                assert self.get(PB + 0x20) & 0x0f000000 == 0x01000000
                assert not self.get(PA + 0x10) & 0x10, "Display not awake"
                self.display_wire.append(value)
                self.put(UART2 + 0x1c, self.get(UART2 + 0x1c) & ~0xc0)
                self.display_due = self.virtual_us + 600
            elif address == UART2 + 0x0c:
                assert value == 2500
            else:
                assert address in (UART2 + 4, UART2 + 8) and value == 0
            return
        super().write(cpu, access, address, size, value, user)
        if (self.display_scenario == "adc_context_loss" and address == probe.ADC and
                value == 4 and not self.display_fault_injected):
            assert self.get(RCC + 0x34) & 0x10000000
            assert self.get(RCC + 0x38) & 0x100000
            self.put(UART2 + 0x0c, 2499)
            self.display_fault_injected = True

    def display_report(self):
        return self.report()[12:19]

    def checks(self):
        result = super().checks()
        result.update({"display_wire_hex": self.display_wire.hex(),
                       "display_report": self.display_report(),
                       "display_mmio_writes": self.display_writes,
                       "visible_display_validated": False})
        return result


def run(data, symbols, scenario):
    m = DisplayCore(data, symbols, scenario)
    if scenario == "dirty_fault":
        m.cpu.reg_write(probe.base.UC_ARM_REG_XPSR, 0x01000003)
        m.execute(symbols["W600_Default_Handler"])
    elif scenario == "bad_clock":
        m.put(RCC + 0x0c, 0)
        m.boot()
    else:
        m.boot()
    if scenario in ("dirty_fault", "bad_clock"):
        assert m.reason == "fatal" and m.report()[2:5] == (0xe0, 1, 1)
        assert m.report()[5:] == (0,) * 27
        assert not m.runtime_zero and not m.display_wire
        assert not m.timer_runs and not m.adc_launches
        return m.checks()
    if scenario == "missing_teack":
        assert m.reason == "fatal" and m.report()[2:5] == (0xe0, 11, 1)
        assert not m.display_wire and not m.timer_runs and not m.adc_launches
        assert not m.get(RCC + 0x34) & 0x20000
        return m.checks()
    assert m.reason != "fatal" and m.get(RCC + 0x34) == 0x60000
    m.milliseconds(200)
    if scenario == "missing_tc":
        assert m.display_report()[1:3] == (4, 6)
        assert m.display_report()[6] == 1 and m.off()
        assert not m.get(RCC + 0x34) & 0x20000
        # A display failure leaves the radio alive, not a frozen application.
        m.receive(probe.base.encode(probe.base.Frame(0x41, 2, bytes.fromhex("07050407d0"))))
        m.milliseconds(70)
        assert probe.base.decode(bytes(m.tx)) == probe.base.Frame(0x41, 3, bytes.fromhex("000705"))
        assert m.report()[2:4] == (2, 0) and not m.timer_runs
        return m.checks()
    assert bytes(m.display_wire) == bytes.fromhex("080000000018ff")
    assert m.display_report()[3:5] == (1, 1)
    m.milliseconds(57)
    # A real UART3 request updates the requested target while the raw test runs.
    m.receive(probe.base.encode(probe.base.Frame(0x42, 2, bytes.fromhex("07000407d0"))))
    m.foreground(2)
    injected = False
    for _ in range(100):
        m.milliseconds(1)
        if scenario in ("context_loss", "cleanup_pending") and not injected and any(pad != 0xc000 for _, pad in m.timer_trace):
            if scenario == "cleanup_pending":
                m.ignore_motor_mode = True
            m.put(UART2 + 0x0c, 2499)
            injected = True
            m.foreground(2)
        report = probe.snapshot(m)
        if scenario == "cleanup_pending" and injected and report["state"] == 5:
            assert report["cleanup_pending"] and report["board_owned"]
            assert m.display_report()[1:3] == (1, 6) and not m.display_report()[6]
            assert m.get(RCC + 0x34) & 0x20000, "Display gate removed during failed motor cleanup"
            assert m.get(UART2) == 0x1609 and not m.get(RCC + 0x24) & 0x20000
            assert not m.get(PA + 0x14) & 0x10, "Display stopped before motor cleanup was proved"
            assert not report["stop_confirmed"]
            m.milliseconds(70)
            assert m.get(RCC + 0x34) & 0x20000
            assert probe.snapshot(m)["cleanup_pending"] and m.report()[2:4] == (2, 0)
            assert probe.base.decode(bytes(m.tx)) == probe.base.Frame(0x42, 3, bytes.fromhex("000700"))
            result = m.checks()
            result["probe_report"] = probe.snapshot(m)
            result["stop_or_cleanup_validated"] = False
            return result
        if report["state"] in (4, 5) and not report["cleanup_pending"]:
            break
    else:
        raise AssertionError((scenario, "Raw test failed to finish boundedly"))
    m.milliseconds(70)
    assert m.off() and report["stop_confirmed"] and not report["board_owned"]
    assert not m.get(RCC + 0x34) & 4 and not m.get(RCC + 0x38)
    assert m.get(RCC + 8) == 0x12
    assert probe.base.decode(bytes(m.tx)) == probe.base.Frame(0x42, 3, bytes.fromhex("000700"))
    if scenario in ("context_loss", "adc_context_loss"):
        assert (injected or m.display_fault_injected) and report["state"] == 5 and report["error"]
        if scenario == "adc_context_loss":
            assert not m.timer_runs and not report["completed_pulses"]
        assert m.display_report()[1:3] == (1, 6) and m.display_report()[6] == 1
        assert m.get(RCC + 0x34) == 0x40000
    else:
        assert report["state"] == 4 and not report["error"]
        assert (report["samples"], report["completed_pulses"], report["nominal_low_ticks_100khz"]) == (3, 2, 3800)
        assert m.get(RCC + 0x34) == 0x60000
        assert bytes(m.display_wire) == bytes.fromhex("080000000018ff08377b001018ff")
        assert m.display_report()[1:5] == (0, 0, 2, 2)
        # Repeating the same value does not flood the display.
        m.receive(probe.base.encode(probe.base.Frame(0x43, 2, bytes.fromhex("07000407d0"))))
        m.milliseconds(70)
        assert m.display_report()[3:5] == (2, 2)
    before = len(m.timer_runs), len(m.adc_launches), len(m.display_wire)
    m.milliseconds(70)
    assert before == (len(m.timer_runs), len(m.adc_launches), len(m.display_wire))
    result = m.checks()
    result["probe_report"] = report
    return result


def main():
    out = Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).parent/"build-display"
    manifest = json.loads((out/"manifest.json").read_text())
    data = (out/"replacement-core.bin").read_bytes()
    assert hashlib.sha256(data).hexdigest() == manifest["bin_sha256"]
    assert manifest["display_probe_bench"]
    results = {scenario:run(data, manifest["symbols"],scenario) for scenario in
               ("normal", "missing_teack", "missing_tc", "context_loss",
                "adc_context_loss", "cleanup_pending", "dirty_fault", "bad_clock")}
    (out/"thumb-results.json").write_text(json.dumps({"hardware_tested":False,
        "visible_leds_or_rf_validated":False,"cases":results},indent=2)+"\n")
    print(f"PASS {len(results)} linked display/raw/radio scenarios; no physical/LED/calibration claim")


if __name__ == "__main__":
    main()
