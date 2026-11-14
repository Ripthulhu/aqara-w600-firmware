"""Actual linked application core with a modeled bus and real handlers.

No device access. IRQ entry/latency, oscillator timing and RF are NOT modeled.
The test never substitutes application/radio functions or writes app_ticks.
"""
import hashlib
import json
from pathlib import Path
import struct
import sys
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_MCLASS, UC_MODE_THUMB
from unicorn import UC_HOOK_CODE, UC_HOOK_MEM_READ, UC_HOOK_MEM_WRITE
from unicorn.arm_const import (UC_ARM_REG_SP, UC_ARM_REG_MSP, UC_ARM_REG_LR,
                              UC_ARM_REG_PC, UC_ARM_REG_XPSR, UC_ARM_REG_PRIMASK)

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent))
from radio_frames import Frame, encode, decode

SENTINEL = 0x20008000
REPORT = 0x20006a00
UART = 0x40004800


class Core:
    def __init__(self, data, symbols, scenario="quiet"):
        self.data, self.symbols, self.scenario = data, symbols, scenario
        self.code_base = symbols["w600_vectors"]
        assert self.code_base in (0x08002000, 0x20000c00, 0x20001000, 0x20002000)
        self.cpu = Uc(UC_ARCH_ARM, UC_MODE_MCLASS | UC_MODE_THUMB)
        for base, size in ((0x20000000, 0x10000), (0x08000000, 0x20000),
                           (0x40000000, 0x30000), (0x50000000, 0x1000),
                           (0x1fff0000, 0x1000), (0xe000e000, 0x2000)):
            self.cpu.mem_map(base, size)
        self.cpu.mem_write(0x20000000, b"\xa5" * 0x7000)
        self.flash = bytearray((i * 3 + 7) & 255 for i in range(0x20000))
        if self.code_base == 0x08002000:
            assert len(data) <= 0xce00
            self.flash[0x2000:0x2000+len(data)] = data
        else:
            self.cpu.mem_write(self.code_base, data)
        self.flash = bytes(self.flash)
        self.cpu.mem_write(0x08000000, self.flash)
        self.canary = b"\xcc" * 32
        self.cpu.mem_write(0x20006c00, self.canary)
        self.pending, self.rx, self.tx = [], [], bytearray()
        self.pressed = scenario == "held_boot"
        self.uart_error = 0
        self.tc = scenario != "tc_failure"
        self.skip_boundary = False
        self.in_handler = False
        self.reason = None
        self.fatal_steps = 0
        self.runtime_zero = False
        self.report_writes_after_off = True
        self.lowest_sp = 0x20007000
        self.instructions = self.ticks = self.uart_irqs = 0
        self.flash_writes = 0
        self.memmove_calls = 0
        self.af_low_mask = 0
        self.ignore_motor_mode = False
        self.vectors = struct.unpack_from("<48I", data)
        assert self.vectors[0] == 0x20007000
        assert self.vectors[1] == symbols["W600_Reset_Handler"] | 1
        assert self.vectors[15] == symbols["W600_SysTick_Handler"] | 1
        assert self.vectors[43] == symbols["W600_UART3_4_Handler"] | 1
        for address, value in {
            0xe000ed00: 0x410cc601, 0x1fff03a4: 2, 0x1fff03a8: 128,
            0x1fff03ac: 28, 0x1fff03cc: 0xffb70048,
            0x40021000: 0x00100500, 0x4002100c: 0x20260,
            0x40021030: 0x100, 0x40021050: 9, 0x40021054: 72,
            0x40022014: 0xc0000000,
        }.items():
            self.put(address, value)
        if scenario == "bad_clock": self.put(0x4002100c, 0)
        if scenario == "bad_trim": self.put(0x1fff03cc, 0x12340048)
        if scenario == "bad_watchdog": self.put(0x40003004, 7)
        if scenario == "bad_iopen": self.put(0x4002102c, 1)
        self.cpu.hook_add(UC_HOOK_CODE, self.code)
        self.cpu.hook_add(UC_HOOK_MEM_READ, self.read)
        self.cpu.hook_add(UC_HOOK_MEM_WRITE, self.write)
        self.cpu.reg_write(UC_ARM_REG_MSP, 0x20007000)
        self.cpu.reg_write(UC_ARM_REG_XPSR, 0x01000000)

    def put(self, address, value):
        self.cpu.mem_write(address, struct.pack("<I", value & 0xffffffff))

    def get(self, address):
        return struct.unpack("<I", self.cpu.mem_read(address, 4))[0]

    def report(self):
        return struct.unpack("<32I", self.cpu.mem_read(REPORT, 128))

    def off(self):
        return ((self.get(0x4002102c) & 7) == 7 and
                self.get(0x50000400) & 0xf0000000 == 0x50000000 and
                self.get(0x50000414) & 0xc000 == 0xc000 and
                self.get(0x50000410) & 0xc000 == 0xc000 and
                self.get(0x50000000) & 0xc0000000 == 0x40000000 and
                self.get(0x50000800) & 0x300000 == 0x100000 and
                not self.get(0x50000404) & 0xc000 and
                not self.get(0x50000004) & 0x8000 and
                not self.get(0x50000804) & 0x400 and
                not self.get(0x5000040c) & 0xf0000000 and
                not self.get(0x5000000c) & 0xc0000000 and
                not self.get(0x5000080c) & 0x300000 and
                not (self.get(0x50000014) | self.get(0x50000010)) & 0x8000 and
                not (self.get(0x50000814) | self.get(0x50000810)) & 0x400)

    def levels(self, base, mode=None):
        mode = self.get(base) if mode is None else mode
        value, odr, pull = self.get(base + 0x10), self.get(base + 0x14), self.get(base + 0x0c)
        for bit in range(16):
            kind = (mode >> (2 * bit)) & 3
            if kind == 1:
                value = (value & ~(1 << bit)) | (odr & (1 << bit))
            elif kind == 0 and (pull >> (2 * bit)) & 3 == 1:
                value |= 1 << bit
            elif kind == 2 and base == 0x50000400 and self.af_low_mask & (1 << bit):
                value &= ~(1 << bit)
        if base == 0x50000400:
            value = (value & ~0x100) if self.pressed else (value | 0x100)
        self.put(base + 0x10, value)

    def code(self, cpu, address, size, user):
        self.instructions += 1
        while self.pending:
            a, value = self.pending.pop(0)
            self.put(a, value)
        sp = cpu.reg_read(UC_ARM_REG_SP)
        self.lowest_sp = min(self.lowest_sp, sp)
        assert 0x20006c20 <= sp <= 0x20007000, f"stack outside reserve:{sp:x}"
        if address == SENTINEL:
            assert self.in_handler
            self.reason = "handler_return"; cpu.emu_stop(); return
        assert self.code_base <= address < self.code_base + len(self.data), f"outside linked code:{address:x}"
        if address == self.symbols["memmove"]: self.memmove_calls += 1
        if address == self.symbols["w600_app_main"]:
            start, size = self.symbols["__bss_start"], self.symbols["__bss_size"]
            assert not any(cpu.mem_read(start, size)), "Reset did not clear dirty BSS"
            n = self.symbols["__data_size"]
            assert bytes(cpu.mem_read(self.symbols["__data_start"], n)) == bytes(
                cpu.mem_read(self.symbols["__data_load"], n))
            self.runtime_zero = True
        if self.report()[2] == 0xe0:
            self.fatal_steps += 1
            if self.fatal_steps >= 24:
                self.reason = "fatal"; cpu.emu_stop(); return
        if not self.in_handler and address == self.symbols["w600_radio_poll"]:
            if self.skip_boundary:
                self.skip_boundary = False
            else:
                self.reason = "foreground_boundary"; cpu.emu_stop()

    def read(self, cpu, access, address, size, value, user):
        assert not (0x40012400 <= address < 0x40012500), "unexpected ADC read"
        assert not (0x40007000 <= address < 0x40007020), "unexpected PMU read"
        assert not (0x40002c00 <= address < 0x40002c10), "gated WWDG read"
        assert not (0x40015800 <= address < 0x40015810 or address == 0xe000edf0), "target debug read"
        if 0x50000000 <= address < 0x50001000:
            port = (address - 0x50000000) // 0x400
            assert self.get(0x4002102c) & (1 << port), "gated GPIO read"
            if address & 0x3ff == 0x10: self.levels(address & ~0x3ff)
        if UART <= address < UART + 0x30:
            assert self.get(0x40021034) & 0x40000, "gated UART read"
        if 0x40022000 <= address < 0x40022048:
            assert self.get(0x40021030) & 0x100, "gated FLASH read"
        if address == UART + 0x1c:
            status = self.get(address) & ~0x2f
            status |= self.uart_error | (0x20 if self.rx else 0)
            if not self.tc: status &= ~0x40
            self.put(address, status)
        if address == UART + 0x24:
            assert self.rx, "RDR consumed without RX byte"
            self.put(address, self.rx.pop(0))
            self.pending.append((UART + 0x1c, self.get(UART + 0x1c) & ~0x20))

    def write(self, cpu, access, address, size, value, user):
        if 0x20000000 <= address < 0x20010000:
            if REPORT <= address < REPORT + 128:
                if self.scenario != "tick_stop_failed" or (address == REPORT+16 and value == 1):
                    assert self.off(), "report published before complete motor inhibit"
            assert not (self.code_base <= address < self.code_base + len(self.data)), "code self-write"
            return
        assert not (0x08000000 <= address < 0x08020000), "FLASH array write"
        assert not (0x40003000 <= address < 0x40003020), "IWDG write"
        assert not (0x40015800 <= address < 0x40015810), "MCU debug/freeze write"
        assert not (0xe000edf0 <= address < 0xe000ee00), "target debug write"
        assert not (0x40012400 <= address < 0x40012500), "ADC write"
        assert not (0x40000800 <= address < 0x40000850), "motor timer write"
        if 0xe000e400 <= address < 0xe000e420:
            assert size in (1, 4); return
        assert size == 4, (hex(address), size)
        if 0x40022000 <= address < 0x40022048:
            assert address == 0x40022000 and value == 2, "FLASH controller write"
            self.flash_writes += 1; return
        if address == 0x40021000:
            ready = (0x400 if value & 0x100 else 0) | (0x2000000 if value & 0x1000000 else 0)
            self.pending.append((address, (value & ~0x2000400) | ready)); return
        if address == 0x40021008:
            self.pending.append((address, (value & ~0x38) | ((value & 3) << 3))); return
        if address == 0x40021024:
            if value & 0x40000:
                for off in range(0, 0x2c, 4): self.put(UART + off, 0)
                self.put(UART + 0x1c, 0xc0)
            return
        if address in (0x4002100c, 0x4002102c, 0x40021030, 0x40021034, 0x40021038):
            if address == 0x40021034: assert not value & ~0x40000, "unowned APB1 gate"
            if address == 0x40021038: assert value == 0, "APB2/debug clock enabled"
            return
        if 0x50000000 <= address < 0x50001000:
            base, off = address & ~0x3ff, address & 0x3ff
            assert self.get(0x4002102c) & (1 << ((base - 0x50000000) // 0x400))
            if off in (0x18, 0x28):
                if base == 0x50000400:
                    assert not (off == 0x28 and value & 0xc000), "motor drive LOW"
                    assert not (off == 0x18 and value & 0xc0000000), "motor BSR reset"
                if base == 0x50000000: assert not (off == 0x18 and value & 0x8000), "PA15 HIGH"
                if base == 0x50000800: assert not (off == 0x18 and value & 0x400), "PC10 HIGH"
                self.put(base + 0x14, self.get(base + 0x14) | value if off == 0x18 else self.get(base + 0x14) & ~value)
                self.levels(base)
            elif off == 0:
                if base == 0x50000400: assert value & 0xf0000000 == 0x50000000
                if base == 0x50000400 and self.ignore_motor_mode:
                    self.pending.append((address, self.get(address)))
                else:
                    self.levels(base, value)
            elif off == 0x0c:
                self.pending.append((base + 0x10, self.get(base + 0x10)))
            elif off == 0x14:
                raise AssertionError("Unexpected direct motor/selector ODR write")
            return
        if UART <= address < UART + 0x30:
            assert self.get(0x40021034) & 0x40000
            if address == UART:
                status = 0x6000c0 if value & 1 else 0xc0
                if not self.tc: status &= ~0x40
                self.pending.append((UART + 0x1c, status))
            if address == UART + 0x20:
                self.uart_error &= ~(value & 15)
                self.pending.append((UART + 0x1c, self.get(UART + 0x1c) & ~value))
            if address == UART + 0x28:
                self.tx.append(value & 255)
                self.pending.append((UART + 0x1c, self.get(UART + 0x1c) | (0xc0 if self.tc else 0x80)))
            return
        if address == 0xe000e100: return
        if address == 0xe000e280:
            self.pending.append((0xe000e200, self.get(0xe000e200) & ~value)); return
        if address in (0xe000ed08, 0xe000ed20, 0xe000e010, 0xe000e014, 0xe000e018): return
        raise AssertionError(f"unmodeled MMIO write:{address:x}={value:x}")

    def execute(self, entry, count=2000000):
        self.reason = None
        self.cpu.emu_start(entry | 1, SENTINEL + 2, count=count)
        assert self.reason, "instruction budget exhausted"

    def boot(self):
        entry = self.symbols["W600_Reset_Handler"]
        if self.scenario == "fault_dirty":
            entry = self.symbols["W600_Default_Handler"]
            self.cpu.reg_write(UC_ARM_REG_XPSR, 0x01000003)
        self.execute(entry)
        if self.reason == "foreground_boundary": self.foreground()

    def foreground(self, slices=2):
        for _ in range(slices):
            if self.reason == "fatal": return
            self.skip_boundary = True
            self.execute(self.cpu.reg_read(UC_ARM_REG_PC))
            assert self.reason in ("foreground_boundary", "fatal")

    def handler(self, vector):
        assert self.reason == "foreground_boundary" and not self.in_handler
        assert self.cpu.reg_read(UC_ARM_REG_PRIMASK) == 0
        if vector == 15: assert self.get(0xe000e010) & 7 == 7
        else: assert vector == 43 and self.get(0xe000e100) & (1 << 27)
        saved = self.cpu.context_save()
        sp = self.cpu.reg_read(UC_ARM_REG_SP) - 32
        self.cpu.reg_write(UC_ARM_REG_MSP, sp)
        self.cpu.reg_write(UC_ARM_REG_XPSR, 0x01000000 | vector)
        self.cpu.reg_write(UC_ARM_REG_LR, SENTINEL | 1)
        self.in_handler = True
        self.execute(self.vectors[vector] & ~1, 100000)
        assert self.reason in ("handler_return", "fatal")
        if self.reason == "handler_return":
            self.cpu.context_restore(saved)
            self.reason = "foreground_boundary"
        self.in_handler = False
        if vector == 15: self.ticks += 1
        else: self.uart_irqs += 1

    def milliseconds(self, count, service=True):
        for _ in range(count):
            if self.reason == "fatal": break
            self.handler(15)
            if service: self.foreground()

    def receive(self, wire, service=False):
        self.rx.extend(wire)
        while self.rx:
            self.handler(43)
            if service: self.foreground()

    def check(self):
        assert self.off() == (self.scenario != "tick_stop_failed")
        assert bytes(self.cpu.mem_read(0x20006c00, 32)) == self.canary
        assert bytes(self.cpu.mem_read(0x08000000, 0x20000)) == self.flash
        assert self.report()[:2] == (0x57364150, 1)
        if self.reason == "fatal":
            assert not self.get(UART) & 0x1fff and not self.get(0xe000e010)
            assert self.report()[2] == 0xe0
            assert self.report()[4] == (self.scenario != "tick_stop_failed")
        return {"report": self.report()[:12], "instructions": self.instructions,
                "manual_systick_calls": self.ticks, "manual_uart_calls": self.uart_irqs,
                "tx_hex": self.tx.hex(), "runtime_bss_zero_observed": self.runtime_zero,
                "lowest_observed_sp": hex(self.lowest_sp), "flash_acr_writes": self.flash_writes,
                "linked_memmove_calls": self.memmove_calls}


def run(data, symbols, scenario):
    m = Core(data, symbols, scenario)
    m.boot()
    if scenario in ("bad_clock", "bad_watchdog", "bad_iopen", "fault_dirty", "bad_trim"):
        assert m.reason == "fatal" and not m.tx
        if scenario in ("bad_clock", "bad_watchdog", "bad_iopen", "fault_dirty"):
            assert not m.runtime_zero and m.report()[3] == 1
        else: assert m.runtime_zero and m.report()[3] == 2
        assert m.report()[5:] == (0,) * 27
    elif scenario == "quiet":
        m.milliseconds(20)
        assert not m.tx and m.report()[2:5] == (2, 0, 1) and m.runtime_zero
    elif scenario in ("tick_af_stop", "tick_stop_failed", "fault_af_stop"):
        # A low AF pad ignores HIGH GPIO latch writes until it is remuxed.
        m.af_low_mask = 0x8000
        m.put(0x50000400, (m.get(0x50000400) & ~0xc0000000) | 0x80000000)
        m.levels(0x50000400)
        assert not m.off() and not m.get(0x50000410) & 0x8000
        m.ignore_motor_mode = scenario == "tick_stop_failed"
        if scenario == "fault_af_stop":
            m.cpu.reg_write(UC_ARM_REG_XPSR, 0x01000003)
            m.execute(symbols["W600_Default_Handler"])
            assert m.reason == "fatal" and m.report()[3:5] == (1, 1)
        else:
            m.milliseconds(1)
            if scenario == "tick_stop_failed":
                assert m.reason == "fatal" and m.report()[3:5] == (8, 0)
                assert m.get(0x50000400) & 0xc0000000 == 0x80000000
            else:
                assert m.report()[2:6] == (2, 0, 1, 1)
                assert m.cpu.reg_read(UC_ARM_REG_PRIMASK) == 0
    elif scenario == "held_boot":
        m.milliseconds(10050, service=False); m.foreground()
        assert not m.tx and m.report()[6:8] == (0, 0)
    elif scenario in ("external_and_safe", "tc_failure"):
        m.receive(encode(Frame(0x35, 2, bytes.fromhex("07050407d0"))))
        m.milliseconds(70)
        if scenario == "tc_failure":
            assert m.reason == "fatal" and m.report()[3] == 7 and m.report()[8] == 5
            assert m.report()[9] == 0
        else:
            frame = decode(bytes(m.tx)); assert frame == Frame(0x35, 3, bytes.fromhex("000705"))
            old = len(m.tx)
            m.receive(encode(Frame(0x36, 2, bytes.fromhex("071d010207090001"))))
            m.milliseconds(120)
            assert decode(bytes(m.tx[old:])) == Frame(0x36, 3, bytes.fromhex("03071d030709"))
            assert m.report()[9] == 2 and not m.report()[10]
    elif scenario == "uart_error":
        m.uart_error = 8; m.handler(43); m.foreground()
        assert m.reason == "fatal" and m.report()[8] == 2 and not m.tx
    elif scenario == "ring_overflow":
        m.receive(bytes(513)); m.foreground()
        assert m.reason == "fatal" and m.report()[8] == 3 and not m.tx
    elif scenario == "codec_resync":
        good = encode(Frame(0x38, 2, bytes.fromhex("07050407d0")))
        bad = bytearray(encode(Frame(0x37, 2, bytes.fromhex("07050407d1"))))
        bad[-1] ^= 1
        m.receive(b"\x99\x55\x00\x55\x05\x00" + bytes(bad) + good)
        m.milliseconds(70)
        assert decode(bytes(m.tx)) == Frame(0x38, 3, bytes.fromhex("000705"))
        assert m.report()[9] == 1 and not m.report()[8] and m.memmove_calls
    elif scenario == "pair_gesture":
        m.milliseconds(30); m.pressed = True; m.foreground()
        m.milliseconds(30); m.milliseconds(3000, service=False); m.foreground()
        m.milliseconds(70)  # Drain the real three-second lock report before pairing.
        m.milliseconds(6929, service=False); m.foreground()
        assert not m.report()[6] and not m.report()[10]
        m.milliseconds(1); m.milliseconds(70)
        # The same hold also reports child-lock changes before pairing.
        offset=0;replies=[]
        while offset<len(m.tx):
            size=10+int.from_bytes(m.tx[offset+5:offset+7],'big')
            replies.append(decode(bytes(m.tx[offset:offset+size])));offset+=size
        assert len([f for f in replies if f.kind==5 and f.payload==bytes.fromhex("00510001")])==1
        assert m.report()[6:8] == (1, 1) and m.report()[10] == 1
    else: raise AssertionError(scenario)
    return m.check()


def main():
    out = Path(sys.argv[1]) if len(sys.argv)>1 else ROOT / "build-stop"
    manifest = json.loads((out / "manifest.json").read_text())
    data = (out / "replacement-core.bin").read_bytes()
    assert hashlib.sha256(data).hexdigest() == manifest["bin_sha256"]
    scenarios = ("quiet", "external_and_safe", "held_boot", "bad_clock", "bad_trim",
                 "bad_watchdog", "bad_iopen", "fault_dirty", "uart_error",
                 "ring_overflow", "codec_resync", "tc_failure", "pair_gesture",
                 "tick_af_stop", "tick_stop_failed", "fault_af_stop")
    results = {name: run(data, manifest["symbols"], name) for name in scenarios}
    output = {"hardware_tested": False, "silicon_irq_dispatch_or_latency": False,
              "rf_or_fresh_pairing_validated": False, "bin_sha256": manifest["bin_sha256"],
              "cases": results}
    (out / "thumb-results.json").write_text(json.dumps(output, indent=2) + "\n")
    print(f"PASS {len(results)} linked replacement-core Thumb cases; bus/time mocked, real handlers manually called, no RF/hardware claim")


if __name__ == "__main__": main()
