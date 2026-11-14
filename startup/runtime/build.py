"""Offline runtime tests and reset link fixture. No device or flash operations."""
import hashlib
import json
import os
from pathlib import Path
import re
import struct
import subprocess

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "build"
OUT.mkdir(exist_ok=True)
warnings = ["-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic",
            "-Wconversion", "-Wshadow"]
for name, extra in (("test-runtime", []), ("test-runtime-sanitize", [
        "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
        "-fno-sanitize-recover=all"])):
    subprocess.check_call(["gcc", *warnings, "-O1", "-g", *extra,
        str(ROOT / "w600_runtime.c"), str(ROOT / "test_runtime.c"),
        "-o", str(OUT / name)])
    subprocess.check_call([str(OUT / name)])

prefix = os.environ.get("ARM_TOOL_PREFIX", "arm-none-eabi-")
flags = [*warnings, "-mcpu=cortex-m0plus", "-mthumb", "-Os", "-g3",
         "-ffreestanding", "-fno-builtin", "-fno-common", "-fstack-usage"]
objects = []
for source in ("w600_reset.S", "w600_runtime.c", "link_fixture.c"):
    obj = OUT / (Path(source).stem + ".o")
    subprocess.check_call([prefix + "gcc", *flags, "-c", str(ROOT / source), "-o", str(obj)])
    objects.append(obj)
runtime = OUT / "w600_runtime.o"
assert not subprocess.check_output([prefix + "nm", "-u", str(runtime)], text=True).strip()
sections = subprocess.check_output([prefix + "size", "-A", str(runtime)], text=True)
for name in (".data", ".bss"):
    found = re.search(r"^" + re.escape(name) + r"\s+(\d+)\s", sections, re.MULTILINE)
    assert found and int(found[1]) == 0

elf = OUT / "reset-link-fixture.elf"
subprocess.check_call([prefix + "gcc", "-mcpu=cortex-m0plus", "-mthumb", "-nostdlib",
    "-Wl,--build-id=none", "-Wl,--fatal-warnings", "-Wl,-Map=" + str(OUT / "reset-link-fixture.map"),
    "-T", str(ROOT / "link_fixture.ld"), *map(str, objects), "-o", str(elf)])
assert not subprocess.check_output([prefix + "nm", "-u", str(elf)], text=True).strip()
symbols = {}
for line in subprocess.check_output([prefix + "nm", "-n", str(elf)], text=True).splitlines():
    fields = line.split()
    if len(fields) == 3:
        symbols[fields[2]] = int(fields[0], 16)
assert symbols["w600_vectors"] == 0x08002000
assert symbols["__stack_top"] == 0x20007000
assert symbols["__data_size"] == 7 and symbols["__bss_size"] == 11
assert symbols["__data_end"] <= symbols["__bss_start"]
assert symbols["__bss_end"] <= symbols["fixture_noinit"] < 0x20006c00
assert 0x08002000 <= symbols["__data_load"] < 0x08010800
vectors_path = OUT / "vectors.fixture"
subprocess.check_call([prefix + "objcopy", "--dump-section", ".vectors=" + str(vectors_path), str(elf)])
vectors = struct.unpack("<48I", vectors_path.read_bytes())
assert vectors[0] == 0x20007000 and vectors[1] == symbols["W600_Reset_Handler"] | 1
assert all(v in (0, symbols["W600_Default_Handler"] | 1) for v in vectors[2:])
vectors_path.unlink()

dis = subprocess.check_output([prefix + "objdump", "-d", str(elf)], text=True)
(OUT / "reset-link-fixture.dis").write_text(dis)
assert not re.search(r"\b(cpsie|bkpt|blx)\b", dis)
reset = dis.split("<W600_Reset_Handler>:", 1)[1].split("<W600_Default_Handler>:", 1)[0]
calls = re.findall(r"\bbl\s+[0-9a-f]+\s+<([^>]+)>", reset)
assert calls == ["w600_reset_early", "w600_runtime_apply", "w600_app_main"]
assert re.search(r"\bcpsid\s+i", reset)
assert re.search(r"\bmsr\s+CONTROL,\s*r0", reset, re.IGNORECASE)
assert re.search(r"\bmsr\s+MSP,\s*r0", reset, re.IGNORECASE)
assert "e000ed08" in reset and "dsb" in reset and "isb" in reset
assert re.search(r"\bbeq\S*\s+[0-9a-f]+\s+<W600_Default_Handler>", reset)
assert "sub\tsp, #8" in reset and "add\tsp, #8" in reset

inputs = ["w600_runtime.h", "w600_runtime.c", "w600_reset.S", "test_runtime.c",
          "link_fixture.c", "link_fixture.ld", "build.py"]
manifest = {
    "artifact_kind": "link_fixture_not_device_firmware",
    "hardware_tested": False, "flash_operations": False,
    "cold_reset_validated": False, "real_safe_hooks_provided": False,
    "host_memory_scenarios": 11, "strict_passed": True, "sanitizers_passed": True,
    "reset_call_order": calls, "symbols": symbols,
    "source_sha256": {p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest() for p in inputs},
    "elf_sha256": hashlib.sha256(elf.read_bytes()).hexdigest(),
}
(OUT / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
print(json.dumps(manifest, indent=2))
print("Runtime/reset link checks passed. Fixture has no real hardware safety hooks.")
