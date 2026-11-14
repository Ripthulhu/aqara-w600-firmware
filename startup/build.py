"""Offline startup checks only; never opens a device connection."""
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parent
SDK = Path(os.environ.get("CIU32_SDK", "/srv/ai-agent/work/aqara-w600-hardware/sdk-build/vendor-sdk/CIU32L071 EclipseExampleProject"))
OUT = ROOT / "build"
OUT.mkdir(exist_ok=True)
includes = ["-isystem", str(SDK / "Drivers/CMSIS/Core"),
            "-isystem", str(SDK / "Drivers/CMSIS/Device")]
warnings = ["-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic"]
for source in ("startup", "board"):
    for suffix, extra in (("", []), ("-sanitize", [
            "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
            "-fno-sanitize-recover=all"])):
        name = "test-" + source + suffix
        subprocess.check_call(["gcc", *warnings, "-O1", "-g", *extra, *includes,
                               str(ROOT / ("test_" + source + ".c")), "-o", str(OUT / name)])
        subprocess.check_call([str(OUT / name)])
prefix = os.environ.get("ARM_TOOL_PREFIX", "arm-none-eabi-")
obj = OUT / "ciu32_startup.o"
subprocess.check_call([prefix + "gcc", *warnings, "-Os", "-g3", "-mcpu=cortex-m0plus",
    "-mthumb", "-ffreestanding", "-fno-builtin", "-fno-common", "-fstack-usage", *includes,
    "-c", str(ROOT / "ciu32_startup.c"), "-o", str(obj)])
assert not subprocess.check_output([prefix + "nm", "-u", str(obj)], text=True).strip()
sections = subprocess.check_output([prefix + "size", "-A", str(obj)], text=True)
for name in (".data", ".bss"):
    match = re.search(r"^" + re.escape(name) + r"\s+(\d+)\s", sections, re.MULTILINE)
    assert match and int(match[1]) == 0, "Startup must not require initialized globals"
dis = subprocess.check_output([prefix + "objdump", "-d", str(obj)], text=True)
assert "cpsie" not in dis and "blx" not in dis
(OUT / "ciu32_startup.dis").write_text(dis)
manifest = {"hardware_tested": False, "artifact_kind": "relocatable_object",
            "production_firmware_integrated": False,
            "source_sha256": hashlib.sha256((ROOT / "ciu32_startup.c").read_bytes()).hexdigest(),
            "test_sha256": hashlib.sha256((ROOT / "test_startup.c").read_bytes()).hexdigest(),
            "object_sha256": hashlib.sha256(obj.read_bytes()).hexdigest()}
(OUT / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
print(json.dumps(manifest, indent=2))
print("Strict, sanitizer and Cortex-M0+ object checks passed; not a firmware image.")
