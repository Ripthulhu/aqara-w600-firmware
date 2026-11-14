"""Offline actual-source MMIO tests and Cortex-M0+ object; no device access."""
import hashlib
import json
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parent
SDK = Path(os.environ.get("CIU32_SDK", "/srv/ai-agent/work/aqara-w600-hardware/sdk-build/vendor-sdk/CIU32L071 EclipseExampleProject"))
OUT = ROOT / "build-feedback"
OUT.mkdir(exist_ok=True)
includes = ["-isystem", str(SDK / "Drivers/CMSIS/Core"),
            "-isystem", str(SDK / "Drivers/CMSIS/Device")]
warnings = ["-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic"]
for suffix, extra in (("", []), ("-sanitize", ["-fsanitize=address,undefined",
        "-fno-omit-frame-pointer", "-fno-sanitize-recover=all"])):
    executable = OUT / ("test-feedback" + suffix)
    subprocess.check_call(["gcc", *warnings, "-O1", "-g", *extra, *includes,
        str(ROOT / "test_feedback.c"), "-o", str(executable)])
    subprocess.check_call([str(executable)])
prefix = os.environ.get("ARM_TOOL_PREFIX", "arm-none-eabi-")
obj = OUT / "w600_feedback.o"
subprocess.check_call([prefix + "gcc", *warnings, "-Os", "-g3", "-mcpu=cortex-m0plus",
    "-mthumb", "-ffreestanding", "-fno-builtin", "-fno-common", "-fstack-usage", *includes,
    "-c", str(ROOT / "w600_feedback.c"), "-o", str(obj)])
undefined = subprocess.check_output([prefix + "nm", "-u", str(obj)], text=True).strip()
helpers = {line.split()[-1] for line in undefined.splitlines()}
assert helpers <= {"__aeabi_lmul", "__aeabi_uldivmod", "__aeabi_uidiv", "__aeabi_ldivmod"}, undefined
# The application already links libgcc; verify the new voltage arithmetic resolves there.
linked = OUT / "w600_feedback-linked.o"
subprocess.check_call([prefix + "gcc", "-mcpu=cortex-m0plus", "-mthumb", "-nostdlib", "-r",
    str(obj), "-lgcc", "-o", str(linked)])
linked_undefined = subprocess.check_output([prefix + "nm", "-u", str(linked)], text=True).strip()
assert not linked_undefined, linked_undefined
dis = subprocess.check_output([prefix + "objdump", "-dr", str(obj)], text=True)
assert "cpsie" not in dis and "cpsid" not in dis and "blx" not in dis
(OUT / "w600_feedback.dis").write_text(dis)
manifest = {"hardware_tested": False, "production_firmware_integrated": False,
            "artifact_kind": "relocatable_object", "undefined_symbols": undefined,
            "libgcc_helpers_resolved": sorted(helpers), "linked_undefined_symbols": linked_undefined,
            "files": {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
                      for name in ("w600_feedback.c", "w600_feedback.h", "test_feedback.c", "build_feedback.py")},
            "object_sha256": hashlib.sha256(obj.read_bytes()).hexdigest()}
(OUT / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
print(json.dumps(manifest, indent=2))
print("Strict, sanitizer and Cortex-M0+ compile passed; not an executable firmware image.")
