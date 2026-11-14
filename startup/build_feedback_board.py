"""Offline actual-source lifecycle tests and Cortex-M0+ object; no device access."""
import hashlib
import json
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parent
SDK = Path(os.environ.get("CIU32_SDK", "/srv/ai-agent/work/aqara-w600-hardware/sdk-build/vendor-sdk/CIU32L071 EclipseExampleProject"))
OUT = Path(os.environ.get("BUILD_OUT", str(ROOT / "build-feedback-board")))
OUT.mkdir(exist_ok=True)
includes = ["-isystem", str(SDK / "Drivers/CMSIS/Core"),
            "-isystem", str(SDK / "Drivers/CMSIS/Device")]
warnings = ["-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic"]
prefix = os.environ.get("ARM_TOOL_PREFIX", "arm-none-eabi-")
objects = {}
for profile, defines in (("", []), ("-display", ["-DW600_DISPLAY_BENCH"])):
    for suffix, extra in (("", []), ("-sanitize", ["-fsanitize=address,undefined",
            "-fno-omit-frame-pointer", "-fno-sanitize-recover=all"])):
        exe = OUT / ("test-feedback-board" + profile + suffix)
        subprocess.check_call(["gcc", *warnings, "-O1", "-g", *defines, *extra, *includes,
            str(ROOT / "test_feedback_board.c"), "-o", str(exe)])
        subprocess.check_call([str(exe)])
    obj = OUT / ("w600_feedback_board" + profile + ".o")
    subprocess.check_call([prefix + "gcc", *warnings, "-Os", "-g3", "-mcpu=cortex-m0plus",
        "-mthumb", "-ffreestanding", "-fno-builtin", "-fno-common", "-fstack-usage",
        *defines, *includes, "-c", str(ROOT / "w600_feedback_board.c"), "-o", str(obj)])
    undefined = subprocess.check_output([prefix + "nm", "-u", str(obj)], text=True).strip()
    assert not undefined, undefined
    dis = subprocess.check_output([prefix + "objdump", "-dr", str(obj)], text=True)
    assert "cpsie" not in dis and "cpsid" not in dis and "blx" not in dis
    (OUT / ("w600_feedback_board" + profile + ".dis")).write_text(dis)
    objects[profile or "default"] = hashlib.sha256(obj.read_bytes()).hexdigest()
manifest = {"hardware_tested": False, "production_firmware_integrated": False,
    "artifact_kind": "relocatable_object", "undefined_symbols": undefined,
    "files": {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
        for name in ("w600_feedback_board.c", "w600_feedback_board.h",
                     "test_feedback_board.c", "build_feedback_board.py")},
    "object_sha256": objects["default"], "profile_objects": objects}
(OUT / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
print(json.dumps(manifest, indent=2))
print("Strict, sanitizer and Cortex-M0+ compile passed; not executable firmware.")
