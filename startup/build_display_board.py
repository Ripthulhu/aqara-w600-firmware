"""Offline native/sanitizer tests and M0+ object only; no device access."""
import hashlib
import json
import os
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parent
sdk = Path(os.environ.get("CIU32_SDK", "/srv/ai-agent/work/aqara-w600-hardware/sdk-build/vendor-sdk/CIU32L071 EclipseExampleProject"))
out = root / "build-display-board"
out.mkdir(exist_ok=True)
inc = ["-isystem", str(sdk / "Drivers/CMSIS/Device"), "-isystem", str(sdk / "Drivers/CMSIS/Core")]
flags = ["-std=c11", "-Wall", "-Wextra", "-Werror", "-Wconversion", "-Wshadow", "-pedantic"]
for name, extra in (("strict", ["-O2"]), ("sanitized", ["-O1", "-g", "-fsanitize=address,undefined",
        "-fno-omit-frame-pointer", "-fno-sanitize-recover=all"])):
    exe = out / ("test-display-board-" + name)
    subprocess.check_call(["gcc", *flags, *inc, *extra, str(root / "test_display_board.c"), "-o", str(exe)])
    subprocess.check_call([str(exe)])
obj = out / "w600_display_board.o"
subprocess.check_call(["arm-none-eabi-gcc", *flags, *inc, "-mcpu=cortex-m0plus", "-mthumb", "-Os",
    "-ffreestanding", "-fno-builtin", "-fstack-usage", "-c", str(root / "w600_display_board.c"), "-o", str(obj)])
undefined = subprocess.check_output(["arm-none-eabi-nm", "-u", str(obj)], text=True).strip()
assert not undefined, undefined
subprocess.check_call(["arm-none-eabi-size", str(obj)])
print((out / "w600_display_board.su").read_text())
manifest = {"hardware_tested": False, "app_linked": False,
    "object_sha256": hashlib.sha256(obj.read_bytes()).hexdigest(), "undefined_symbols": undefined,
    "sources": {name: hashlib.sha256((root / name).read_bytes()).hexdigest() for name in
        ("w600_display_board.c", "w600_display_board.h", "test_display_board.c", "build_display_board.py")}}
(out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
print(json.dumps(manifest, indent=2))
