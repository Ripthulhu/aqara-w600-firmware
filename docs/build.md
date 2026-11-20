# Build and test

## Native tests: no device or vendor SDK needed

On Linux with Python 3 and GCC:

```sh
python3 tools/test_native.py
```

This builds portable protocol, control, calibration, display, and OTA fixtures
with strict warnings and again with address/undefined-behavior checks. It does
not contact hardware. GitHub Actions runs the same set. This is not the full lab
suite: some peripheral models and captured-data replays need private fixtures.

## CIU32 application build

Use Linux with Python 3.11+, native GCC, and GNU Arm Embedded tools
(`arm-none-eabi-gcc`, `objcopy`, `nm`, `objdump`). Obtain the **CIU32L071 vendor
SDK separately**. Its project directory must contain `Drivers/CMSIS/Core/`
and `Drivers/CMSIS/Device/ciu32l071.h`.

Set `CIU32_SDK` to the SDK project folder:

```sh
export CIU32_SDK='/absolute/path/to/CIU32L071 EclipseExampleProject'
python3 firmware/build.py \
  --flash-application \
  --controller-revision 50 \
  --development-link-only
```

Output is `firmware/build-application/replacement-core.bin`, an ELF, and a
manifest containing hashes/test results. This command does **not** flash a head.
`--development-link-only` skips the full application tests; do not release its
image. The full lab model run also needs `unicorn`, `pyelftools`,
and recorded fixtures. Do not use bench-motor options on a mounted or unattended
head.

Release 54 uses `--deep-sleep` to enter STOP once the display is off and the
valve is idle. Local builds without the flag use light sleep. The full test
run checks wake from the radio, both dial directions, the button, and the timer.
Standby current and battery life have not been measured for this release.

On a runtime radio or STOP fault, the controller verifies motor shutdown and
requests one recovery reboot if flash is idle and outside OTA. It retains the
fault code in SRAM and uses light sleep after that reboot. Position must be
established again; the firmware does not start calibration on its own. A repeat
fault remains stopped rather than rebooting again. The linked tests inject both
fault classes and check that the recovered controller answers a radio command.

## Update authorization and packaging

The **Package release OTA** workflow runs when a release is published. Create a
draft release, attach exactly one `w600-controllerREVISION.bin` and its entry in
`SHA256SUMS`, then publish. CI downloads the pinned stock 2848 donor, verifies
both inputs, and attaches the `.ota`, packaging manifest, OTA checksum, and
single-file `w600_custom.mjs` converter.
Existing assets must match; CI refuses to replace different files.

To package an existing release, run the workflow manually with its release tag.
This job packages an already compiled application. Source compilation still
requires the vendor SDK described above.

For local packaging, download the [stock 2848 donor](https://raw.githubusercontent.com/Koenkk/zigbee-OTA/master/images/Lumi/20260626190610_OTA_lumi.airrtc.aeu005_0.0.0_2848_20260623_071869.ota)
as `W600-2848.ota`, then run:

```sh
python3 tools/package_ota.py --stock W600-2848.ota \
  --image w600-controller50.bin --revision 50 --output w600-custom50.ota
```

The packer checks the donor's pinned SHA-256 and keeps its radio payload
unchanged. Firmware dumps and the vendor SDK are not included in the source.

Persistent builds always include follow-up OTA. The builder seals the image's
W600 identity, revision, OTA capability and SHA256; the packer verifies these
without changing the image. No next-image hash or special enable flag is needed.
Use a higher controller revision for each update (46–255).

The receiver trusts the paired Zigbee network, checks the whole staged image
again before activation, and restarts after a successful update. These are
integrity checks, not a firmware signature. Keep the Zigbee network trusted.
