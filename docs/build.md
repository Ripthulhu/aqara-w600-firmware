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

Set the SDK path explicitly; the source's private-lab fallback is not a
downloadable dependency:

```sh
export CIU32_SDK='/absolute/path/to/CIU32L071 EclipseExampleProject'
python3 firmware/build.py \
  --flash-application \
  --controller-revision 44 \
  --development-link-only
```

Output is `firmware/build-application/replacement-core.bin`, an ELF, and a
manifest containing hashes/test results. This command does **not** flash a head.
`--development-link-only` leaves runtime qualification pending; its image is not
an approved release. The full lab model run also needs `unicorn`, `pyelftools`,
and recorded fixtures. Do not use bench-motor options on a mounted or unattended
head.

## Update authorization and packaging

Use `tools/package_ota.py` with the stock donor and custom application as shown
in the [OTA guide](ota.md). The public packer needs no firmware dump; the bundled
obfuscation stream is not executable firmware. Aqara's radio payload stays in
the donor and generated local package, not this repository.

The supplied application does not authorize a subsequent custom application.
Further custom updates currently require a wired installation or an installed
build explicitly authorizing the next image.
