# Install with Zigbee2MQTT

For a CIU32L071 W600 running **stock 0.0.0_2848** (controller 28 / radio 48).
Use stable power, update one head at a time, and pause automations that control
that head. Keep SWD recovery available: this firmware is experimental.

## 1. Create the OTA file

Download `w600-controller44.bin` and `SHA256SUMS` from the
[release](https://github.com/Ripthulhu/aqara-w600-firmware/releases), and verify
the application's SHA-256 against `SHA256SUMS`.

Download the [stock 2848 donor](https://raw.githubusercontent.com/Koenkk/zigbee-OTA/master/images/Lumi/20260626190610_OTA_lumi.airrtc.aeu005_0.0.0_2848_20260623_071869.ota)
and save it as `W600-2848.ota`. Its SHA-256 must be:

```text
9929022f8755a23099efac929250284580296472d5fb98adf4513b5861d76d4d
```

From a checkout of this repository, with Python 3:

```sh
python3 tools/package_ota.py --stock W600-2848.ota \
  --image w600-controller44.bin --revision 44 --output w600-custom44.ota
```

The packer checks the donor and application, keeps the stock radio unchanged,
and writes `w600-custom44.ota` plus a checksum manifest. It does not contact or
flash a valve. Do not rename a raw `.bin` to `.ota`.

## 2. Install the converter

Install the [reference-status converter](../zigbee2mqtt/README.md)
before updating the head. It prevents an old cached calibration
status from hiding a lost position reference.

## 3. Send the update

Copy `w600-custom44.ota` to the running Zigbee2MQTT instance's data directory,
for example `/app/data/ota/w600-custom44.ota`. Briefly press the valve's button
to wake it; do not hold it.

Publish this **non-retained** MQTT message to
`zigbee2mqtt/bridge/request/device/ota_update/update`, replacing the device ID:

```json
{
  "id": "0xYOUR_DEVICE_IEEE",
  "url": "/app/data/ota/w600-custom44.ota"
}
```

The path is on Zigbee2MQTT's machine/container, not the computer showing its
web interface. You can also use its frontend's custom firmware upload where
available. These are [Zigbee2MQTT's custom-update mechanisms](https://www.zigbee2mqtt.io/guide/usage/ota_updates.html#using-custom-firmware-files-or-index).

Watch the progress and matching update response. After the head restarts,
request an update check and verify that the **device reports file version
11312 (`0x2c30`)**. Transfer completion alone does not prove the controller
installed the application.

## 4. Calibrate and restore control

The application starts off. Wait at least two seconds after restart.
Once the head is mounted, run calibration from
Zigbee2MQTT and wait for completion; allow up to six minutes. Confirm that
opening heats the radiator and closing shuts it off, then restore your room
automations. Select the external temperature sensor and supply its readings
from your room sensor.

The stock-to-custom packaging path is tested. Each release separately states
whether its exact application has had mounted hardware testing.
