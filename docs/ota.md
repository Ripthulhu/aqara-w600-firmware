# Install with Zigbee2MQTT

For a CIU32L071 W600 running stock **0.0.0_2848** (controller 28 / radio 48).
The firmware is experimental; keep SWD recovery available.

## 1. Download

Download the `.ota` file and its `.sha256` from
[Releases](https://github.com/Ripthulhu/aqara-w600-firmware/releases).
GitHub Actions packages the OTA file and checks the firmware and stock donor.

## 2. Install the converter

Add [w600_custom.mjs](../zigbee2mqtt/README.md) through Zigbee2MQTT's external-converter
editor and save. It detects custom-firmware heads automatically.

## 3. Update

Pause automations controlling the head and turn it Off. Use Zigbee2MQTT's
custom-firmware upload to send the `.ota` file. Briefly press its button to wake
it when starting the update. Update one head at a time with stable power.

After restart, request an update check and confirm the installed version matches
the release. Transfer completion alone does not confirm installation.
If the head still reports the old controller version after the update has
finished and it is responding again, power-cycle it once and check again.

## 4. Calibrate

The head starts Off after restart and rejects Heat requests until calibration
restores its position reference.
Once mounted, wait at least two seconds after restart, then start calibration
from Zigbee2MQTT. Allow up to six minutes.

Check that the radiator heats when open and shuts off when closed. Select the
external temperature sensor, supply its room readings, then restore your
automations. Never calibrate an unmounted head.

## MQTT alternative

Publish a non-retained message to
`zigbee2mqtt/bridge/request/device/ota_update/update`:

```json
{
  "id": "0xYOUR_DEVICE_IEEE",
  "url": "RELEASE_ASSET_URL"
}
```

See [Zigbee2MQTT's custom-update documentation](https://www.zigbee2mqtt.io/guide/usage/ota_updates.html#using-custom-firmware-files-or-index)
for local-file and custom-index options. Developer packaging instructions are
in [Build and test](build.md).

## Updating an early custom build

Custom controllers 46 and later support subsequent OTA updates. Our controllers
43–45 had an update-check bug and require one wired update before OTA works
again. This restriction does not apply to the stock firmware.
