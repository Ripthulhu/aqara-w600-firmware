# Install with Zigbee2MQTT

For a CIU32L071 W600 running stock **0.0.0_2848** (controller 28 / radio 48).
The firmware is experimental; keep SWD recovery available.

## 1. Download

Download `w600-custom44.ota` and its `.sha256` file from the
[release](https://github.com/Ripthulhu/aqara-w600-firmware/releases/tag/controller44-alpha1).
GitHub Actions packages the OTA file and checks the firmware and stock donor.

## 2. Install the converter

Install the [reference-status converter](../zigbee2mqtt/README.md) for your
custom-firmware heads. It keeps lost calibration from appearing as a valid
valve position.

## 3. Update

Pause automations controlling the head. Use Zigbee2MQTT's custom-firmware upload
to send the `.ota` file to the valve. Briefly press its button to wake it when
starting the update. Update one head at a time with stable power.

After restart, request an update check and confirm file version **11312
(`0x2c30`)**. Transfer completion alone does not confirm installation.

## 4. Calibrate

The head starts off. Once mounted, wait at least two seconds after restart,
then start calibration from Zigbee2MQTT. Allow up to six minutes.

Check that the radiator heats when open and shuts off when closed. Select the
external temperature sensor, supply its room readings, then restore your
automations. Never calibrate an unmounted head.

## MQTT alternative

Publish a non-retained message to
`zigbee2mqtt/bridge/request/device/ota_update/update`:

```json
{
  "id": "0xYOUR_DEVICE_IEEE",
  "url": "https://github.com/Ripthulhu/aqara-w600-firmware/releases/download/controller44-alpha1/w600-custom44.ota"
}
```

See [Zigbee2MQTT's custom-update documentation](https://www.zigbee2mqtt.io/guide/usage/ota_updates.html#using-custom-firmware-files-or-index)
for local-file and custom-index options. Developer packaging instructions are
in [Build and test](build.md).
