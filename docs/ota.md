# Install with Zigbee2MQTT

For a CIU32L071 W600 running stock **0.0.0_2848** (controller 28 / radio 48).
The firmware is experimental; keep SWD recovery available.

Controller46 and later can use the same procedure for subsequent updates.
Controllers43–45 require one wired update first: their installed update check
rejects all follow-up images. Sending a different OTA package cannot change it.

## 1. Download

Download the `.ota` file and its `.sha256` from
[Releases](https://github.com/Ripthulhu/aqara-w600-firmware/releases).
Choose controller46 or later for follow-up OTA support.
GitHub Actions packages the OTA file and checks the firmware and stock donor.

## 2. Install the converter

Add [w600_custom.mjs](../zigbee2mqtt/README.md) through Zigbee2MQTT's external-converter
editor and save. It detects custom-firmware heads automatically.

## 3. Update

Pause automations controlling the head. Use Zigbee2MQTT's custom-firmware upload
to send the `.ota` file to the valve. Briefly press its button to wake it when
starting the update. Update one head at a time with stable power.

After restart, request an update check and confirm the installed version matches
the release. Transfer completion alone does not confirm installation.

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
  "url": "RELEASE_ASSET_URL"
}
```

See [Zigbee2MQTT's custom-update documentation](https://www.zigbee2mqtt.io/guide/usage/ota_updates.html#using-custom-firmware-files-or-index)
for local-file and custom-index options. Developer packaging instructions are
in [Build and test](build.md).
