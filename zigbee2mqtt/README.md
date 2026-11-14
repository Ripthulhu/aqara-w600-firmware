# Zigbee2MQTT setup

Install the single converter file before updating the firmware.

1. Download [w600_custom.mjs](https://github.com/Ripthulhu/aqara-w600-firmware/releases/download/controller44-alpha1/w600_custom.mjs).
2. In Zigbee2MQTT, open **Settings → Dev console → External converters**.
3. Add `w600_custom.mjs`, paste its contents, and save.

If external converters are disabled, enable `advanced.enable_external_js` first.
See [Zigbee2MQTT's external-converter settings](https://www.zigbee2mqtt.io/advanced/more/external_converters.html).

Custom-firmware valves are detected automatically. You do not need to edit
device addresses or copy helper files.

Continue with the [OTA installation guide](../docs/ota.md).

## Development

The release job bundles the converter sources into `w600_custom.mjs`.
Run `node --test zigbee2mqtt/test_w600_reference.mjs` and
`python3 tools/test_bundle_converter.py` from the repository root.
