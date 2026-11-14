# W600 position status after a controller restart

This companion adapter is required for accurate lost-reference reporting with
the custom firmware. It wraps Zigbee2MQTT's bundled W600 definition and handles
private position `0xffff` when the controller loses its position reference.
That marker is not a stock position value or a stock fault. Do not enable the
adapter for heads running stock firmware.

The stock radio forwards that marker as public position `6553.5`. The adapter
shows an unknown position, `not_ready`, and idle instead. It leaves real
calibration errors and progress visible. A cached Ready response cannot restore
an old position. A new calibration must report progress, then a valid position
and Ready; cache reads do not count.

The radio hop, unknown display, and cached-read latch were checked with
Zigbee2MQTT 2.14.3 / zigbee-herdsman-converters 26.125.0. Completion ordering,
reload-state recovery, and error handling have offline tests. This is not a
claim that every W600 radio version forwards the marker the same way.

## Install

1. Back up Zigbee2MQTT's data directory. Keep the heads off and pause any
   recovery/calibration automations during installation.
2. Copy `w600_reference.mjs` to `custom/w600_reference.mjs` inside Zigbee2MQTT's
   data directory. Do **not** put this helper in `external_converters`.
3. Edit `customDevices` in `w600_reference_converter.mjs`. Replace the example
   IEEE address with the exact addresses of your custom-firmware heads. The
   example is rejected intentionally. Matching uses both model and exact IEEE
   address; other heads keep the native definition.
4. Save the edited wrapper as `external_converters/w600_reference_converter.mjs`
   through Zigbee2MQTT's external-converter editor/save API. The helper path is
   relative to that folder. The save API loads it without restarting Zigbee2MQTT.
   Check for a successful load before re-enabling control. Existing converters
   do not need to be replaced.
5. After a controller restart or OTA, wait at least **two seconds** before
   starting calibration. A late startup marker deliberately invalidates any
   earlier calibration attempt. Confirm a fresh successful calibration before
   restoring automatic control.

The adapter imports the installed native W600 definition; it does not contain
a copy. Native controls and OTA support remain in that definition. Removing
the external wrapper restores native decoding, but does not itself establish
the controller's position: require fresh honest status or calibration before
resuming control.

## Offline checks

From this folder, run `node --test test_w600_reference.mjs`. No broker, device,
or extra package is needed for these tests. The helper has an empty allowlist
by default; calling it without an explicit device list changes nothing.
