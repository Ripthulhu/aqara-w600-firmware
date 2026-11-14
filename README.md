# Aqara W600 custom firmware

Replacement controller firmware for the Aqara W600 (`lumi.airrtc.aeu005`) with
the **CIU32L071** MCU. Uses Zigbee through the original radio; the radio and
bootloader are not replaced.

## What it fixes

- Prevents endless motor runs with timeouts for every movement and calibration.
- Checks end stops using motor feedback between drive pulses, sustained-load
  detection, and a back-off and return.
- Starts off after a restart, without moving the motor automatically.
- Wakes the display when you turn the dial or press the button. Routine Zigbee
  traffic leaves it dark.
- Recovers from failed firmware transfers instead of staying stuck in update mode.
- Marks the valve position as unknown when calibration is lost. Requires the
  supplied Zigbee2MQTT converter.

Room control uses a separate temperature sensor.

## Install from stock

Start with **stock 0.0.0_2848**, controller 28 / radio 48.

1. Download the ready-to-upload `.ota` file from [Releases](https://github.com/Ripthulhu/aqara-w600-firmware/releases).
2. Follow the [Zigbee2MQTT OTA guide](docs/ota.md) to install the converter and update.
3. Mount the head, calibrate it, and check that the radiator both heats and shuts
   off before enabling automatic control.

GitHub Actions packages each release using a checked stock download, retaining
the original radio. You do not need a local build or packaging tools.

## Development

[Build and test](docs/build.md) · [Compatibility and limits](docs/status.md)

Experimental firmware: keep SWD recovery available. Battery life and long-term
reliability are not yet qualified. Never calibrate an unmounted head.
