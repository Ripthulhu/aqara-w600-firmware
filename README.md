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

1. Download the custom application from [Releases](https://github.com/Ripthulhu/aqara-w600-firmware/releases).
2. Follow the [Zigbee2MQTT OTA guide](docs/ota.md) to create and install the update.
3. Mount the head, calibrate it, and check that the radiator both heats and shuts
   off before enabling automatic control.

The packer uses a stock download supplied by you, so this repository does not
redistribute Aqara's radio firmware. A raw `.bin` is not an OTA file.

## Development

[Build and test](docs/build.md) · [Compatibility and limits](docs/status.md)

Experimental firmware: keep SWD recovery available. Battery life and long-term
reliability are not yet qualified. Never calibrate an unmounted head.
