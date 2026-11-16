# Aqara W600 custom firmware

Replacement controller firmware for the Aqara W600 (`lumi.airrtc.aeu005`) with
the **CIU32L071** MCU. Uses Zigbee through the original radio; the radio and
bootloader are not replaced.

## What it fixes

- Uses the external room temperature for control, not just for display. Missing
  or stale readings trigger a close command, not a switch to the internal sensor.
- Keeps Off in charge: temperature reports and target refreshes cannot reopen a
  valve that has been turned off.
- Starts opening below the room target without waiting for a 2°C drop, responds
  to small target changes, and requests full closure at or above the target.
- Fixes the stock motor/calibration hang that could keep the motor running and
  leave the valve completely unresponsive. Every movement and calibration has
  a timeout.
- Checks end stops using motor feedback between drive pulses, sustained-load
  detection, and a back-off and return.
- Starts off after a restart, without moving the motor automatically.
- Wakes the display when you turn the dial or press the button. Routine Zigbee
  traffic leaves it dark and does not extend the ten-second timeout.
- Keeps dial adjustment available when a motor fault blocks movement.
- Recovers from failed firmware transfers instead of staying stuck in update mode.
- Supports further OTA updates without hard-coding the next image (controller46+).
- Marks the valve position as unknown when calibration is lost. Requires the
  supplied Zigbee2MQTT converter.

The temperature-control changes address problems reported in the
[W600 community issue thread](https://forum.aqara.com/t/w600-thermostat-collection-of-infos-and-open-issues/188010),
including the [external-sensor control issue](https://forum.aqara.com/t/w600-external-sensor-does-not-control-valve-firmware-issue/341137).
Room control requires a separate temperature sensor.

Aqara smart schedules, clock/DST problems, group behaviour and Matter/Thread
support are not covered by these fixes. See
[Compatibility and limits](docs/status.md).

## Install from stock

Start with **stock 0.0.0_2848**, controller 28 / radio 48.

1. Download the ready-to-upload `.ota` file from [Releases](https://github.com/Ripthulhu/aqara-w600-firmware/releases).
2. Follow the [Zigbee2MQTT OTA guide](docs/ota.md) to install the converter and update.
3. Mount the head, calibrate it, and check that the radiator both heats and shuts
   off before enabling automatic control.

GitHub Actions packages each release using a checked stock download, retaining
the original radio. You do not need a local build or packaging tools.

## Development

[Build and test](docs/build.md) · [Hardware schematic and pin map](docs/hardware.md) · [Compatibility and limits](docs/status.md)

Experimental firmware: keep SWD recovery available. Battery life and long-term
reliability are not yet qualified. Never calibrate an unmounted head.
