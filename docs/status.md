# Compatibility and limits

- **Hardware:** CIU32L071-based Aqara W600 (`lumi.airrtc.aeu005`), original
  bootloader and Zigbee radio 48.
- **Starting firmware:** stock `0.0.0_2848`. Other hardware and stock versions
  have not been tested.
- **Control:** separate room-temperature input; needs a mounted calibration
  after each restart before normal regulation. Saved mode and target settings
  can be restored. Install the reference-status converter
  so unknown travel is not shown as an old valid position.
- **OTA:** stock-to-custom installation has been tested. Custom controllers 46
  and later support subsequent OTA updates. Our controllers 43–45 had an
  update-check bug and need one wired update; Aqara's stock firmware is unaffected.
- **Update trust:** OTA trusts the paired Zigbee network. Newer W600 images
  must retain OTA support and pass identity, size, vector, CRC and SHA256 checks.
  The hash detects corruption; updates are not cryptographically signed.
- **Release quality:** experimental. Test coverage is listed in each release.

Battery life, long-term wear, exact physical opening percentage, and power-loss
rollback are still unproven. Keep SWD recovery available. This replaces the valve
controller; the original radio is retained.

## Known issues in this custom firmware

The current source fixes both issues below. They remain in the downloadable
controller 50 release; the controller 51 test build has not been released.
Its targeted checks pass, but the full simulation suite still has outdated
calibration and clock tests.

- If external-temperature updates stop for about five days, the timer can wrap
  and make the old reading appear fresh again. The valve can then request heat
  from that stale reading.
- A serial-link error during an active OTA transfer can freeze the controller
  until power is cycled. The motor stops, but the expected automatic restart
  does not happen.

## Temperature control

The controller uses the external room reading supplied through Zigbee2MQTT.
It does not silently fall back to the thermometer beside the radiator. A missing
reading or one not updated for twelve minutes requests closure and reports a sensor
problem. Keep the external-temperature updates running.

Below target it uses a simple proportional rule, with a minimum 20% opening
request; at or above target it requests closure. Small target changes are
accepted. These rules are covered by software tests, and mounted opening and
shutoff have been checked. Room-temperature regulation still needs testing on
more radiator types.

The community reports also include Aqara smart-schedule, clock/DST, group and
Matter/Thread problems. This Zigbee firmware does not address those.
The old Zigbee2MQTT 10% position-scaling issue was fixed upstream, not here.

## Motor and local controls

End-stop detection uses feedback between motor pulses, sustained-load detection,
and a back-off and return. A motor fault blocks movement but leaves dial
adjustment available.

Turning the dial or pressing the button lights the display for ten seconds.
Routine Zigbee traffic does not wake it or extend that timeout.
