# Compatibility and limits

- **Hardware:** CIU32L071-based Aqara W600 (`lumi.airrtc.aeu005`), original
  bootloader and Zigbee radio 48.
- **Starting firmware:** stock `0.0.0_2848`. Other hardware and stock versions
  are not qualified.
- **Control:** separate room-temperature input; starts off and needs a mounted
  calibration before normal regulation. Install the reference-status converter
  so unknown travel is not shown as an old valid position.
- **OTA:** stock-to-custom installation has been verified. Controller46 and
  later support successive custom updates. Controllers43–45 need one wired
  update to enable this; they cannot install that repair over OTA themselves.
- **Update trust:** OTA trusts the paired Zigbee network. Newer W600 images
  must retain OTA support and pass identity, size, vector, CRC and SHA256 checks.
  The hash detects corruption; updates are not cryptographically signed.
- **Release quality:** experimental. Test coverage is listed in each release.

Battery runtime, long-term wear, exact physical opening percentage, and power-loss
rollback remain unqualified. Keep SWD recovery available. This replaces the valve
controller, not the radio or the boiler's own controls.

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
Matter/Thread problems. Those are not qualified fixes in this Zigbee firmware.
The old Zigbee2MQTT 10% position-scaling issue was fixed upstream, not here.
