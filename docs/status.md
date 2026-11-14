# Compatibility and limits

- **Hardware:** CIU32L071-based Aqara W600 (`lumi.airrtc.aeu005`), original
  bootloader and Zigbee radio 48.
- **Starting firmware:** stock `0.0.0_2848`. Other hardware and stock versions
  are not qualified.
- **Control:** separate room-temperature input; starts off and needs a mounted
  calibration before normal regulation. Install the reference-status converter
  so unknown travel is not shown as an old valid position.
- **OTA:** stock-to-custom installation has been verified. Follow-up custom
  updates are not enabled in the supplied application and currently need wired
  installation.
- **Release quality:** experimental. Read the release's exact test coverage.
  Software tests are not a replacement for mounted heat-delivery and shutoff
  checks.

Battery runtime, long-term wear, exact physical opening percentage, and power-loss
rollback remain unqualified. Keep SWD recovery available. This replaces the valve
controller, not the radio or the boiler's own controls.
