"""Run public, SDK-free C fixtures; never contacts hardware."""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
C = 'replacement-control/'
F = 'firmware/'
CASES = [(C+'test_w600_'+name+'.c', [C+p for p in sources], []) for name, sources in (
    ('frame', ['w600_frame.c']),
    ('endpoint', ['w600_frame.c', 'w600_endpoint.c']),
    ('pairing', ['w600_frame.c', 'w600_pairing.c']),
    ('temperature', ['w600_temperature.c']),
    ('buttons', ['w600_buttons.c']),
    ('display', ['w600_display.c']),
    ('settings', ['w600_settings.c']),
    ('settings_store', ['w600_settings.c', 'w600_settings_store.c']),
    ('settings_service', ['w600_settings.c', 'w600_settings_store.c', 'w600_settings_service.c']),
)]
for name, sources in (
    ('motor_guard', ['motor_guard.c']),
    ('motor_homing', ['motor_homing.c', 'motor_guard.c']),
    ('motor_position', ['motor_position.c']),
    ('motor_slowdown', ['motor_slowdown.c']),
    ('motor_calibration', ['motor_calibration.c', 'motor_homing.c', 'motor_guard.c', 'motor_slowdown.c']),
    ('motor_confirmation', ['motor_calibration.c', 'motor_homing.c', 'motor_guard.c', 'motor_slowdown.c']),
):
    CASES.append((C+'test_'+name+'.c', [C+p for p in sources], []))
CASES += [
    (F+'test_app.c', [C+'w600_buttons.c', C+'w600_endpoint.c'], []),
    (F+'test_app.c', [C+'w600_display.c', C+'w600_buttons.c', C+'w600_endpoint.c'], ['-DW600_DISPLAY_BENCH=1']),
    (F+'test_controller_revision.c', [C+'w600_endpoint.c'], []),
    (F+'test_valve_calibration.c', [C+'motor_calibration.c', C+'motor_position.c', C+'motor_homing.c',
                                  C+'motor_guard.c', C+'w600_endpoint.c', C+'motor_slowdown.c'], []),
    (F+'test_display_tx.c', [F+'display_tx.c'], ['-DW600_DISPLAY_TX_TEST=1']),
    (F+'test_ota_auth.c', [F+'ota_auth.c'], ['-DW600_OTA_AUTH_TEST=1']),
    (F+'test_ota_receiver.c', [F+'ota_receiver.c'], []),
    (F+'test_ota_flash.c', [F+'ota_flash.c'], ['-DW600_OTA_FLASH_TEST=1']),
    (F+'test_ota_chain.c', [F+'ota_receiver.c', F+'ota_auth.c', F+'ota_flash.c'], ['-DW600_OTA_FLASH_TEST=1','-DW600_OTA_AUTH_TEST=1']),
]
for ota in (False, True):
    CASES.append((F+'test_radio.c', [F+'radio.c', C+'w600_frame.c', C+'w600_endpoint.c', C+'w600_pairing.c'] +
                  ([F+'ota_receiver.c'] if ota else []),
                  ['-DW600_RADIO_TEST=1'] + (['-DW600_OTA_RECEIVER=1'] if ota else [])))


def main():
    count = 0
    with tempfile.TemporaryDirectory(prefix='w600-native-') as folder:
        for index, (test, sources, flags) in enumerate(CASES):
            for sanitized in (False, True):
                output = Path(folder) / f'test-{index}-{int(sanitized)}'
                command = [os.environ.get('CC', 'gcc'), '-std=c11', '-Wall', '-Wextra', '-Werror', '-O1', '-g',
                           '-DW600_CONTROLLER_REVISION=44', *flags]
                if sanitized:
                    command += ['-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-fno-sanitize-recover=all']
                subprocess.run([*command, str(ROOT/test), *(str(ROOT/p) for p in sources), '-o', str(output)], check=True)
                subprocess.run([str(output)], check=True)
                count += 1
    print(f'PASS: {count} SDK-free native test builds/runs; no hardware access')


if __name__ == '__main__':
    main()
