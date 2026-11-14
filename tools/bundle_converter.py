"""Create the single-file Zigbee2MQTT converter shipped with releases."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / 'zigbee2mqtt'
HELPER_IMPORT = "import {withReferenceState} from './w600_reference.mjs';"


def bundle():
    wrapper = (ROOT / 'w600_reference_converter.mjs').read_text()
    helper = (ROOT / 'w600_reference.mjs').read_text().replace('export ', '')
    if wrapper.count(HELPER_IMPORT) != 1:
        raise ValueError('Converter helper import changed; update the bundler')
    return ('// Generated from the converter sources; install this file unchanged.\n' +
            wrapper.replace(HELPER_IMPORT, helper)).encode()


if __name__ == '__main__':
    import sys
    sys.stdout.buffer.write(bundle())
