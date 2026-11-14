"""Check the downloadable converter as a complete, self-contained module."""
import base64
import subprocess
import unittest

from bundle_converter import bundle


class BundleTests(unittest.TestCase):
    def test_only_native_package_import_remains(self):
        code = bundle().decode()
        imports = [line for line in code.splitlines() if line.startswith('import ')]
        self.assertEqual(imports, ["import {definitions} from 'zigbee-herdsman-converters/devices/lumi';"])
        self.assertNotIn('customDevices', code)
        self.assertNotIn('0x00124b', code)
        self.assertEqual(code.count('export default'), 1)

    def test_bundle_loads_and_autodetects_without_user_edits(self):
        code = bundle().decode().replace(
            "import {definitions} from 'zigbee-herdsman-converters/devices/lumi';",
            "const definitions = [{zigbeeModel: ['lumi.airrtc.aeu005'], ota: true, "
            "fromZigbee: [{convert: (model, msg) => ({position: msg.data[0x0360]})}]}];")
        encoded = base64.b64encode(code.encode()).decode()
        script = """
import assert from 'node:assert/strict';
const {default: converter} = await import('data:text/javascript;base64,ENCODED');
assert.deepEqual(converter.zigbeeModel, ['lumi.airrtc.aeu005']);
assert.equal(converter.ota, true);
const receive = (position, ieee) => converter.fromZigbee[0].convert(null,
  {cluster: 'manuSpecificLumi', type: 'attributeReport', device: {ieeeAddr: ieee}, data: {0x0360: position}},
  () => {}, {}, {});
assert.deepEqual(await receive(24, 'stock'), {position: 24});
assert.deepEqual(await receive(6553.5, 'custom'),
  {position: null, calibrated: 'not_ready', running_state: 'idle', w600_custom_firmware: true});
assert.deepEqual(await receive(50, 'stock'), {position: 50});
""".replace('ENCODED', encoded)
        subprocess.run(['node', '--input-type=module', '-e', script], check=True)


if __name__ == '__main__':
    unittest.main()
