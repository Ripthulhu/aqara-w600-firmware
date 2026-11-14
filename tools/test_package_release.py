"""Release packaging guards; no network or valve access."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import package_release as release


class ReleaseTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.inputs = self.root / 'input'
        self.inputs.mkdir()
        self.image = self.inputs / 'w600-controller44.bin'
        self.image.write_bytes(b'application')
        self.checksums = self.inputs / 'SHA256SUMS'
        self.checksums.write_text(release.sha256(b'application') + '  ' + self.image.name + '\n')
        self.stock = self.root / 'stock.ota'
        self.stock.write_bytes(b'stock')
        self.output = self.root / 'output'

    def prepare(self):
        return release.prepare(self.inputs, self.stock, self.output)

    @patch.object(release, 'build_package', return_value=(b'ota', {'revision': 44}))
    def test_verified_application_reaches_existing_packer(self, build):
        files = self.prepare()
        self.assertEqual(files[0].read_bytes(), b'ota')
        self.assertEqual(json.loads(files[1].read_text()), {'revision': 44})
        self.assertEqual(files[2].read_text(), release.sha256(b'ota') + '  w600-custom44.ota\n')
        self.assertEqual(files[3].name, 'w600_custom.mjs')
        self.assertIn(b'export default withReferenceState(native)', files[3].read_bytes())
        self.assertEqual(build.call_args.args[:3], (b'stock', b'application', 44))

    @patch.object(release, 'build_package')
    def test_checksum_errors_stop_before_packaging(self, build):
        valid = self.checksums.read_text()
        for text in ('', '0' * 64 + '  ' + self.image.name, valid + valid):
            self.checksums.write_text(text)
            with self.assertRaises(ValueError):
                self.prepare()
        build.assert_not_called()

    def test_missing_ambiguous_or_malformed_image_is_refused(self):
        self.image.unlink()
        with self.assertRaises(ValueError):
            self.prepare()
        for name in ('w600-controller44.bin', 'w600-controller45.bin'):
            (self.inputs / name).write_bytes(b'application')
        with self.assertRaises(ValueError):
            self.prepare()
        (self.inputs / 'w600-controller45.bin').unlink()
        self.image.rename(self.inputs / 'w600-controller44-extra.bin')
        with self.assertRaises(ValueError):
            self.prepare()

    @patch.object(release.subprocess, 'run')
    @patch.object(release.subprocess, 'check_output', return_value='{"assets": []}')
    def test_publish_uploads_only_missing_files(self, view, run):
        release.publish('controller44-alpha1', [self.image])
        self.assertEqual(run.call_args.args[0],
                         ['gh', 'release', 'upload', 'controller44-alpha1', str(self.image)])

    @patch.object(release.subprocess, 'run')
    @patch.object(release.subprocess, 'check_output')
    def test_existing_assets_must_match_before_any_upload(self, view, run):
        view.return_value = json.dumps({'assets': [{'name': self.image.name}]})
        def download(command, **kwargs):
            folder = Path(command[command.index('--dir') + 1])
            (folder / self.image.name).write_bytes(b'application')
        run.side_effect = download
        release.publish('controller44-alpha1', [self.image])
        self.assertEqual(run.call_count, 1)
        run.reset_mock()
        run.side_effect = lambda command, **kwargs: (Path(command[-1]) / self.image.name).write_bytes(b'different')
        with self.assertRaises(ValueError):
            release.publish('controller44-alpha1', [self.image, self.stock])
        self.assertEqual(run.call_count, 1)

    @patch.object(release.subprocess, 'check_output')
    def test_invalid_tag_never_reaches_github(self, view):
        for tag in ('--help', '', 'tag;command', 'tag with spaces'):
            with self.assertRaises(ValueError):
                release.publish(tag, [])
        view.assert_not_called()


if __name__ == '__main__':
    unittest.main()
