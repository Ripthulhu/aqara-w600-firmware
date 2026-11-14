"""Synthetic packaging tests: no manufacturer binary, network, or valve needed."""
import binascii
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch
import zlib

import package_ota as ota
from test_ota_image import image as sealed_image


class PackageTests(unittest.TestCase):
    def setUp(self):
        self.stream = bytes((i * 17 + 13) % 256 for i in range(52720))
        self.header_key = bytes((i * 31 + 3) % 256 for i in range(128))
        plain = bytearray(128)
        struct.pack_into('<I', plain, 0, 0x6c756d69)
        struct.pack_into('<I', plain, 8, 128)
        struct.pack_into('<III', plain, 32, 52720, 0x2a79, 28)
        struct.pack_into('<I', plain, 124, binascii.crc_hqx(plain[:124], 0))
        self.plain_header = bytes(plain)
        self.metadata = bytes((i * 7 + 5) % 256 for i in range(96))
        parts = [(0, b'R' * 609632), (0xf007, self.metadata),
                 (0xf000, ota.xor(plain, self.header_key) + b'C' * 52720)]
        body = b''.join(struct.pack('<HI', tag, len(data)) + data for tag, data in parts)
        self.stock = struct.pack('<IHHHHHIH32sI', 0x0beef11e, 0x100, 56, 0, 0x115f, 0x1400, 0x1c30, 2,
                                 b'SYNTHETIC TEST ONLY', 56 + len(body)) + body
        self.image = bytearray(252)
        struct.pack_into('<II', self.image, 0, 0x20007000, 0x08002081)
        self.pins = patch.multiple(ota, STOCK_SHA256=ota.sha256(self.stock), STREAM_SHA256=ota.sha256(self.stream))
        self.pins.start()
        self.addCleanup(self.pins.stop)

    def build(self, **changes):
        return ota.build_package(changes.get('stock', self.stock), changes.get('image', self.image),
                                 changes.get('revision', 44), changes.get('stream', self.stream))

    def test_payload_crc_metadata_and_unchanged_radio(self):
        package, facts = self.build()
        version, parts = ota.parse_ota(package)
        self.assertEqual(version, 0x2c30)
        self.assertEqual(parts[0], ota.parse_ota(self.stock)[1][0])
        body = ota.xor(parts[-1][2][128:], self.stream[:256])
        self.assertEqual(body, bytes(self.image) + b'\xff' * 4)
        header = ota.xor(parts[-1][2][:128], self.header_key)
        self.assertEqual(struct.unpack_from('<III', header, 32), (252, binascii.crc_hqx(self.image, 0), 44))
        self.assertEqual(struct.unpack_from('<I', header, 124)[0], binascii.crc_hqx(header[:124], 0))
        metadata = parts[1][2]
        self.assertEqual(metadata[0x4d:0x51], self.metadata[0x4d:0x51])
        allowed = set(range(28, 32)) | set(range(0x42, 0x48))
        self.assertTrue(all(metadata[i] == self.metadata[i] for i in range(96) if i not in allowed))
        self.assertEqual(struct.unpack_from('<I', metadata, 28)[0] ^ struct.unpack_from('<I', self.metadata, 28)[0],
                         zlib.crc32(metadata[32:]) ^ zlib.crc32(self.metadata[32:]))
        self.assertTrue(facts['radio_payload_unchanged'])

    def test_wrong_donor_or_stream_is_refused(self):
        for changes in ({'stock': self.stock[:-1]}, {'stream': self.stream[:-1]}, {'stream': bytes(52720)}):
            with self.subTest(changes=list(changes)), self.assertRaises(ValueError):
                self.build(**changes)

    def test_follow_up_image_is_verified_not_resealed(self):
        image=sealed_image(47)
        package,facts=self.build(image=image,revision=47)
        _,parts=ota.parse_ota(package)
        self.assertEqual(ota.xor(parts[-1][2][128:],self.stream[:len(image)]),image)
        for data in (self.image,image[:-4],image[:-1]+bytes((image[-1]^1,))):
            with self.assertRaises(ValueError):self.build(image=data,revision=47)
        with self.assertRaises(ValueError):self.build(image=image,revision=48)

    def test_invalid_revision_lengths_and_vectors(self):
        for revision in (0, 28, 256, -1, True, 44.5):
            with self.subTest(revision=revision), self.assertRaises(ValueError):
                self.build(revision=revision)
        for length in (0, 188, 251, 52724):
            with self.subTest(length=length), self.assertRaises(ValueError):
                self.build(image=bytes(length))
        for sp, entry in ((0, 0x08002081), (0x20007001, 0x08002081), (0x20007000, 0x08002080), (0x20007000, 0x08010001)):
            image = bytearray(self.image)
            struct.pack_into('<II', image, 0, sp, entry)
            with self.assertRaises(ValueError):
                self.build(image=image)

    def test_parser_rejects_truncated_and_inconsistent_input(self):
        for data in (b'', self.stock[:-1], self.stock + b'X'):
            with self.assertRaises(ValueError):
                ota.parse_ota(data)
        broken = bytearray(self.stock)
        struct.pack_into('<I', broken, 58, 0xffffffff)
        with self.assertRaises(ValueError):
            ota.parse_ota(broken)

    def test_existing_different_output_is_never_replaced(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'test.ota'
            ota.write_new_or_identical(path, b'original')
            ota.write_new_or_identical(path, b'original')
            with self.assertRaises(FileExistsError):
                ota.write_new_or_identical(path, b'changed')
            self.assertEqual(path.read_bytes(), b'original')


if __name__ == '__main__':
    unittest.main()
