"""Build/package identity guards, no hardware or SDK."""
import hashlib
import struct
import unittest
from pathlib import Path
from ota_image import MAGIC, seal, validate

def image(revision=47,length=768):
    data=bytearray((i*17+3)%256 for i in range(length))
    struct.pack_into('<II',data,0,0x20007000,0x080020c1)
    data[-64:]=struct.pack('<16s4I32s',MAGIC,1,revision,0,1,bytes(32))
    return seal(data,revision)

class ImageTests(unittest.TestCase):
    def test_default_application_keeps_install_and_restart_connected(self):
        app=(Path(__file__).with_name('app.c')).read_text()
        self.assertNotIn('W600_OTA_PINNED_BUILD',app)
        self.assertIn('w600_ota_flash_init(&app_ticks,ota_authorize,NULL)',app)
        self.assertIn('if(ota.state==W600_OTA_COMMITTED)',app)
        build=Path(__file__).with_name('build.py').read_text()
        self.assertNotIn('--ota-allow-image',build)
        self.assertIn('"ota_commit_authorization_enabled":flash',build)

    def test_seal_and_hash(self):
        for length in (256,260,768,52720):
            data=image(length=length)
            validate(data,47)
            self.assertEqual(data[-32:],hashlib.sha256(data[:-32]).digest())

    def test_every_byte_corruption_is_rejected(self):
        data=image()
        for i in range(len(data)):
            broken=bytearray(data);broken[i]^=1
            with self.assertRaises(ValueError):validate(broken,47)

    def test_identity_and_capability_cannot_be_relabelled(self):
        data=image()
        for offset,value in ((-48,2),(-44,48),(-40,len(data)-4),(-36,0)):
            broken=bytearray(data);struct.pack_into('<I',broken,len(data)+offset,value)
            broken[-32:]=hashlib.sha256(broken[:-32]).digest()
            with self.assertRaises(ValueError):validate(broken,47)
        for revision in (0,45,46,48,256,True):
            with self.assertRaises(ValueError):validate(data,revision)

    def test_unsealed_or_already_sealed_input_is_rejected(self):
        data=image()
        with self.assertRaises(ValueError):seal(data,47)
        with self.assertRaises(ValueError):seal(bytes(768),47)
        with self.assertRaises(ValueError):validate(data[:-4],47)

if __name__=='__main__':unittest.main()
