"""Seal/check W600 update identity and integrity; this is not a signature."""
import hashlib
import struct

MAGIC=b'W600-CIU32-OTA1\0'
IDENTITY_BYTES=64

def validate(image,revision):
    if not isinstance(revision,int) or isinstance(revision,bool) or not 46<=revision<=255:
        raise ValueError('Follow-up OTA revision must be 46..255')
    if not 256<=len(image)<=52720 or len(image)%4:
        raise ValueError('Invalid OTA application length')
    magic,version,embedded,length,capabilities=struct.unpack_from('<16s4I',image,len(image)-64)
    if (magic,version,embedded,length,capabilities)!=(MAGIC,1,revision,len(image),1):
        raise ValueError('Wrong W600 update identity, revision, length or OTA capability')
    if hashlib.sha256(image[:-32]).digest()!=image[-32:]:
        raise ValueError('OTA application SHA256 mismatch')

def seal(image,revision):
    out=bytearray(image)
    if len(out)<256 or out[-64:-48]!=MAGIC:
        raise ValueError('Missing linked W600 update identity')
    if struct.unpack_from('<4I',out,len(out)-48)!=(1,revision,0,1) or any(out[-32:]):
        raise ValueError('Update identity is not an unsealed OTA-enabled build')
    struct.pack_into('<I',out,len(out)-40,len(out))
    out[-32:]=hashlib.sha256(out[:-32]).digest()
    validate(out,revision)
    return bytes(out)
