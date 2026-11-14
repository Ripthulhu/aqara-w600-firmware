"""Create a local W600 Zigbee OTA file from a stock donor and custom application."""
import argparse
import binascii
import hashlib
import json
from pathlib import Path
import struct
import zlib

STOCK_SHA256 = '9929022f8755a23099efac929250284580296472d5fb98adf4513b5861d76d4d'
STREAM_SHA256 = '60fb2b4b475ed3eacecfa84e569d3d736bbe4f4ec61693fe490ff604227c192f'
STREAM_BYTES = 52720
STREAM_PATH = Path(__file__).with_name('ota_stream.bin')


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def xor(left, right):
    if len(left) != len(right):
        raise ValueError('XOR input lengths differ')
    return bytes(a ^ b for a, b in zip(left, right))


def parse_ota(data):
    if len(data) < 56:
        raise ValueError('Truncated Zigbee OTA header')
    magic, version, size, flags, manufacturer, kind, revision, stack, label, total = struct.unpack_from('<IHHHHHIH32sI', data)
    if (magic, version, size, flags, manufacturer, kind) != (0x0beef11e, 0x100, 56, 0, 0x115f, 0x1400) or total != len(data):
        raise ValueError('Unexpected or inconsistent Zigbee OTA header')
    parts = []
    offset = size
    while offset < total:
        if total - offset < 6:
            raise ValueError('Truncated subelement header')
        tag, length = struct.unpack_from('<HI', data, offset)
        offset += 6
        if length > total - offset:
            raise ValueError('Truncated subelement body')
        parts.append((tag, offset, data[offset:offset + length]))
        offset += length
    return revision, parts


def build_package(stock, image, revision, stream):
    if sha256(stock) != STOCK_SHA256:
        raise ValueError('Stock donor SHA-256 differs: use the documented stock 2848 file')
    if len(stream) != STREAM_BYTES or sha256(stream) != STREAM_SHA256:
        raise ValueError('Bundled obfuscation stream is missing or modified')
    if not isinstance(revision, int) or isinstance(revision, bool) or not 29 <= revision <= 255:
        raise ValueError('Controller revision must be an integer from 29 to 255')
    if not 192 <= len(image) <= STREAM_BYTES or len(image) % 4:
        raise ValueError('Application must contain 192..52720 bytes, aligned to four bytes')
    sp, entry = struct.unpack_from('<II', image)
    if sp % 8 or not 0x20000008 <= sp <= 0x20007000 or not entry & 1 or not 0x08002000 <= (entry & ~1) < 0x08002000 + len(image):
        raise ValueError('Application vectors do not fit the CIU32 application region')
    old_version, parts = parse_ota(stock)
    if old_version != 0x1c30 or [tag for tag, _, _ in parts] != [0, 0xf007, 0xf000]:
        raise ValueError('Unexpected stock version or subelement order')
    radio, metadata, controller = (part[2] for part in parts)
    if len(radio) != 609632 or len(metadata) != 96 or len(controller) != 128 + STREAM_BYTES:
        raise ValueError('Unexpected stock subelement extent')
    offset = parts[-1][1]
    if offset + len(controller) != len(stock):
        raise ValueError('Unexpected trailing stock data')
    crc = binascii.crc_hqx(image, 0)
    delta = bytearray(128)
    delta[32:44] = xor(struct.pack('<III', STREAM_BYTES, 0x2a79, 28), struct.pack('<III', len(image), crc, revision))
    struct.pack_into('<I', delta, 124, binascii.crc_hqx(delta[:124], 0))
    header = xor(controller[:128], delta)
    padding = (-len(image)) % 16
    body = image + b'\xff' * padding
    if len(body) > len(stream):
        raise ValueError('Padded body exceeds the recovered stream extent')
    replacement = header + xor(body, stream[:len(body)])
    patched = bytearray(metadata)
    struct.pack_into('<I', patched, 0x42, struct.unpack_from('<I', metadata, 0x42)[0] ^ (128 + STREAM_BYTES) ^ (128 + len(image)))
    struct.pack_into('<H', patched, 0x46, struct.unpack_from('<H', metadata, 0x46)[0] ^ 28 ^ revision)
    crc_delta = zlib.crc32(metadata[32:]) ^ zlib.crc32(patched[32:])
    struct.pack_into('<I', patched, 28, struct.unpack_from('<I', metadata, 28)[0] ^ crc_delta)
    # Preserve unknown +0x4d: only the known selector/CRC patch has live evidence.
    result = bytearray(stock[:offset - 6] + struct.pack('<HI', 0xf000, len(replacement)) + replacement)
    start = parts[1][1]
    result[start:start + len(metadata)] = patched
    version = revision << 8 | 48
    struct.pack_into('<I', result, 14, version)
    struct.pack_into('<I', result, 52, len(result))
    result = bytes(result)
    new_version, checked = parse_ota(result)
    allowed = set(range(28, 32)) | set(range(0x42, 0x48))
    if (new_version != version or checked[0] != parts[0] or checked[1][2] != patched or
            any(patched[i] != metadata[i] for i in range(96) if i not in allowed) or
            xor(checked[-1][2][128:], stream[:len(body)]) != body):
        raise ValueError('Package failed unchanged-radio/metadata/application verification')
    return result, {'controller_revision': revision, 'radio_revision': 48,
                    'file_version': version, 'file_version_hex': hex(version),
                    'application_bytes': len(image), 'application_sha256': sha256(image),
                    'application_crc16_xmodem': hex(crc), 'padding_bytes': padding,
                    'radio_payload_unchanged': True, 'unknown_metadata_retained': True,
                    'stock_sha256': sha256(stock), 'stream_sha256': sha256(stream),
                    'ota_bytes': len(result), 'ota_sha256': sha256(result)}


def write_new_or_identical(path, data):
    try:
        with path.open('xb') as output:
            output.write(data)
    except FileExistsError:
        if path.read_bytes() != data:
            raise FileExistsError('Refusing to overwrite a different file: ' + str(path))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stock', required=True, type=Path)
    parser.add_argument('--image', required=True, type=Path)
    parser.add_argument('--revision', required=True, type=lambda value: int(value, 0))
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    manifest = output.with_suffix(output.suffix + '.json')
    inputs = {args.stock.resolve(), args.image.resolve(), STREAM_PATH.resolve()}
    if output in inputs or manifest in inputs:
        parser.error('Output must not replace an input')
    package, facts = build_package(args.stock.read_bytes(), args.image.read_bytes(), args.revision, STREAM_PATH.read_bytes())
    report = (json.dumps(facts, indent=2) + '\n').encode()
    for path, data in ((output, package), (manifest, report)):
        if path.exists() and path.read_bytes() != data:
            raise FileExistsError('Refusing to overwrite a different file: ' + str(path))
    output.parent.mkdir(parents=True, exist_ok=True)
    write_new_or_identical(output, package)
    write_new_or_identical(manifest, report)
    print(report.decode(), end='')


if __name__ == '__main__':
    main()
