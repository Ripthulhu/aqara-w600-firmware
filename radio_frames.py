"""Offline W600 CIU32 UART frame codec. Never opens a serial port.

Derived from flash SHA256 6320104be1376b3525e1eb6c229f9a1c89668d41d088e8dddbdcf4171fdbb8f0.
Serializer: 0x08007464; RX parser: 0x08007f08; checksum: 0x08007ae0;
CRC nibble table/code: 0x0800ead8 / 0x08007aa8. Checked against nine live RX
frames in live-20261106/active-rx-1794004541-rx.bin; no TX/ACK tested.
Kinds are preserved as bytes: MCU RX accepts 1/2/3/7, but MCU TX also uses 4/5.
"""

from dataclasses import dataclass

MAGIC = b"\x55\x05\x02"
MAX_PAYLOAD = 256


@dataclass(frozen=True)
class Frame:
    sequence: int
    kind: int
    payload: bytes


def crc16(data: bytes) -> int:
    """Reflected 0xA001, initial 0xFFFF, no final XOR."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc


def encode(frame: Frame) -> bytes:
    if not 0 <= frame.sequence <= 255 or not 0 <= frame.kind <= 255:
        raise ValueError("sequence and kind must be bytes")
    if not isinstance(frame.payload, bytes) or len(frame.payload) > MAX_PAYLOAD:
        raise ValueError("payload must be bytes, at most 256 bytes")
    header = MAGIC + bytes((frame.sequence, frame.kind)) + len(frame.payload).to_bytes(2, "big")
    body = header + bytes((-sum(header) & 255,)) + frame.payload
    return body + crc16(body).to_bytes(2, "big")


def decode(data: bytes) -> Frame:
    if len(data) < 10 or data[:3] != MAGIC:
        raise ValueError("not a complete W600 frame")
    length = int.from_bytes(data[5:7], "big")
    if length > MAX_PAYLOAD or len(data) != length + 10:
        raise ValueError("invalid payload length")
    if sum(data[:8]) & 255:
        raise ValueError("invalid header checksum")
    if crc16(data[:-2]) != int.from_bytes(data[-2:], "big"):
        raise ValueError("invalid frame CRC")
    return Frame(data[3], data[4], data[8:-2])


def _self_test() -> None:
    assert crc16(b"123456789") == 0x4B37
    captured = [
        ("550502510200044d071400007804", Frame(0x51, 2, bytes.fromhex("07140000"))),
        ("550502520200044c071800018b2c", Frame(0x52, 2, bytes.fromhex("07180001"))),
        ("550502530200044b071800001b55", Frame(0x53, 2, bytes.fromhex("07180000"))),
    ]
    for wire, expected in captured:
        packet = bytes.fromhex(wire)
        assert decode(packet) == expected
        assert encode(expected) == packet
    for payload in (b"", b"\x07\x09", bytes(range(256))):
        f = Frame(1, 1, payload)
        packet = encode(f)
        assert sum(packet[:8]) & 255 == 0
        assert len(packet) == len(payload) + 10
        assert decode(packet) == f
    good = encode(Frame(1, 2, b"\x07\x09\x00\x01"))
    bad_header = bytearray(good)
    bad_header[7] ^= 1
    bad_payload = bytearray(good)
    bad_payload[8] ^= 1
    for packet in (b"", good[:-1], good + b"\x00", bytes(bad_header), bytes(bad_payload)):
        try:
            decode(packet)
        except ValueError:
            pass
        else:
            raise AssertionError("malformed packet accepted")
    try:
        encode(Frame(1, 1, bytes(257)))
    except ValueError:
        pass
    else:
        raise AssertionError("oversized payload accepted")


if __name__ == "__main__":
    _self_test()
    print("Offline frame codec checks passed; no hardware accessed.")
