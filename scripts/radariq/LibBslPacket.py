"""
Common serial communication packet format for Beta Solutions projects.

@copyright Beta Solutions 2019
"""

import radariq.LibCrc as LibCrc
from radariq.compatability import pack, unpack, int_to_bytes

LIB_BSL_PACKET_HEAD = 0xB0
LIB_BSL_PACKET_FOOT = 0xB1
LIB_BSL_PACKET_ESC = 0xB2
LIB_BSL_PACKET_XOR = 0x04
LIB_BSL_PACKET_HEAD_BYTES = int_to_bytes(LIB_BSL_PACKET_HEAD)
LIB_BSL_PACKET_FOOT_BYTES = int_to_bytes(LIB_BSL_PACKET_FOOT)
LIB_BSL_PACKET_ESC_BYTES = int_to_bytes(LIB_BSL_PACKET_ESC)
LIB_BSL_PACKET_XOR_BYTES = int_to_bytes(LIB_BSL_PACKET_XOR)


def decode(src):
    """
    Decodes data from src with BSLs packet structure.
    Expects header and footer bytes at the start and end of packet respectively.
    Unescapes any header, footer or escape bytes within the packet
    warning: Does not support messages over 255.

    :param src: Data to decode
    :type src: bytes
    :return: decoded data
    :rtype: bytes
    """
    if src[0:1] != LIB_BSL_PACKET_HEAD_BYTES:
        raise Exception("First byte of the message to decode is not a header byte")
    if src[-1:] != LIB_BSL_PACKET_FOOT_BYTES:
        raise Exception("Last byte of the message to decode is not a footer byte")
    dest = b""
    srcIdx = 0

    # Loop through all bytes except footer
    while srcIdx < len(src) - 1:
        char = src[srcIdx:srcIdx + 1]
        if char == LIB_BSL_PACKET_HEAD_BYTES:
            pass

        elif char == LIB_BSL_PACKET_ESC_BYTES:
            srcIdx += 1  # Advance to the byte after the escape character
            char = src[srcIdx:srcIdx + 1]
            c = unpack("<B", char)[0]  # convert byts to int
            dest += int_to_bytes(c ^ LIB_BSL_PACKET_XOR)

        else:
            dest += char

        srcIdx += 1

    # Crc check
    data = dest[0: -2]
    crc = LibCrc.crc16_ccitt(data)
    try:
        rx_crc = unpack("<H", dest[-2:])[0]
    except Exception:
        as_hex = ''.join(format(x, '02x') for x in src)
        raise Exception("Failed to extract CRC: {}".format(as_hex))
    if crc != rx_crc:
        as_hex = ''.join(format(x, '02x') for x in src)
        raise Exception("CRC Fail: {}".format(as_hex))
    else:
        return data


def encode(src):
    """
    Encodes data from src with BSLs packet structure.
    Adds header and footer to bytes to start and end of packet respectively. Escapes any header, footer or escape bytes.
    Does not support messages over 255.

    :param src Data needing to be encoded
    :type src: bytes
    :return: Encoded packet
    :rtype: bytes
    """

    # Add CRC to the source string (so it can be encoded)
    crc = LibCrc.crc16_ccitt(src)
    src += pack("<H", crc)
    srcIdx = 0

    # Add packet header
    dest = LIB_BSL_PACKET_HEAD_BYTES
    # Loop through data, check for footer bytes in data and escape them
    while srcIdx <= len(src):
        char = src[srcIdx:srcIdx + 1]
        if char == LIB_BSL_PACKET_HEAD_BYTES:
            dest += LIB_BSL_PACKET_ESC_BYTES
            dest += int_to_bytes(LIB_BSL_PACKET_HEAD ^ LIB_BSL_PACKET_XOR)

        elif char == LIB_BSL_PACKET_FOOT_BYTES:
            dest += LIB_BSL_PACKET_ESC_BYTES
            dest += int_to_bytes(LIB_BSL_PACKET_FOOT ^ LIB_BSL_PACKET_XOR)

        elif char == LIB_BSL_PACKET_ESC_BYTES:
            dest += LIB_BSL_PACKET_ESC_BYTES
            dest += int_to_bytes(LIB_BSL_PACKET_ESC ^ LIB_BSL_PACKET_XOR)

        else:
            dest += char
        srcIdx += 1

    # Add the packet footer
    dest += LIB_BSL_PACKET_FOOT_BYTES

    if len(dest) > 255:
        raise Exception("Encoded packet is greater than the maximum of 255 bytes")

    return dest
