"""
CRC Algorithm as implemented in C

@copyright Beta Solutions 2019
"""
import six

def crc16_ccitt(data):
    """
    CRC CCITT 0xFFFF algorithm
    :param data: data to calculate the crc for
    :type data: bytes
    """
    crc = 0xffff
    msb = crc >> 8
    lsb = crc & 255
    for c in six.iterbytes(data):
        x = c ^ msb
        x ^= (x >> 4)
        msb = (lsb ^ (x >> 3) ^ (x << 4)) & 255
        lsb = (x ^ (x << 5)) & 255
    return (lsb << 8) + msb
