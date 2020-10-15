"""
Wrappers for python 2 & 3 compatibility
"""

import struct
import six

if six.PY2:
    import Queue as queue
else:
    import queue


def pack(fmt, *args):
    if six.PY2:
        # print(struct.pack(fmt, *args))
        return struct.pack(fmt, *args)
    else:
        return struct.pack(fmt, *args)


def unpack(fmt, value):
    if not isinstance(value, bytes):
        value = value.encode('utf8')
    return struct.unpack(fmt, value)


def as_hex(msg):
    """
    Print a string as hex values
    :param msg:
    :return:
    """
    if six.PY2:
        return " ".join("{:02x}".format(ord(c)) for c in msg)
    else:
        return msg.hex()


def int_to_bytes(num): # should work cross platform
    return bytes(bytearray([num]))

def bc_to_int(inp):
    if six.PY2:
        return ord(inp)
    else:
        return inp[0]