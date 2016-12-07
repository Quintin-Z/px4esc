#!/usr/bin/env python3
#
# This module allows to decode real-time tracing samples encoded in a modified Z85 bin-to-text format.
#
# Copyright (c) 2016  Pavel Kirienko  <pavel.kirienko@zubax.com>
#

import struct


ALPHABET = b'0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ.-:+=^!/*?&<>()[]{}@%$#'
DECODE_LUT = {chr(c): idx for idx, c in enumerate(ALPHABET)}


class Fields:
    """This helper class can be used to build format strings in a human-friendly way.
    Example:
    >>> Fields.UINT64 + Fields.FLOAT64 + Fields.repeat(4).FLOAT32
    'Qd4f'
    """

    FLOAT32 = 'f'
    FLOAT64 = 'd'

    INT8 = 'b'
    INT16 = 'h'
    INT32 = 'i'
    INT64 = 'q'

    UINT8 = 'B'
    UINT16 = 'H'
    UINT32 = 'I'
    UINT64 = 'Q'

    BOOL = '?'

    CHAR = 's'

    @staticmethod
    def repeat(num):
        assert num >= 0
        c = Fields()
        for k in Fields.__dict__.keys():
            if not k.startswith('__') and k.upper() == k:
                obj = getattr(Fields, k)
                if isinstance(obj, str):
                    setattr(c, k, str(num) + obj)
        return c


def decode_sample(payload_format, text, big_endian=False):
    if not isinstance(text, str):
        text = text.decode('ascii')

    if len(text) % 5:
        raise ValueError('Encoded string has length %d which is not multiple of 5' % len(text))

    ints = [0] * (len(text) // 5)
    try:
        for i in range(0, len(text), 5):
            ints[i // 5] = DECODE_LUT[text[i + 0]] * 52200625 + \
                           DECODE_LUT[text[i + 1]] * 614125 + \
                           DECODE_LUT[text[i + 2]] * 7225 + \
                           DECODE_LUT[text[i + 3]] * 85 + \
                           DECODE_LUT[text[i + 4]]
    except KeyError as ex:
        raise ValueError('Encoded string contains invalid character %r' % ex.args[0]) from None

    endiannes_specifier = '>' if big_endian else '<'

    raw_bin = struct.pack('%s%dI' % (endiannes_specifier, len(ints)), *ints)

    full_payload_format = endiannes_specifier + payload_format
    format_size = struct.calcsize(full_payload_format)

    return struct.unpack(full_payload_format, raw_bin[:format_size])


if __name__ == '__main__':
    fmt = Fields.UINT64 + Fields.repeat(4).UINT8
    print(fmt)
    print(decode_sample(fmt, 'HelloWorld00993'))
    print(decode_sample(Fields.FLOAT64 + Fields.repeat(2).FLOAT32, '00000kWKG>l/-:$l}8Ra'))
    print(decode_sample(Fields.FLOAT64 + Fields.BOOL + Fields.repeat(3).FLOAT32, 'PvF.$kU%y+0000100000nzWLJrAi3+'))
