#!/usr/bin/env python3
#
# Decoder of real-time variable tracing dumps.
#
# Copyright (c) 2016-2017  Pavel Kirienko  <pavel.kirienko@zubax.com>
#

import numpy as np
from logging import getLogger
from .codec import Fields, decode_sample


TIMESTAMP_VARIABLE_NAME = 'time'

logger = getLogger(__name__)


class Decoder:
    """
    Trace files require a stateful parser.
    """
    DEFAULT_VARIABLE_SIZE_TO_FIELD_TYPE_MAP = {
        1: Fields.INT8,
        2: Fields.INT16,
        4: Fields.FLOAT32,
        8: Fields.FLOAT64
    }

    def __init__(self, size_to_type_map=None, big_endian=False):
        self._format = None
        self._names = None
        self._big_endian = big_endian

        self._size_to_type_map = self.DEFAULT_VARIABLE_SIZE_TO_FIELD_TYPE_MAP
        if size_to_type_map:
            self._size_to_type_map.update(size_to_type_map)

    def process_line_strict(self, line):
        if isinstance(line, bytes):
            line = line.decode('utf8', errors='ignore')

        line = line.strip()

        if ('~\t' in line) and ('/' in line) and (len(line) >= 5):  # Format declaration; best effort if damaged
            line = line.split('~\t')[-1]
            declarations = [x.split('/') for x in line.split()]

            self._names = [x[0] for x in declarations]
            self._format = ''.join(self._size_to_type_map[int(x[1])] for x in declarations)

            logger.info('Decoder: Variables: %r; format: %r', self._names, self._format)
            return

        if not self._format:
            return

        # The previous line may be terminated with plain CR, we need to handle that correctly
        line = line.split()[-1]

        if line.startswith('~'):
            # Recovering the latest sample if multiple lines were erroneously joined together
            line = line.split('~')[-1]

            variables = decode_sample(self._format, line, big_endian=self._big_endian)
            assert len(variables) == len(self._names)

            return dict(zip(self._names, variables))

    def process_line(self, line):
        # noinspection PyBroadException
        try:
            return self.process_line_strict(line)
        except Exception:
            logger.error('Decoder: Failed to process line: %r', line, exc_info=True)


def iter_samples_in_file(path, **decoder_args):
    """
    Loads real-time tracing data from text file. Each line may contain either arbitrary text (which is ignored) or
    encoded data sample.
    :param path:                path to the data file
    :param decoder_args:        extra arguments that will be passed into the Decoder constructor
    :return:                    generator of samples, where each sample is a dict of the form {variable: value}
    """
    decoder = Decoder(**decoder_args)

    with open(path, 'rb') as f:
        for line_index, ln in enumerate(f):
            output = decoder.process_line(ln)
            if output:
                yield output


def list_variable_names_in_file(path):
    """
    Quickly scans the file and builds a list of all variables that are present in it.
    :param path:
    :return:
    """
    ret = set()

    with open(path, 'rb') as f:
        for ln in f:
            if len(ln) < 5:              # Minimal declaration: '~\tX/0'
                continue
            if b'~\t' not in ln:
                continue

            ln = ln.split(b'~\t')[-1]    # Splitting handles the corner case when the previous line was not terminated
            ln = ln.decode('utf8')

            ret |= set(d.split('/')[0] for d in ln.split())

    # Sorting so that the 'time' variable, if present, gets to the front
    return list(sorted(ret, key=lambda x: ' ' if x == TIMESTAMP_VARIABLE_NAME else x))


def iter_matrices(samples, variables):
    """
    This function accepts an iterable of samples and the list of variables of interest, and returns a generator of
    matrices of size SxV, where S is the number of samples in the sequence, and V is the number of variables.
    It is advised to always keep 'time' as the first variable, for compatibility reasons.
    :param samples:     an iterable of tracing sample; each sample is a dict {name: value}
    :param variables:   list of variable names; it is advised to always use 'time' as the first variable
    :return:            an iterable of NumPy matrices
    """
    if len(variables) < 1:
        raise ValueError('Variable set cannot be empty')

    raise NotImplementedError('Fix me')
