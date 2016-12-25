#!/usr/bin/env python3

import sys
import variable_trace_decoder as vtd

try:
    input_file = sys.argv[1]
except IndexError:
    input_file = None
    print('Usage:', sys.argv[0], 'path-to-log-file', '> output.csv', file=sys.stderr)
    exit(1)

keys = vtd.list_variable_names_in_file(input_file)
print('Variables:', list(keys), file=sys.stderr)
print(','.join(keys))


def render_values(sam):
    for k in keys:
        try:
            value = sam[k]
        except KeyError:
            value = None

        if value is None:
            yield ''
        elif isinstance(value, float):
            # noinspection PyStringFormat
            yield '%.9f' % value
        else:
            yield str(value)


num_samples = 0
for sample in vtd.iter_samples_in_file(input_file):
    print(','.join(render_values(sample)))
    num_samples += 1

print('Number of samples:', num_samples, file=sys.stderr)
