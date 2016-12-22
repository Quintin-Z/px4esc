#!/usr/bin/env python3

import sys
import px4esc_data_tools as dt

try:
    input_file = sys.argv[1]
except IndexError:
    input_file = None
    dt.print_stderr('Usage:', sys.argv[0], 'path-to-log-file', '> output.csv')
    exit(1)


def make_generator():
    return dt.iter_trace_samples(input_file)

# Scanning the file once in order to build the list of keys
dt.print_stderr('Looking for keys in %r...' % input_file)

keys = set()
num_lines = 0

for sample in make_generator():
    keys |= set(sample.keys())
    num_lines += 1
    if num_lines % 100000 == 0:
        dt.print_stderr('\r%.0fK lines parsed...  \r' % (num_lines / 1e3), end='')
        sys.stderr.flush()

dt.print_stderr('Keys:', list(keys), 'Lines:', num_lines)

# Second pass, extracting the keys found earlier
keys = list(sorted(keys))
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


for line_index, sample in enumerate(make_generator()):
    print(','.join(render_values(sample)))
    if line_index % 100000 == 0:
        dt.print_stderr('\r%.0f %%  \r' % (100 * (line_index + 1) / num_lines), end='')
        sys.stderr.flush()
