#!/usr/bin/env python3
#
# Collection of useful functions for working with PX4ESC data dumps.
#

import os
import sys
import json
import numpy
import math
import matplotlib.pyplot as plt
from logging import getLogger

logger = getLogger(__name__)


def amap(f, x):
    return numpy.array(list(map(f, x)))


def difference_list(cont):
    out = []
    prev_x = cont[0]
    for x in cont:
        out.append(x - prev_x)
        prev_x = x
    return numpy.array(out)


def running_mean(x, n):
    if n % 2 == 0:
        n += 1
    augmentation = (min(len(x), n) - 1) // 2
    return numpy.convolve(numpy.concatenate(([x[0]] * augmentation,
                                             x,
                                             [x[-1]] * augmentation)),
                          numpy.ones((n,)) / n, mode='valid')


def find_index(fn, container):
    for i, x in enumerate(container):
        if fn(x):
            return i


def iter_trace_samples(data):
    """
    Loads real-time tracing data from text file. Each line may contain either comments or encoded data sample.
    If the line starts with '{' (left curly brace), it is assumed to contain encoded sample, otherwise it will be
    ignored. The samples are encoded in YAML or Python literal dict format (for this purpose, these two formats are
    identical and interchangeable).
    :param data:                path to the data file
    :return:                    generator of samples, where each sample is a dict of the form {variable: value}
    """
    # TODO: add support for packed files (raw Base85 encoded traces)
    with open(data, encoding='utf8') as f:
        for line_index, ln in enumerate(f):
            if len(ln) < 1 or ln[0] != '{':
                continue
            try:
                # We used to use this approach:
                # sample = ast.literal_eval(ln)
                # but it turned out to be 5 times slower than JSON. YAML is about 12 times slower than JSON!
                # Speed matters because the trace files can be several gigabytes large!
                sample = json.loads(ln.replace("'", '"'))
                if not isinstance(sample, dict):
                    raise ValueError('Sample is not dict')
                if 'time' not in sample:
                    raise ValueError('Sample has no timestamp')
                yield sample
            except Exception as ex:
                raise ValueError('Parsing error at line %d: %r' % (line_index + 1, ln)) from ex


def iter_contiguous_sequences_where_all_variables_are_available(samples, variables):
    var_set = set(variables)
    if 'time' in var_set:
        raise ValueError('Time trace cannot be used in this context')
    if len(var_set) < 1:
        raise ValueError('Variable set cannot be empty')

    current_sequence = []
    for s in samples:
        if var_set.issubset(set(s.keys())):
            current_sequence.append(s)
        else:
            if len(current_sequence) > 0:
                yield current_sequence
            current_sequence = []


def transform_trace_samples(samples) -> dict:
    """
    This function rearranges the data and returns a dict of sample sets, where each sample set contains an N-by-2
    matrix, where N is the number of samples in the set; the first column contains timestamps, and the second column
    contains data points. Consider the example:

        'Iq': array([[  1.03063251e+04,   5.84651232e+00],
                     [  1.03063253e+04,   5.19571972e+00],
                     [  1.03063255e+04,   5.15734243e+00], ...

    Additionally, the returned dict has a special key 'time', which contains only time samples in the form of a
    column vector. Example:

        'time': array([[ 10258.26232085],
                       [ 10258.26260085],
                       [ 10258.26282085], ...
    """
    samples = list(samples)
    return {
        field: amap(lambda x: numpy.array([float(x['time']), x[field]]
                                          if field != 'time' else
                                          [float(x['time'])]),
                    filter(lambda x: field in x,
                           samples))
        for field in set(x
                         for it in samples
                         for x in it.keys())
    }


def rescale_trace(trace, scale):
    """
    Helper function for easy rescaling of traces produced by iter_trace_samples(). Usage:
    >>> rescale_trace(trace['phi'], 1e3)
    This function preserves shape of the input data. Obviously, rescaling does not affect the timestamp information.
    """
    trace = numpy.array(trace)
    if trace.ndim == 1:
        return trace * scale

    if trace.ndim == 2:
        if 1 in trace.shape:
            return trace * scale
        if 2 not in trace.shape:
            raise ValueError('Invalid shape: %r' % trace.shape)
        if trace.shape[0] == 2:
            return numpy.array((trace[0], trace[1] * scale))
        else:
            trace = trace.T
            return numpy.array((trace[0], trace[1] * scale)).T

    raise ValueError('Invalid number of dimensions: %r; shape: %r' % (trace.ndim, trace.shape))


def normalize_angle(x):
    pi2 = math.pi * 2
    if x >= pi2:
        return x - pi2
    if x < 0:
        return x + pi2
    return x


def plot(left_plots,
         right_plots=None,
         colors='bgrcmykw',
         linewidth=0.5,
         title=None,
         x_label=None,
         y_labels=None,
         save_to_file=None):
    """
    A convenience wrapper over matplotlib's pyplot features.
    Supported data formats for each element of the lists left_plots and right_plots:
     - [[x...], [y...]]
     - [[x, y], [x, y], ...]
     - [y...]
    If X values are not provided, indexes of the Y values will be used instead.
    """
    if right_plots is None:
        right_plots = []

    if y_labels is None:
        y_labels = []
    if len(y_labels) not in (0, 1, 2):
        raise ValueError('Invalid set of Y labels: %r' % y_labels)

    fig = plt.figure()
    if title:
        fig.suptitle(title, fontweight='bold')

    ax1 = fig.add_subplot(111)
    if len(left_plots) > len(colors) or len(right_plots) > len(colors):
        raise ValueError('Too many plots; make sure the Y axis data are wrapped into an iterable container')

    def extract_xy(data):
        data = numpy.array(data)
        if data.ndim == 1:
            return list(range(len(data))), data

        if data.ndim == 2:
            if 1 in data.shape:
                data = data.flatten()
                return range(len(data)), data

            if 2 not in data.shape:
                raise ValueError("Don't know how to plot data of shape %r" % data.shape)

            if data.shape[0] != 2:
                data = data.T

            assert data.shape[0] == 2
            if data.shape[1] <= 2:
                raise ValueError('Data dimension too small, make sure the shape is correct: %r' % data.shape)

            assert data.shape[0] == 2 and data.shape[1] > 2
            return data[0], data[1]

        raise ValueError('Only one or two dimensional data is supported; got %r' % data.shape)

    color_selector = iter(colors)
    for i, p in enumerate(left_plots):
        ax1.plot(*extract_xy(p), linewidth=linewidth, color=next(color_selector), label=str(i))

    ax1.legend(loc='upper left')
    ax1.ticklabel_format(useOffset=False)

    if x_label:
        ax1.set_xlabel(x_label)

    if len(y_labels) > 0:
        ax1.set_ylabel(y_labels[0])

    if len(right_plots):
        color_selector = iter(colors)
        ax2 = ax1.twinx()
        for i, p in enumerate(right_plots):
            ax2.plot(*extract_xy(p), '--', linewidth=linewidth, color=next(color_selector), label=str(i))

        ax2.legend(loc='upper right')
        ax2.set_yticks(numpy.linspace(ax2.get_yticks()[0], ax2.get_yticks()[-1], len(ax1.get_yticks())))
        ax2.grid(None)
        ax2.ticklabel_format(useOffset=False)
        if len(y_labels) == 2:
            ax2.set_ylabel(y_labels[1])
    else:
        if len(y_labels) >= 2:
            raise ValueError('Unused Y label for the right side Y axis')

    fig.tight_layout()
    if save_to_file is not None:
        if not isinstance(save_to_file, str):
            save_to_file = str(save_to_file)
            if '.' in save_to_file:
                save_to_file += '.png'

        fig.savefig(save_to_file)
    else:
        fig.show()


def constrainer(low, high):
    if low >= high:
        raise ValueError('Invalid range constraint: %r is not less than %r' % (low, high))
    return lambda x: min(max(x, low), high)


def vector(*x):
    """Constructs a column vector"""
    return numpy.matrix(amap(float, x)).T


# noinspection PyPep8Naming
def List(*x):
    """Helper for Mathematica generated code"""
    if all(map(lambda x: hasattr(x, '__iter__'), x)):
        return numpy.matrix(x)
    return amap(float, x)


def plot_kalman_filter_states(time_stamps, filter_states, state_indexes, scale=1, *args, **kwargs):
    """Plots the selected states and the corresponding variance from an arbitrary Kalman filter state history"""
    assert len(time_stamps) == len(filter_states)
    state = [amap(lambda x: x.x[i, 0] * scale, filter_states) for i in state_indexes]
    stdev = [amap(lambda x: math.sqrt(x.P[i, i]), filter_states) for i in state_indexes]
    plot([(time_stamps, x) for x in state],
         [(time_stamps, x) for x in stdev],
         x_label='Time [s]',
         y_labels=['State', 'Stdev'],
         *args, **kwargs)


def print_stderr(*args, **kwargs):
    kwargs['file'] = sys.stderr
    print(*args, **kwargs)


if __name__ == '__main__':
    decoded = transform_trace_samples(iter_trace_samples(os.path.expanduser('~/latest_data.log')))
    print(decoded.keys())
    for k, v in decoded.items():
        print(k, v.shape)
    print()
    print(decoded)

    vars = ['Ud', 'Uq', 'AVel']
    print('Contiguous sequences for variables:', vars)
    for seq in iter_contiguous_sequences_where_all_variables_are_available(
            iter_trace_samples(os.path.expanduser('~/latest_data.log')),
            vars):
        print(seq[0]['time'], seq[-1]['time'], len(seq))

    plot([decoded['Ud']], [decoded['AVel']], save_to_file='test')

    class KalmanStub:
        def __init__(self, x):
            self.x = numpy.matrix([[x]])
            self.P = numpy.matrix([[x]])

    plot_kalman_filter_states([0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
                              [KalmanStub(x) for x in (1, 3, 2, 4, 0, 5)],
                              [0],
                              save_to_file='kalman_test')
