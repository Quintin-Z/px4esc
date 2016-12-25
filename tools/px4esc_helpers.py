#!/usr/bin/env python3
#
# Collection of useful functions for working with PX4ESC data dumps.
#

import numpy as np
import math


def amap(f, x):
    return np.array(list(map(f, x)))


def difference_list(cont):
    out = []
    prev_x = cont[0]
    for x in cont:
        out.append(x - prev_x)
        prev_x = x
    return np.array(out)


def running_mean(x, n):
    if n % 2 == 0:
        n += 1
    augmentation = (min(len(x), n) - 1) // 2
    return np.convolve(np.concatenate(([x[0]] * augmentation,
                                       x,
                                       [x[-1]] * augmentation)),
                       np.ones((n,)) / n, mode='valid')


def find_index(fn, container):
    for i, x in enumerate(container):
        if fn(x):
            return i


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
    import matplotlib.pyplot as plt

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
        data = np.array(data)
        if data.ndim == 1:
            return list(range(len(data))), data

        if data.ndim == 2:
            if 1 in data.shape:
                data = data.flatten()
                return range(len(data)), data

            if 2 not in data.shape:
                raise ValueError("Don't know how to plot data of shape %r" % (data.shape,))

            if data.shape[0] != 2:
                data = data.T

            assert data.shape[0] == 2
            if data.shape[1] <= 2:
                raise ValueError('Data dimension too small, make sure the shape is correct: %r' % (data.shape,))

            assert data.shape[0] == 2 and data.shape[1] > 2
            return data[0], data[1]

        raise ValueError('Only one or two dimensional data is supported; got %r' % (data.shape,))

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
        ax2.set_yticks(np.linspace(ax2.get_yticks()[0], ax2.get_yticks()[-1], len(ax1.get_yticks())))
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
    return np.matrix(amap(float, x)).T


# noinspection PyPep8Naming
def List(*x):
    """Helper for Mathematica generated code"""
    if all(map(lambda x: hasattr(x, '__iter__'), x)):
        return np.matrix(x)
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


if __name__ == '__main__':
    demo_ts = np.linspace(0, 10, 10000)
    demo_plots = np.sin(demo_ts) * 0.1, np.cos(demo_ts) * 100 - 1000

    plot([np.matrix([demo_ts, demo_plots[0]]).T],
         [np.matrix([demo_ts, demo_plots[1]])],
         save_to_file='test')

    class KalmanStub:
        def __init__(self, x):
            self.x = np.matrix([[x]])
            self.P = np.matrix([[x]])

    plot_kalman_filter_states([0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
                              [KalmanStub(x) for x in (1, 3, 2, 4, 0, 5)],
                              [0],
                              save_to_file='kalman_test')
