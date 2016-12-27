#!/usr/bin/env python3
#
# Collection of useful functions for working with PX4ESC data dumps.
#

import numpy as np
import math
import typing
import copy
from logging import getLogger


logger = getLogger(__name__)


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


def downsample(source, ratio):
    """
    :param source:  an iterable, possibly a generator, with the input numerical data, scalar or vector
    :param ratio:   downsampling ratio, must be larger than or equal 1; non-integers are allowed
    :return:        a generator that yields averaged downsampled values
    """
    if ratio < 1:
        raise ValueError('Invalid downsampling ratio %r' % ratio)

    # It is possible to use numpy for this, but we'd like to support generators properly
    # See this: http://stackoverflow.com/questions/20322079/downsample-a-1d-numpy-array
    # Also the current approach supports non-integer ratios
    a, n = None, 0
    ratio_counter = 0

    for s in source:
        if a is None:
            a, n = copy.deepcopy(s), 1
        else:
            a += s
            n += 1

        ratio_counter += 1
        if ratio_counter >= ratio:
            ratio_counter -= ratio
            yield a / n
            a, n = None, 0

    if a is not None and n > 0:
        yield a / n             # Trailing residuals


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


class Plot:
    def __init__(self,
                 subplots: typing.Union[int, tuple]=None,
                 title: str=None):
        import matplotlib.pyplot
        self._fig = matplotlib.pyplot.figure()

        if subplots is None:
            subplots = 1, 1
        if isinstance(subplots, int):
            subplots = subplots, 1

        self._subplots = []
        # noinspection PyTypeChecker
        for num in range(subplots[0] * subplots[1]):
            self._subplots.append(self._fig.add_subplot(*subplots, num + 1))

        self._next_subplot_index = 0

        if title:
            # Ugly hack; see this: http://stackoverflow.com/questions/8248467
            self._tight_layout_rect = 0, 0, 1, 0.95
            self._fig.suptitle(title, fontsize='x-large')
        else:
            self._tight_layout_rect = 0, 0, 1, 1

    @staticmethod
    def _extract_xy(data):
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

    @staticmethod
    def _separate_data_sequences_and_names(data_sequences_and_names):
        data, names = [], []
        for item in data_sequences_and_names:
            if isinstance(item, (str, int, float)):
                names.append(str(item))
            elif isinstance(item, bytes):
                raise ValueError('Cannot plot %r' % type(item))
            else:
                data.append(item)

        while len(names) < len(data):
            names.append(None)

        if len(names) > len(data):
            raise ValueError('Too many names: %r names, %r data sequences' % (len(names), len(data)))

        assert len(data) == len(names)
        return data, names

    def plot(self,
             *data_sequences_and_names,
             subplot_index=None,
             title=None,
             x_label=None,
             y_label=None,
             colors=None,
             line_width=None):
        data_sequences, names = self._separate_data_sequences_and_names(data_sequences_and_names)
        colors = 'bgrcmyk' if colors is None else colors
        line_width = 0.5 if line_width is None else float(line_width)

        if len(data_sequences) < 1:
            raise ValueError('The set of the left side data sequences cannot be empty')
        elif len(data_sequences) > len(colors):
            raise ValueError('Too many data sequences')

        if subplot_index is None:
            subplot_index = self._next_subplot_index
            self._next_subplot_index += 1

        if not (0 <= subplot_index < len(self._subplots)):
            raise IndexError('Subplot index out of range: 0 <= %r < %r' % (subplot_index, len(self._subplots)))

        subplot = self._subplots[subplot_index]
        for i, (seq, col, nm) in enumerate(zip(data_sequences, colors, names)):
            subplot.plot(*self._extract_xy(seq),
                         linewidth=float(line_width),
                         color=str(col),
                         label=str(nm if nm is not None else i))

        if len(data_sequences) > 1 or names[0] is not None:
            subplot.legend(loc='upper left')

        subplot.ticklabel_format(useOffset=False)

        if title:
            subplot.set_title(title)

        if x_label:
            subplot.set_xlabel(x_label)

        if y_label:
            subplot.set_ylabel(y_label)

        right_default_colors = colors

        # noinspection PyShadowingNames
        def plot_right(*data_sequences_and_names,
                       y_label=None,
                       colors=None):
            data_sequences, names = self._separate_data_sequences_and_names(data_sequences_and_names)
            colors = right_default_colors if colors is None else colors

            if len(data_sequences) < 1:
                return
            elif len(data_sequences) > len(colors):
                raise ValueError('Too many data sequences')

            subplot_right = subplot.twinx()
            for i, (seq, col, nm) in enumerate(zip(data_sequences, colors, names)):
                subplot_right.plot(*self._extract_xy(seq),
                                   '--',
                                   linewidth=float(line_width),
                                   color=str(col),
                                   label=str(nm if nm is not None else i))

            if len(data_sequences) > 1 or names[0] is not None:
                subplot_right.legend(loc='upper right')

            subplot_right.set_yticks(np.linspace(subplot_right.get_yticks()[0],
                                                 subplot_right.get_yticks()[-1],
                                                 len(subplot.get_yticks())))
            subplot_right.grid(None)
            subplot_right.ticklabel_format(useOffset=False)
            if y_label:
                subplot_right.set_ylabel(y_label)

        return plot_right

    def _finalize(self, size_px, dpi):
        dpi = 100 if dpi is None else dpi       # Larger DPI --> Larger fonts
        if size_px is not None:
            self._fig.set_dpi(dpi)
            self._fig.set_size_inches(size_px[0] / dpi,
                                      size_px[1] / dpi)

        # noinspection PyBroadException
        try:
            # Matplotlib sometimes mysteriously fails to perform tight_layout(), e.g. like:
            # ValueError: left cannot be >= right
            # See this: http://stackoverflow.com/questions/22708888
            self._fig.tight_layout(rect=self._tight_layout_rect)
        except Exception:
            logger.error('Plot: tight_layout() failed', exc_info=True)

    def save(self, name, size_px=None, dpi=None):
        self._finalize(size_px, dpi)
        if not isinstance(name, str):
            name = str(name)
            if '.' in name:
                name += '.png'

        self._fig.savefig(name, dpi='figure')

    def show(self, size_px=None, dpi=None):
        self._finalize(size_px, dpi)
        self._fig.show()


def plot(*data_sequences_and_names,
         title=None,
         x_label=None,
         y_label=None):
    """
    A simple wrapper over Plot suitable for plotting images without multiple subplots.
    Usage:
    >>> data_left = [1, 2, 3, 4, 5]
    >>> plot(data_left)('my_data')                          # The plot will be saved into 'my_data.png'
    >>> plot(data_left, 'my sequence')()                    # A window will appear; plot legend will be added
    >>> data_right = [-4, -7, -12]
    >>> plot('left data', data_left, 'right data', data_right)(size_px=(640, 480))
    >>> plot(data_left)(data_right)('my_data_combined')
    """
    p = Plot(title=title)

    def output(name=None,
               size_px=None):
        if name is None:
            p.show(size_px=size_px)
        else:
            p.save(name, size_px=size_px)

    right = p.plot(*data_sequences_and_names, x_label=x_label, y_label=y_label)

    # noinspection PyShadowingNames
    def plot_right(*data_sequences_and_names,
                   y_label=None,
                   size_px=None):
        if len(data_sequences_and_names) == 0:
            assert y_label is None
            output(size_px=size_px)
        elif len(data_sequences_and_names) == 1 and isinstance(data_sequences_and_names[0], str):
            assert y_label is None
            output(data_sequences_and_names[0], size_px=size_px)
        else:
            right(*data_sequences_and_names, y_label=y_label)
            return output

    return plot_right


def plot_kalman_filter_states(plot_instance: Plot,
                              time_stamps,
                              filter_states,
                              state_indexes,
                              state_names=None,
                              scale=None,
                              title=None):
    """
    Plots the selected states and the corresponding variance from an arbitrary Kalman filter state history.
    """
    assert len(time_stamps) == len(filter_states)
    state_names = [] if state_names is None else state_names
    scale = 1 if scale is None else float(scale)
    state = [amap(lambda x: x.x[i, 0] * scale, filter_states) for i in state_indexes]
    stdev = [amap(lambda x: math.sqrt(x.P[i, i]) * scale, filter_states) for i in state_indexes]
    plot_instance.plot(*state, *state_names, title=title, x_label='Time [s]', y_label='State')\
        (*stdev, *state_names, y_label='Stdev')


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


if __name__ == '__main__':
    demo_x = np.linspace(0, 10, 10000)
    demo_y = np.sin(demo_x)
    demo_x_downsampled = list(downsample(demo_x, 1000))
    demo_y_downsampled = list(downsample(demo_y, 1000))
    plot([demo_x, demo_y], [demo_x_downsampled, demo_y_downsampled], 'true', 'downsampled')('downsample')

    demo_ts = np.linspace(0, 10, 10000)
    demo_plots = np.sin(demo_ts) * 0.1, np.cos(demo_ts) * 100 - 1000

    plot('sin', np.matrix([demo_ts, demo_plots[0]]).T)('cos', np.matrix([demo_ts, demo_plots[1]]))('test', (2000, 800))

    demo_plot = Plot(subplots=2, title='Combined Plot')

    demo_plot.plot(np.matrix([demo_ts, demo_plots[0]]).T, title='Demo Plots', y_label='sin')\
        (np.matrix([demo_ts, demo_plots[1]]), y_label='cos')

    demo_plot.plot('raw', (demo_x, demo_y),
                   'downsampled', (demo_x_downsampled, demo_y_downsampled))

    demo_plot.save('combined', size_px=(1000, 1000))

    class KalmanStub:
        def __init__(self, x):
            self.x = np.matrix([[x]])
            self.P = np.matrix([[x]])

    demo_plot = Plot(title='Kalman states')
    plot_kalman_filter_states(demo_plot,
                              [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
                              [KalmanStub(x) for x in (1, 3, 2, 4, 0, 5)],
                              [0])
    demo_plot.save('kalman_test')
