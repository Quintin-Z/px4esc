#!/usr/bin/env python3
#
# Pavel Kirienko, 2016 <pavel.kirienko@gmail.com>
#
# This is a quick hack that allows to plot values from serial port and at the same time have access to CLI.
# This script may be superseded with Zubax Toolbox at some point.
#

import numpy
import os
import sys
import threading
import time
import glob
import queue
import trace_decoder
from high_throughput_serial_port import SerialPort

from PyQt5.QtWidgets import QVBoxLayout, QWidget, QApplication, QMainWindow, QAction
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QKeySequence

try:
    import pyqtgraph
except ImportError:
    os.system('git clone --recursive https://github.com/pyqtgraph/pyqtgraph-core pyqtgraph')
    import pyqtgraph

from pyqtgraph import PlotWidget, mkPen, InfiniteLine

NO_GUI = '--nogui' in sys.argv
if NO_GUI:
    sys.argv.remove('--nogui')

if len(sys.argv) > 1:
    SER_PORT = sys.argv[1]
else:
    SER_PORT = (glob.glob('/dev/serial/by-id/usb-*_PX4ESC_*-if00') +
                glob.glob('/dev/serial/by-id/usb-*Black_Magic_Probe*-if02'))[0]
    print('Selected port', SER_PORT)

SER_BAUDRATE = 115200


VARIABLE_SCALING = {
    'AVel': 1e-3,
    'oAVe': 1e-3,
    'phi':  1e3,
    'IRQF': 1e6,
    'IRQM': 1e6,
}


# Borrowed from the UAVCAN GUI Tool
def add_crosshair(plot, render_measurements, color=Qt.gray):
    pen = mkPen(color=QColor(color), width=1)
    vline = InfiniteLine(angle=90, movable=False, pen=pen)
    hline = InfiniteLine(angle=0, movable=False, pen=pen)

    plot.addItem(vline, ignoreBounds=True)
    plot.addItem(hline, ignoreBounds=True)

    current_coordinates = None
    reference_coordinates = None

    def do_render():
        render_measurements(current_coordinates, reference_coordinates)

    def update(pos):
        nonlocal current_coordinates
        if plot.sceneBoundingRect().contains(pos):
            mouse_point = plot.getViewBox().mapSceneToView(pos)
            current_coordinates = mouse_point.x(), mouse_point.y()
            vline.setPos(mouse_point.x())
            hline.setPos(mouse_point.y())
            do_render()

    def set_reference(ev):
        nonlocal reference_coordinates
        if ev.button() == Qt.LeftButton and current_coordinates is not None:
            reference_coordinates = current_coordinates
            do_render()

    plot.scene().sigMouseMoved.connect(update)
    plot.scene().sigMouseClicked.connect(set_reference)


class RealtimePlotWidget(QWidget):
    AUTO_RANGE_FRACTION = 0.99

    COLORS = [Qt.red, Qt.blue, Qt.green, Qt.magenta, Qt.cyan,
              Qt.darkRed, Qt.darkBlue, Qt.darkGreen, Qt.darkYellow, Qt.gray]

    def __init__(self, display_measurements, parent):
        super(RealtimePlotWidget, self).__init__(parent)
        self.setAttribute(Qt.WA_DeleteOnClose)              # This is required to stop background timers!
        self._plot_widget = PlotWidget()
        self._plot_widget.setBackground((0, 0, 0))
        self._legend = self._plot_widget.addLegend()
        self._plot_widget.showButtons()
        self._plot_widget.showGrid(x=True, y=True, alpha=0.3)
        vbox = QVBoxLayout(self)
        vbox.addWidget(self._plot_widget)
        self.setLayout(vbox)

        self._update_timer = QTimer(self)
        self._update_timer.setSingleShot(False)
        # noinspection PyUnresolvedReferences
        self._update_timer.timeout.connect(self._update)
        self._update_timer.start(500)

        self._color_index = 0
        self._curves = {}

        # Crosshair
        def _render_measurements(cur, ref):
            text = 'time %.6f sec,  y %.6f' % cur
            if ref is None:
                return text
            dt = cur[0] - ref[0]
            dy = cur[1] - ref[1]
            if abs(dt) > 1e-12:
                freq = '%.6f' % abs(1 / dt)
            else:
                freq = 'inf'
            display_measurements(text + ';' + ' ' * 4 + 'dt %.6f sec,  freq %s Hz,  dy %.6f' % (dt, freq, dy))

        display_measurements('Hover to sample Time/Y, click to set new reference')
        add_crosshair(self._plot_widget, _render_measurements)

        # Final reset
        self.reset()

    def add_curve(self, curve_id, curve_name, data_x=None, data_y=None):
        color = QColor(self.COLORS[self._color_index % len(self.COLORS)])
        self._color_index += 1
        pen = mkPen(color, width=1)
        plot = self._plot_widget.plot(name=curve_name, pen=pen)
        data_x = numpy.array(data_x if data_x is not None else [])
        data_y = numpy.array(data_y if data_y is not None else [])
        self._curves[curve_id] = {'data': (data_x, data_y), 'plot': plot}

    def update_values(self, curve_id, x, y):
        curve = self._curves[curve_id]
        old_x, old_y = curve['data']
        curve['data'] = numpy.append(old_x, x), numpy.append(old_y, y)

    def reset(self):
        for curve in self._curves.keys():
            self._plot_widget.removeItem(self._curves[curve]['plot'])

        self._curves = {}
        self._color_index = 0

        self._plot_widget.enableAutoRange(enable=self.AUTO_RANGE_FRACTION,
                                          x=self.AUTO_RANGE_FRACTION,
                                          y=self.AUTO_RANGE_FRACTION)

        self._legend.scene().removeItem(self._legend)
        self._legend = self._plot_widget.addLegend()

    def _update(self):
        for curve in self._curves.values():
            if len(curve['data'][0]):
                curve['plot'].setData(*curve['data'])


class Window(QMainWindow):
    def __init__(self):
        super(Window, self).__init__()
        self.setWindowTitle('Serial Plot')

        self.statusBar().show()

        self._plot = RealtimePlotWidget(self.statusBar().showMessage, self)

        # Actions menu
        clear_action = QAction('&Clear', self)
        clear_action.setShortcut(QKeySequence('Ctrl+Shift+C'))
        # noinspection PyUnresolvedReferences
        clear_action.triggered.connect(self._plot.reset)

        actions_menu = self.menuBar().addMenu('&Actions')
        actions_menu.addAction(clear_action)

        # Layout
        self.setCentralWidget(self._plot)

    @property
    def plot(self):
        return self._plot


class FileDumpWriter(threading.Thread):
    def __init__(self):
        super(FileDumpWriter, self).__init__(name='FileDumpWriter', daemon=True)
        self._f = None
        self._q = queue.Queue()
        self.start()

    def run(self):
        max_x = 0
        while True:
            x, values, names = self._q.get()
            if (x < max_x) or not self._f:
                if self._f:
                    self._f.close()
                self._f = open('latest_data.log', 'w', encoding='utf8')
                self._f.write('Started at %.6f real\n' % time.time())
            max_x = max(max_x, x)
            sample = dict(zip(names, values))
            sample['time'] = x
            self._f.write('%s\n' % sample)

    def add(self, x, values, names):
        self._q.put((x, values, names))


class SerialReader:
    DEFAULT_SIZE_TO_FORMAT_MAP = {
        1: trace_decoder.Fields.UINT8,
        2: trace_decoder.Fields.UINT16,
        4: trace_decoder.Fields.FLOAT32,
        8: trace_decoder.Fields.FLOAT64
    }

    def __init__(self, port, baudrate):
        self._port = SerialPort(port, baudrate)
        self._format = None
        self._names = None
        self._timestamp_index = 0

    def poll(self, value_handler, raw_handler):
        line = self._port.readline(timeout=1e-3).decode('latin1')
        if not line:
            return False

        # FIXME HACK splitting lines terminated with CR - needs proper handling!
        if '\r~' in line:
            line = line[line.rfind('\r~')+1:]

        if not line.startswith('~'):
            raw_handler(line)
        else:
            line = line[1:].rstrip()
            if line[0] == '\t':
                vs = [x.split('/') for x in line.split()]
                self._names = [x[0] for x in vs]
                self._timestamp_index = self._names.index('time')
                del self._names[self._timestamp_index]
                self._format = ''.join(self.DEFAULT_SIZE_TO_FORMAT_MAP[int(x[1])] for x in vs)
                print(self._names, self._timestamp_index, self._format, file=sys.stderr)
            elif self._format is not None:
                # Recovering the latest sample if multiple lines were erroneously joined together
                line = line.split('~')[-1]
                try:
                    values = list(trace_decoder.decode_sample(self._format, line))
                except Exception as ex:
                    raise Exception('Sample decoding failed on %r (%s)' % (line, ex))
                assert len(values) == len(self._names) + 1
                ts = values[self._timestamp_index]
                del values[self._timestamp_index]
                value_handler(ts, values, self._names)
            else:
                pass    # We missed the sample format declaration, let's wait for the next one

        return True

    def process_queued_data(self, value_handler, raw_handler):
        try:
            while self.poll(value_handler, raw_handler):
                pass
        except Exception as ex:
            print('Serial poll failed:', ex)


class CLIInputReader(threading.Thread):
    def __init__(self, on_line_received):
        super(CLIInputReader, self).__init__(name='CLIInputReader', daemon=True)
        self.on_line_received = on_line_received
        self.start()

    def run(self):
        while True:
            try:
                line = input()
                self.on_line_received(line)
            except Exception as ex:
                print('CLI input poll failed:', ex)


def value_handler(x, values, names):
    dumper.add(x, values, names)
    if NO_GUI:
        return
    for val, name in zip(values, names):
        try:
            scale = VARIABLE_SCALING[name]
            val *= scale
            name += ' %.3e' % scale
        except KeyError:
            pass
        try:
            window.plot.update_values(name, [x], [val])
        except KeyError:
            window.plot.add_curve(name, name, [x], [val])


if __name__ == '__main__':
    reader = SerialReader(SER_PORT, SER_BAUDRATE)
    dumper = FileDumpWriter()
    cli = CLIInputReader(lambda line: reader._port.write((line + '\r\n').encode()))

    if NO_GUI:
        while True:
            time.sleep(0.001)
            reader.process_queued_data(value_handler, lambda s: print(s.rstrip()))
    else:
        app = QApplication(sys.argv)
        window = Window()

        poll_timer = QTimer(window)
        poll_timer.setSingleShot(False)
        # noinspection PyUnresolvedReferences
        poll_timer.timeout.connect(lambda: reader.process_queued_data(value_handler, lambda s: print(s.rstrip())))
        poll_timer.start(10)

        window.show()
        exit(app.exec_())
