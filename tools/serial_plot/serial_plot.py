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
import datetime

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))  # Adding outer directory

import variable_trace_decoder as vtd
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
        while True:
            try:
                text = self._q.get(timeout=1)
            except queue.Empty:
                if self._f is not None:
                    self._f.flush()
                continue

            if isinstance(text, str):
                text = text.encode('utf8')

            if not self._f:
                name = 'cli.%s.log' % datetime.datetime.now().strftime('%Y-%m-%d-%H:%M:%S')
                self._f = open(name, 'wb')

            self._f.write(text)

    def add(self, text):
        self._q.put(text)


class SerialReader:
    def __init__(self, port, baudrate, dumper):
        self._port = SerialPort(port, baudrate)
        self._decoder = vtd.Decoder()
        self._dumper = dumper

    def poll(self, value_handler, raw_handler):
        line = self._port.readline(timeout=1e-3)
        if not line:
            return False

        self._dumper.add(line)

        if NO_GUI:
            if not line.startswith(b'~'):
                raw_handler(line.decode('latin1'))
        else:
            sample = self._decoder.process_line(line)
            if sample:
                ts = sample['time']
                del sample['time']
                value_handler(ts, sample)
            else:
                raw_handler(line.decode('latin1'))

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


def value_handler(x, sample):
    for name, val in sample.items():
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
    dumper = FileDumpWriter()
    reader = SerialReader(SER_PORT, SER_BAUDRATE, dumper)
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
