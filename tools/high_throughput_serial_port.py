#!/usr/bin/env python3

import sys
import os
import serial
import threading
import multiprocessing
import queue
import time
from logging import getLogger, basicConfig as configure_logging

try:
    # noinspection PyUnresolvedReferences
    sys.getwindowsversion()
    RUNNING_ON_WINDOWS = True
except AttributeError:
    RUNNING_ON_WINDOWS = False


__all__ = ['SerialPort']


IO_PROCESS_NICENESS_INCREMENT = -18

RW_TIMEOUT = 0.1

EOL = b'\n'


logger = getLogger('high_throughput_serial_port')


# noinspection PyUnresolvedReferences
def _raise_self_process_priority():
    if RUNNING_ON_WINDOWS:
        import win32api
        import win32process
        import win32con
        handle = win32api.OpenProcess(win32con.PROCESS_ALL_ACCESS, True, win32api.GetCurrentProcessId())
        win32process.SetPriorityClass(handle, win32process.REALTIME_PRIORITY_CLASS)
    else:
        import os
        os.nice(IO_PROCESS_NICENESS_INCREMENT)


# noinspection PyBroadException
def _io_process(port_name:  str,
                tx_queue:   multiprocessing.Queue,
                rx_queue:   multiprocessing.Queue,
                parent_pid: int,
                baudrate:   int):
    configure_logging(stream=sys.stderr, level='INFO', format='%(asctime)s %(levelname)s %(name)s: %(message)s')

    # We don't need stdin
    try:
        stdin_fileno = sys.stdin.fileno()
        sys.stdin.close()
        os.close(stdin_fileno)
    except Exception:
        logger.warning('Could not close stdin', exc_info=True)

    def is_parent_process_alive():
        if RUNNING_ON_WINDOWS:
            return True  # TODO: Find a working solution for Windows (os.kill(ppid, 0) doesn't work)
        else:
            return os.getppid() == parent_pid

    try:
        _raise_self_process_priority()
    except Exception as ex:
        logger.info('Could not adjust own process priority (this is not an error): %r', ex)

    try:
        port = serial.Serial(port_name, baudrate=baudrate, timeout=RW_TIMEOUT, write_timeout=RW_TIMEOUT)
    except Exception as ex:
        logger.error('Could not open port', exc_info=True)
        rx_queue.put(ex)
        return

    should_exit = False

    def rx_thread_entry_point():
        nonlocal should_exit
        try:
            while not should_exit and is_parent_process_alive():
                data = port.read(max(1, port.in_waiting))
                if data:
                    rx_queue.put(data)
        except Exception as ex:
            logger.error('RX thread failed', exc_info=True)
            rx_queue.put(ex)
        finally:
            should_exit = True
            logger.info('RX thread is stopping')

    rx_thread_handle = threading.Thread(target=rx_thread_entry_point, name='htsp_rx_thread', daemon=True)

    try:
        rx_thread_handle.start()

        while not should_exit and rx_thread_handle.is_alive() and is_parent_process_alive():
            try:
                data = tx_queue.get(timeout=RW_TIMEOUT)
            except queue.Empty:
                pass
            else:
                if data:
                    port.write(data)
                else:
                    should_exit = True
    except Exception as ex:
        logger.error('IO process failed', exc_info=True)
        rx_queue.put(ex)
    finally:
        should_exit = True
        if rx_thread_handle.is_alive():
            rx_thread_handle.join()

        port.close()

    logger.info('Stopping')


def synchronized(method):
    def decorator(self, *arg, **kws):
        with self._lock:
            return method(self, *arg, **kws)
    return decorator


class SerialPort:
    def __init__(self,
                 port_name: str,
                 baudrate:  int):
        self._lock = threading.RLock()

        self._mp_context = multiprocessing.get_context('spawn')

        self._backlog = bytes()

        self._txq = self._mp_context.Queue()
        self._rxq = self._mp_context.Queue()

        parent_pid = os.getpid()

        self._process = self._mp_context.Process(target=_io_process, name='htsp_io_process',
                                                 args=(port_name, self._txq, self._rxq, parent_pid, baudrate))
        self._process.daemon = True
        self._process.start()

    def _read_into_backlog_until(self, termination_condition, timeout):
        """Reads data until termination_condition() returns true, or until read timeout is exceeded.
        """
        while not termination_condition():
            blocked_at = time.monotonic()
            try:
                if timeout is not None:
                    timeout = max(0, timeout)
                    should_block = timeout > 0
                else:
                    should_block = True
                ret = self._rxq.get(block=should_block, timeout=timeout)
            except queue.Empty:
                if not self._process.is_alive():
                    raise RuntimeError('IO process is dead')
                break               # Timeout

            if timeout is not None:
                timeout -= time.monotonic() - blocked_at

            if isinstance(ret, BaseException):
                raise RuntimeError('IO process error') from ret

            self._backlog += ret

    @synchronized
    def read(self, size=-1, timeout=None) -> bytes:
        """
        Reads the specified number of bytes with optional timeout.
        With default arguments this method acts as a proxy for readall().
        :param size:        number of bytes to read, infinity (negative) by default.
        :param timeout:     timeout in seconds, None for infinity
        :return:            bytes, possibly empty
        """
        if size < 0 and timeout is None:
            return self.readall()

        def termination_condition():
            if size >= 0:
                return len(self._backlog) >= size
            else:
                return False

        self._read_into_backlog_until(termination_condition, timeout)

        ret, self._backlog = self._backlog[:size], self._backlog[size:]
        return ret

    @synchronized
    def readall(self) -> bytes:
        """
        Performs non-blocking read of all data stored in the read buffer.
        :return:    bytes, possibly empty
        """
        self._read_into_backlog_until(lambda: False, 0)
        ret, self._backlog = self._backlog, bytes()
        return ret

    @synchronized
    def readline(self, timeout=None) -> bytes:
        """
        Reads one line until LF; returns the line without modifications, including the trailing LF character.
        :param timeout:     timeout in seconds, None for infinity
        :return:            the read line, or empty byte sequence on timeout
        """
        def termination_condition():
            return EOL in self._backlog

        self._read_into_backlog_until(termination_condition, timeout)

        if EOL in self._backlog:
            split_pos = self._backlog.find(EOL) + len(EOL)
            ret, self._backlog = self._backlog[:split_pos], self._backlog[split_pos:]
            return ret

        return bytes()

    def write(self, data, timeout=None) -> int:
        """
        This method can be invoked concurrently from multiple sources, even if there are other threads blocked on
        any of the reading methods.
        :param data:        data, bytes or str
        :param timeout:     timeout in seconds, None for infinity
        :return:            number of bytes written - always either len(data) or 0 on timeout
        """
        if isinstance(data, str):
            data = data.encode('utf8')

        if not isinstance(data, bytes):
            raise ValueError('Invalid data type: %r' % type(data))

        try:
            if len(data) > 0:
                self._txq.put(data, timeout=timeout)
        except queue.Full:
            return 0
        else:
            return len(data)

    @synchronized
    def close(self):
        if self._process.is_alive():
            self._txq.put(None)
            self._process.join()


if __name__ == '__main__':
    sp = SerialPort('/dev/serial/by-id/usb-Zubax_Robotics_PX4ESC_26003B00115134333130373000000000-if00', 115200)

    sp.write('\r\nsysinfo\r\n\r\nstatus\r\n')

    while True:
        line = sp.readline(timeout=.1)
        if line:
            print(line.decode('latin1').rstrip())
        else:
            break

    time.sleep(1)
    print(sp.read().decode('latin1'))

    sp.close()
