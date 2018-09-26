#!/usr/bin/env python
import serial


class NoopSerial(serial.Serial):
    '''
    Inherits from serial.Serial, doing nothing for each function.
    Allows super classes to implement custom behavior for simulating
    serial devices.
    '''
    port = 'noop-serial'

    def __init__(*args, **kwargs):
        pass

    def open(self):
        pass

    @property
    def in_waiting(self):
        return 0

    @property
    def out_waiting(self):
        return 0

    def close(self):
        pass

    def __del__(self):
        pass

    def read(self, **kwargs):
        pass

    def write(self, *args):
        pass

    def flush(self):
        pass

    def flushInput(self):
        pass

    def flushOuput(self):
        pass

    def reset_input_buffer(self):
        pass

    def reset_output_buffer(self):
        pass

    def send_break(self, *args, **kwargs):
        pass


class SimulatedSerial(NoopSerial):
    '''
    Simulates a serial device, storing a buffer to be read in a program like a normal OS serial device.

    Intended to be extended by other classes, which should override the write function to recieve writes to
    the simulated device. These classes simply append to the buffer string which will be returned
    on reads to the simulated device.

    Note: NoopSerial and SimulatedSerial are generic and are candidates for mil_common.
    '''

    def __init__(self, *args, **kwargs):
        self.buffer = ''

    @property
    def in_waiting(self):
        return len(self.buffer)

    def reset_input_buffer(self):
        self.buffer = ''

    def read(self, length):
        b, self.buffer = self.buffer[0:length], self.buffer[length:]
        return b
