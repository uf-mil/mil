import serial
import rospy
from constants import constants
import numpy as np
from std_srvs.srv import SetBool, SetBoolResponse


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


class SimulatedKillBoard(SimulatedSerial):
    '''
    Pretends to be NaviGator's kill board over serial, responding according to the protocol
    to requests and sending current state periodically
    '''
    port = 'simulated-kill-board'

    def __init__(self, *args, **kwargs):
        super(SimulatedKillBoard, self).__init__()
        self.last_ping = None
        self.memory = {
            'BUTTON_FRONT_PORT': False,
            'BUTTON_AFT_PORT': False,
            'BUTTON_FRONT_STARBOARD': False,
            'BUTTON_AFT_STARBOARD': False,
            'COMPUTER': False,
            'HEARTBEAT_COMPUTER': False,
            'BUTTON_REMOTE': False,
            'HEARTBEAT_REMOTE': False,
        }
        for key in constants['KILLS']:
            if key.find('BUTTON') == 0:
                rospy.Service('~{}'.format(key), SetBool, lambda req, _button=key: self._set_button(_button, req.data))

        self.killed = False
        self.light = 'OFF'
        rospy.Timer(rospy.Duration(0.2), self._timer_cb)

    def _set_button(self, button, pressed):
        self._set_kill(button, pressed)
        return SetBoolResponse(success=True)

    def _timer_cb(self, *args):
        self._check_timeout()

    def _check_timeout(self, *args):
        if self.last_ping is None:
            return
        if (rospy.Time.now() - self.last_ping).to_sec() >= constants['TIMEOUT_SECONDS']:
            self._set_kill('HEARTBEAT_COMPUTER', True)
        else:
            self._set_kill('HEARTBEAT_COMPUTER', False)

    def _set_kill(self, name, on, update=True):
        if self.memory[name] != on:
            self.memory[name] = on
            self.killed = np.any([self.memory[x] for x in self.memory])
        if not update:
            return
        if on:
            self.buffer += constants[name]['TRUE']
        else:
            self.buffer += constants[name]['FALSE']
        if self.killed:
            self.buffer += constants['OVERALL']['TRUE']
        else:
            self.buffer += constants['OVERALL']['FALSE']

    def _set_light(self, status):
        if self.light != status:
            print 'setting lights', status
            self.light = status

    def _get_status(self, byte):
        def _res(boolean):
            return constants['RESPONSE_TRUE'] if boolean else constants['RESPONSE_FALSE']
        if byte == constants['OVERALL']['REQUEST']:
            self.buffer = _res(self.killed) + self.buffer
            return
        for key in self.memory:
            if byte == constants[key]['REQUEST']:
                self.buffer = _res(self.memory[key]) + self.buffer
                return

    def _handle_sync(self, data):
        # Handle syncronous requests
        if data == constants['PING']['REQUEST']:
            self.last_ping = rospy.Time.now()
            self.buffer = constants['PING']['RESPONSE'] + self.buffer
        elif data == constants['COMPUTER']['KILL']['REQUEST']:
            self._set_kill('COMPUTER', True)
            self.buffer = constants['COMPUTER']['KILL']['RESPONSE'] + self.buffer
        elif data == constants['COMPUTER']['CLEAR']['REQUEST']:
            self._set_kill('COMPUTER', False)
            self.buffer = constants['COMPUTER']['CLEAR']['RESPONSE'] + self.buffer
        elif data == constants['LIGHTS']['OFF_REQUEST']:
            self._set_light('OFF')
            self.buffer = constants['LIGHTS']['OFF_RESPONSE'] + self.buffer
        elif data == constants['LIGHTS']['YELLOW_REQUEST']:
            self._set_light('YELLOW')
            self.buffer = constants['LIGHTS']['YELLOW_RESPONSE'] + self.buffer
        elif data == constants['LIGHTS']['GREEN_REQUEST']:
            self._set_light('GREEN')
            self.buffer = constants['LIGHTS']['GREEN_RESPONSE'] + self.buffer
        else:
            self._get_status(data)

    def write(self, data):
        def s(data):
            '''
            Serialize data into a string representing a hex code.
            ex:
            '''
            return hex(ord(data))
        self._check_timeout()
        self._handle_sync(data)
        return len(data)
