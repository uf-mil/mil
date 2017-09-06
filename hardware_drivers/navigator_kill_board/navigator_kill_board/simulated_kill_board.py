import serial
import rospy
from constants import constants
import numpy as np
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

class NoopSerial(serial.Serial):
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
        self.async = True
        self.last_ping = None
        self.memory = {
            'BUTTON_FRONT_PORT': False,
            'BUTTON_AFT_PORT': False,
            'BUTTON_FRONT_STARBOARD': False,
            'BUTTON_AFT_STARBOARD': False,
            'COMPUTER': False,
            'REMOTE': False,
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
        if self.async:
            self.buffer += self._handle_async()
        self._check_timeout()

    def _check_timeout(self, *args):
        if self.last_ping is None:
            return
        if (rospy.Time.now() - self.last_ping).to_sec() >= constants['TIMEOUT_SECONDS']:
            self._set_kill('REMOTE', True)
        else:
            self._set_kill('REMOTE', False)

    def _set_kill(self, name, on):
        if self.memory[name] != on:
            self.memory[name] = on
            self.killed = np.any([self.memory[x] for x in self.memory])
            print 'Setting {} {}, Overall={}'.format(name, on, self.killed)

    def _set_light(self, status):
        if self.light != status:
            print 'setting lights',status
            self.light = status

    def _ping(self):
        self.last_ping = rospy.Time.now()
        return constants['PING']['RESPONSE']

    _res = lambda boolean: constants['RESPONSE_TRUE'] if boolean else constants['RESPONSE_FALSE']

    def _get_status(self, byte):
        if byte == constants['OVERALL']['REQUEST']:
            return self._res(self.killed)
        for key in constants:
            if 'REQUEST' in constants[key] and key in self.memory and byte == constants[key]['REQUEST']:
                return self._res(self.memory[key])
        return ''

    def _handle_async(self):
        tmp = ''
        tmp += constants['OVERALL']['TRUE'] if self.killed else constants['OVERALL']['FALSE']
        for key in self.memory:
            if key in self.memory:
                print key, self.memory[key]
                tmp += constants[key]['TRUE'] if self.memory[key] else constants[key]['FALSE']
        return tmp

    def _handle_sync(self, data):
        # Handle syncronous requests
        if data == constants['PING']['REQUEST']:
            return self._ping()
        if data == constants['COMPUTER']['KILL']['REQUEST']:
            self._set_kill('COMPUTER', True)
            return constants['COMPUTER']['KILL']['RESPONSE']
        if data == constants['COMPUTER']['CLEAR']['REQUEST']:
            self._set_kill('COMPUTER', False)
            return constants['COMPUTER']['CLEAR']['RESPONSE']
        if data == constants['LIGHTS']['OFF_REQUEST']:
            self._set_light('OFF')
            return constants['LIGHTS']['OFF_RESPONSE']
        if data == constants['LIGHTS']['YELLOW_REQUEST']:
            self._set_light('YELLOW')
            return constants['LIGHTS']['YELLOW_RESPONSE']
        if data == constants['LIGHTS']['GREEN_REQUEST']:
            self._set_light('GREEN')
            return constants['LIGHTS']['GREEN_RESPONSE']
        return self._get_status(data)

    def write(self, data):
        def s(data):
            '''
            Serialize data into a string representing a hex code.
            ex:
            '''
            return hex(ord(data))
        self._check_timeout()
        self.buffer += self._handle_sync(data)
        return len(data)
