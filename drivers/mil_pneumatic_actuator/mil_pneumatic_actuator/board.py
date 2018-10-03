#!/usr/bin/env python
from mil_ros_tools import thread_lock
from simulated_board import SimulatedPnuematicActuatorBoard
from constants import Constants
import threading
import serial

lock = threading.Lock()


class PnuematicActuatorDriverError(Exception):
    def __init__(self, message):
        super(PnuematicActuatorDriverError, self).__init__('Actuator board: ' + message)


class PnuematicActuatorDriverChecksumError(PnuematicActuatorDriverError):
    def __init__(self, checksum_is, checksum_should_be):
        message = 'Invalid checksum. Recievied {}, should be {}'.format(hex(checksum_is), hex(checksum_should_be))
        super(PnuematicActuatorDriverChecksumError, self).__init__(message)


class PnuematicActuatorDriverResponseError(PnuematicActuatorDriverError):
    def __init__(self, received, expected):
        message = 'Unexpected response. Expected {}, recieved {}'.format(hex(received), hex(expected))
        super(PnuematicActuatorDriverResponseError, self).__init__(message)


class PnuematicActuatorDriver(object):

    '''
    Allows high level ros code to interface with Daniel's pneumatics board.

    For dropper and grabber, call service with True or False to open or close.
    For shooter, sending a True signal will pulse the valve.

    TODO: Add a function to try and reconnect to the serial port if we lose connection.

    Infomation on communication protcol:

    Sending bytes: send byte (command), then send byte XOR w/ 0xFF (checksum)
    Receiving bytes: receive byte (response), then receive byte XOR w/ 0xFF (checksum)

    Base opcodes:
    - 0x10 ping
    - 0x20 open valve (allow air flow)
    - 0x30 close valve (prevent air flow)
    - 0x40 read switch

    - To 'ping' board (check if board is operational): send 0x10, if operational,
      will reply with 0x11.

    - To open valve (allow air flow): send 0x20 + valve number
      (ex. 0x20 + 0x04 (valve #4) = 0x24 <-- byte to send), will reply with 0x01.

    - To close valve (prevent air flow): send 0x30 + valve number
      (ex. 0x30 + 0x0B (valve #11) = 0x3B <-- byte to send),will reply with 0x00.

    - To read switch: send 0x40 + switch number
      (ex. 0x40 + 0x09 (valve #9) = 0x49 <-- byte to send), will reply with 0x00
      if switch is open (not pressed) or 0x01 if switch is closed (pressed).
    '''
    def __init__(self, port, baud=9600, simulated=False):
        if simulated:
            self.ser = SimulatedPnuematicActuatorBoard()
        else:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=None)
        self.ser.flushInput()

    @classmethod
    def _verify_checksum(cls, byte, checksum):
        if not Constants.verify_checksum(byte, checksum):
            raise PnuematicActuatorDriverChecksumError(checksum, Constants.create_checksum(byte))

    def _get_response(self):
        data = self.ser.read(2)
        response = Constants.deserialize_packet(data)
        data = response[0]
        chksum = response[1]
        self._verify_checksum(data, chksum)
        return data

    @thread_lock(lock)
    def _send_request(self, byte, expected_response=None):
        data = Constants.serialize_packet(byte)
        self.ser.write(data)
        response = self._get_response()
        if expected_response is not None and expected_response != response:
            raise PnuematicActuatorDriverResponseError(response, expected_response)
        return response

    def open_port(self, port):
        byte = Constants.OPEN_REQUEST_BASE + port
        return self._send_request(byte, Constants.OPEN_RESPONSE)

    def close_port(self, port):
        byte = Constants.CLOSE_REQUEST_BASE + port
        return self._send_request(byte, Constants.CLOSE_RESPONSE)

    def set_port(self, port, do_open):
        if do_open:
            return self.open_port(port)
        else:
            return self.close_port(port)

    def get_port(self, port):
        byte = Constants.READ_REQUEST_BASE + port
        return self._send_request(byte)

    def ping(self):
        return self._send_request(Constants.PING_REQUEST, Constants.PING_RESPONSE)
