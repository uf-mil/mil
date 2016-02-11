#!/usr/bin/env python
import time
import numpy as np
import struct
import binascii
from sub8_thruster_comm.protocol import Const
import serial
import rospy


class ThrusterPort(object):
    _baud_rate = 115200
    _timeout = 0.01  # How long to wait for a serial response
    _max_motor_id = 100  # Max motor ID for which to verify the existence

    def __init__(self, port_info):
        '''Communicate on a single port with some thrusters
        port_info should be a dictionary, drawn from a .yaml of the form...

        - port: /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A403IMC9-if00-port0
          thrusters:
            FLV: {
              node_id: 10,
            }
            FLL: {
              node_id: 11,
            }
        Note:
            - The thrusters automatically shut down if they do not recieve a
             command for ~2 seconds

            - Potential source of problem for future users:
                If the sub is ever using more than 100 thrusters, and you find that the Sub isn't finding them,
                it is because of max_motor_id
            - This might also trigger timeout errors in the thruster_driver node spin-up
        TODO:
            --> 0x88, 0x8c: Set slew-rate up and down to something tiny
            --> Send messages without getting status
            --> Get status without sending thrust
            --> Determine which thrusters are on a port
        '''
        self.port_name = port_info['port']
        self.port = self.connect_port(self.port_name)
        self.thruster_dict = {}

        # Determine which thrusters are really on this port
        self.motor_ids_on_port = []
        for guess_id in range(self._max_motor_id):  # search a subset of possible motor ids
            have_thruster = self.check_for_thruster(guess_id)
            if have_thruster:
                self.motor_ids_on_port.append(guess_id)

        # Load thruster configurations and check if the requested thruster exists on this port
        for thruster_name, thruster_info in port_info['thrusters'].items():
            self.load_thruster_config(thruster_name, thruster_info)

    def connect_port(self, port_name):
        '''Connect to and return a serial port'''
        try:
            serial_port = serial.Serial(port_name, baudrate=self._baud_rate, timeout=self._timeout)
        except IOError, e:
            rospy.logerr("Could not connect to thruster port {}".format(port_name))
            raise(e)
        return serial_port

    def load_thruster_config(self, thruster_name, thruster_info):
        # Get our humungous error string ready
        errstr = "Could not find motor_id {} (Called {}) on port {}; existing ids are {}".format(
            thruster_info['node_id'], thruster_name, self.port_name, self.motor_ids_on_port)
        # Check if we can actually find the thruster on this port
        if not int(thruster_info['node_id']) in self.motor_ids_on_port:
            rospy.logerr(errstr)
            raise(IOError(errstr))
        self.thruster_dict[thruster_name] = int(thruster_info['node_id'])

    def checksum_struct(self, _struct):
        '''Take a struct, convert it to a bytearray, and append its crc32 checksum'''
        struct_bytearray = bytearray(_struct)
        struct_checksum = bytearray(struct.pack('<I', binascii.crc32(struct_bytearray) & 0xffffffff))
        return struct_bytearray + struct_checksum

    def make_header(self, node_id, msg_size):
        '''Construct a header'''
        length = 2 + (msg_size * 4)
        flag = Const.response_thruster_standard
        header = self.checksum_struct(
            struct.pack(
                'HBBBB',
                Const.sync_request,
                node_id,
                flag,
                Const.addr_custom_command,
                length
            )
        )
        return header

    def make_thrust_payload(self, motor_id, thrust):
        '''Construct a payload that commands a thrust'''
        send_thrust = np.clip(thrust, -1, 1)
        payload = self.checksum_struct(
            struct.pack(
                '<BBf',
                Const.propulsion_command,
                motor_id,
                send_thrust
            )
        )
        return payload

    def make_poll_payload(self, motor_id):
        payload = self.checksum_struct(
            struct.pack(
                '<BB',
                Const.propulsion_command,
                motor_id,
            )
        )
        return payload

    def to_register(self, motor_id, register, value=None):
        '''Get a register (and optionally try to set it)
        Note: This fails randomly
        '''
        assert register in Const.csr_address.keys(), "Unknown register, {}".format(register)
        address, return_size = Const.csr_address[register]

        size_char = Const.format_char_map[return_size]

        if value is not None:
            send_size = return_size
            payload = self.checksum_struct(struct.pack('<' + size_char, value))
        else:
            payload = self.checksum_struct(struct.pack('<'))
            send_size = 0

        header = self.checksum_struct(
            struct.pack(
                '<HBBBB',
                Const.sync_request,
                int(motor_id),
                0x80 | return_size,
                # Const.addr_custom_command,
                address,
                send_size
            )
        )

        packet = header + payload
        # prev = self.port.read(100)
        self.port.write(bytes(packet))
        response_bytearray = self.port.read(Const.response_normal_length + return_size)
        if len(response_bytearray) == 0:
            return None

        # rest = self.port.read(100)
        # if len(response_bytearray) ==
        response = struct.unpack('<HBBBB I B I'.format(size_char), response_bytearray)
        response_contents = [
            'sync',
            'response_node_id',
            'flag',
            'CSR_address',
            'length',
            'header_checksum',
            register,
            'payload_checksum',
        ]
        response_dict = dict(zip(response_contents, response))
        return response_dict

    def send_poll_msg(self, motor_id):
        payload = self.make_poll_payload(int(motor_id))
        header = self.make_header(int(motor_id), msg_size=0)
        packet = header + payload
        self.port.write(bytes(packet))

    def send_thrust_msg(self, motor_id, thrust):
        '''Construct and send a message to set thrust'''
        payload = self.make_thrust_payload(int(motor_id), thrust)
        header = self.make_header(int(motor_id), msg_size=1)
        packet = header + payload
        self.port.write(bytes(packet))

    def make_hex(self, msg):
        '''Return a bytearray formatted as a string of hexadecimal characters
        Useful for packet debugging
        '''
        return ":".join("{:02x}".format(c) for c in msg)

    def check_for_thruster(self, motor_id):
        self.send_poll_msg(motor_id)
        response_bytearray = self.port.read(Const.thrust_response_length)
        if len(response_bytearray) != Const.thrust_response_length:
            return False
        else:
            # We have the thruster!
            return True

    def read_status(self):
        response_bytearray = self.port.read(Const.thrust_response_length)
        # We should always get $Const.thrust_response_length bytes, if we don't, we *are* failing to communicate
        assert len(response_bytearray) == Const.thrust_response_length, "Cannot communicate with thruster"
        response = struct.unpack('<HBBBB I BffffB I', response_bytearray)
        response_contents = [
            'sync',
            'response_node_id',
            'flag',
            'CSR_address',
            'length',
            'header_checksum',
            'device_type',
            'rpm',
            'bus_voltage',
            'bus_current',
            'temperature',
            'fault',
            'payload_checksum',
        ]

        response_dict = dict(zip(response_contents, response))

        return response_dict

    def command_thruster(self, thruster_name, normalized_thrust):
        '''normalized_thrust should be between 0 and 1'''
        assert thruster_name in self.thruster_dict.keys(), "{} is not associated with port {}".format(
            thruster_name, self.port_name)
        motor_id = self.thruster_dict[thruster_name]
        self.send_thrust_msg(motor_id, normalized_thrust)
        thruster_status = self.read_status()
        return thruster_status


if __name__ == '__main__':
    '''Module test code - this requires hardware, and as such is not a unit test'''
    # port = '/dev/serial/by-id/usb-FTDI_USB-RS485_Cable_FTX1O9GJ-if00-port0'
    port = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A403IMC9-if00-port0'
    node_id = 10
    thruster_name = 'FLV'
    print('Test thruster comm over port {}, node_id {}, thruster name {}'.format(
        port,
        node_id,
        thruster_name
    ))

    port_info = {
        'port': port,
        'thrusters': {
            # 'BRV': {
            #     'node_id': 16,
            # },
            thruster_name: {
                'node_id': node_id,
            }
        }
    }

    tp = ThrusterPort(port_info)
    # tp.send_thrust_msg(node_id, 0.06)
    print tp.command_thruster(thruster_name, 0.04)
