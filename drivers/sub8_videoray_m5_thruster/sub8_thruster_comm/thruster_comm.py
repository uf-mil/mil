#!/usr/bin/env python
import numpy as np
import struct
import binascii
from .protocol import Const
import serial
import rospy


class ThrusterPort(object):
    _baud_rate = 115200

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
        '''
        self.port_name = port_info['port']
        self.thruster_dict = {}
        for thruster_name, thruster_info in port_info['thrusters'].items():
            self.load_thruster_config(thruster_name, thruster_info)

        self.port = self.connect_port(self.port_name)

    def connect_port(self, port_name):
        '''Connect to and return a serial port'''
        try:
            serial_port = serial.Serial(port_name, self._baud_rate)
        except IOError, e:
            rospy.logerr("Could not connect to thruster port {}".format(port_name))
            raise(e)
        return serial_port

    def load_thruster_config(self, thruster_name, thruster_info):
        self.thruster_dict[thruster_name] = thruster_info['node_id']

    def checksum_struct(self, _struct):
        '''Take a struct, convert it to a bytearray, and append its crc32 checksum'''
        struct_bytearray = bytearray(_struct)
        struct_checksum = bytearray(struct.pack('i', binascii.crc32(struct_bytearray)))
        return struct_bytearray + struct_checksum

    def make_header(self, node_id, msg_size):
        '''Construct a header'''
        length = 2 + (msg_size * 4)
        flag = Const.response_thruster_standard
        header = self.checksum_struct(
            struct.pack('HBBBB',
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
            struct.pack('<BBf',
                Const.propulsion_command,
                motor_id,
                send_thrust
            )
        )
        return payload

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

    def read_status(self):
        response_length = (
            Const.protocol_vrcsr_header_size +
            Const.protocol_vrcsr_xsum_size +
            Const.response_thruster_standard_length +
            Const.protocol_vrcsr_xsum_size
        )
        response_bytearray = self.port.read(response_length)
        response = struct.unpack('=HBBBB I BffffB I', response_bytearray)

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
    port = '/dev/serial/by-id/usb-FTDI_USB-RS485_Cable_FTX1O9GJ-if00-port0'
    node_id = 17
    thruster_name = 'BRL'
    print 'Testing thruster communication over port {}, node_id {}, thruster name {}'.format(port, node_id, thruster_name)

    port_info = {
        'port': port,
        'thrusters': {
            'BRV': {
                'node_id': 16,
            },
            thruster_name: {
                'node_id': node_id,
            }
        }
    }

    tp = ThrusterPort(port_info)
    tp.send_thrust_msg(node_id, 0.06)