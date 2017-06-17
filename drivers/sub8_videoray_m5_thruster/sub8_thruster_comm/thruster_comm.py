#!/usr/bin/env python
from __future__ import division
import numpy as np
import scipy.interpolate
import struct
import binascii
import copy
from sub8_thruster_comm.protocol import Const
import serial
import rospy
import json

class UnavailableThrusterException(BaseException):
    ''' Indicates that a thruster is not available to be commanded '''
    def __init__(self, motor_id=None, name=None):
        self.thruster_name = name
        self.motor_id = motor_id

    def __repr__(self):
        return 'Thruster ({}, {}) is not available to be commanded'.format(self.motor_id, self.thruster_name)

    def __str__(self):
        return self.__repr__()

    __str__ = __repr__

class UndeclaredThrusterException(BaseException):
    ''' Indicates that a thruster was not declared in the thruster layout '''
    def __init__(self, motor_id=None, name=None):
        self.thruster_name = name
        self.motor_id = motor_id

    def __repr__(self):
        return 'Thruster ({}, {}) was not declared in the layout'.format(self.motor_id, self.thruster_name)

    __str__ = __repr__

class ThrusterModel(object):
    ''' Holds the pose and effort to thrust mapping for a thruster '''

    def __init__(self, thruster_definition):
        # VideoRay comms motor_id (know in VideoRay firmware as node_id)
        motor_id = thruster_definition['motor_id']

        # Get thruster geometry
        position = thruster_definition['position']
        direction = thruster_definition['direction']

        # Minimum and Maximum thrust in newtons
        thrust_bounds = thruster_definition['thrust_bounds']

        # Loads forward and backward thrust to effort mappings (4th order polynomial)
        calib = thruster_definition['calib']

        # Validate config yaml
        assert type(motor_id) == int
        assert len(position) == 3, len(direction) == 3
        assert len(thrust_bounds) == 2, 'Need positive and backwards bounds'
        assert len(calib) == 2, 'Calib should have 2 fields: (forward, backward)'
        assert len(calib['forward']) == 4, 'Expected coefficients of 4th order model'
        assert len(calib['backward']) == 4, 'Expected coefficients of 4th order model'

        # Everything's good, assign to object
        self.motor_id = motor_id
        self.position = position
        self.direction = direction
        self.thrust_bounds = thrust_bounds
        self.calib = calib

    def get_effort_from_thrust(self, thrust):
        '''
        Uses thruster calibration to determine what effort should be sent to the motor
        to produce the desired thrust (clips output to [-1, 1])
        '''
        #t = np.array((thrust**4, thrust**3, thrust**2, thrust, 1))
        t = np.array((0, 0, 0, thrust, 1))
        calib = np.hstack((self.calib['forward'], [0])) if thrust >= 0 else \
            np.hstack((self.calib['backward'], [0]))
        return np.clip(np.dot(t, calib), -1.0, 1.0)


class ThrusterPort(object):
    _baud_rate = 115200
    _read_timeout = 0.1            # How long to wait for a serial response
    _wait_for_line_timeout = 0.02  # How long to wait for the line to become available

    def __init__(self, port_info, thruster_definitions):
        '''Communicate on a single port with some thrusters
        port_info an associates a serial port name with a list of thruster names.
        YAML form: (part of list)

        - port: /dev/serial/by-id/usb-MIL_Data_Merge_Board_Ports_1_to_4_DMBP14-if00-port0
          thruster_names: [FLH, FLV]

        thruster_definitions should be a dictionary mapping thruster names to thruster properties.
        YAML form:

        thrusters:
          FLH: {
            motor_id:  0
            position:  [0.2678, 0.2795, 0.0],
            direction: [-0.866, 0.5, 0.0],
            bounds:    [-90.0, 90.0],
            calib:  {
                forward: [3.160275618251677e-09, -2.994243542435666e-07, 6.455032155657309e-05, 0.0022659798706770326],
                backward: [4.5751802697389835e-08, 4.862785867579268e-06, -4.481294657490641e-05, 0.004241966780303877]
            }
          }
          <other_thruster_definitions_continue_here>

        Note:
            - The thrusters automatically shut down if they do not recieve a
             command for ~2 seconds

        TODO:
            --> Send messages without getting status
        '''
        self.port_name = port_info['port']
        self.port = self.connect_port(self.port_name)

        self.thruster_info = {}  # Information about all thrusters declared in the layout
        self.online_thruster_names = set()  # Keeps track of all thrusters that can be allocated thrust

        # Flag to avoid simultaneous sharing of the serial line b/w thrusters
        self.serial_busy = False

        # Load thruster configurations and check if the requested thruster exists on this port
        for thruster_name in port_info['thruster_names']:
            self.load_thruster(thruster_name, thruster_definitions)

        # Setup infrastructure to monitor comms quality
        num_thrusters = len(self.online_thruster_names)
        names = self.get_declared_thruster_names()
        self.command_tx_count = dict.fromkeys(names, 0)  # Commands sent per thruster
        self.status_rx_count = dict.fromkeys(names, 0)  # Statuses received per thruster
        self.command_latency_avg = dict.fromkeys(names, rospy.Duration(0))  # Average latency tx rx per thruster

    def connect_port(self, port_name):
        '''Connect to and return a serial port'''
        try:
            serial_port = serial.Serial(port_name, baudrate=self._baud_rate, timeout=self._read_timeout)
        except IOError as e:
            rospy.logerr("Could not connect to thruster port {}".format(port_name))
        return serial_port

    def load_thruster(self, thruster_name, thruster_definitions):
        '''
        Checks for ability to communicate with a thruster on this port. If the thruster is responsive,
        it will load the thruster definition from the thruster layout yaml (including pose and calibration)
        '''
        # Get our humongous error string ready
        errstr = "Could not get a response from motor_id {} (Called {}) on port {}".format(
            thruster_definitions[thruster_name]['motor_id'], thruster_name, self.port_name)

        # Load thruster info regardless of wether thruster is responsive
        self.thruster_info[thruster_name] = ThrusterModel(thruster_definitions[thruster_name])

        # See if thruster is responsive on this port
        if not self.check_for_thruster(self.thruster_info[thruster_name].motor_id):
            rospy.logwarn(errstr)
        else:
            self.online_thruster_names.add(thruster_name)

    def get_declared_thruster_names(self):
        ''' Gets the names of all the ports that were declared on this port '''
        return self.thruster_info.keys()

    def get_offline_thruster_names(self):
        ''' Gets the names of all of the declared thrusters that are offline '''
        offline = self.get_declared_thruster_names()
        for name in copy.deepcopy(offline):
            if name in self.online_thruster_names:
                offline.remove(name)
        return offline

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

    def reboot_thruster(self, motor_id):
        ''' Reboots a thruster '''
        address, register_size, format_char, permission = Const.csr_address['UTILITY']
        self.send_VRCSR_request_packet(
                node_id=int(motor_id),
                flags=0x80 | register_size,
                address=address,
                length=register_size,
                payload_bytes=self.checksum_struct(struct.pack('<' + format_char, 0xDEAD))
        )


    def send_VRCSR_request_packet(self, node_id, flags, address, length, payload_bytes):
        ''' Wries a VideoRay CSR (Control Service Register) packet to the serial port '''
        # Create VR_CSR packet
        header = self.checksum_struct(
            struct.pack('<HBBBB',
                Const.sync_request,
                int(node_id),
                flags,
                address,
                length
            )
        )
        packet = header + payload_bytes

        # Write packet
        self.port.write(bytes(packet))

    def parse_VRCSR_response_packet(self, packet):
        '''
        Returns a dictionary with a parsed version of the response packet header fields and the payload bytes
        '''
        payload_start_idx = Const.header_size + Const.xsum_size + 1
        header_bytes = packet[:payload_start_idx]
        payload_bytes = packet[payload_start_idx:]
        header_names = ['sync', 'node_id', 'flag', 'address', 'length', 'header_xsum', 'device_type']
        header_fields = struct.unpack('<HBBBB I B', header_bytes)
        response_dict = dict(zip(header_names, header_fields))
        response_dict['payload_bytes'] = payload_bytes
        return response_dict

    def read_register(self, motor_id, register):
        '''
        Reads a CSR register on a thruster microcontroller
        Returns the a tuple with the value of the register and the raw bytes)
        Note: although not documented by VideoRay, the payload checksum needs to be
          ommitted for this to work
        '''
        # Validate input
        assert register in Const.csr_address.keys(), 'Unknown register: {}'.format(register)
        address, register_size, format_char, permission = Const.csr_address[register]
        response_roi_len = Const.header_size + Const.xsum_size + 1 + register_size
        expected_response_length = Const.header_size + Const.xsum_size + 1 + register_size + Const.xsum_size
        assert 'R' in permission, 'Register ' + register + ' doesn\'t have read permission'

        # Do serial comms
        self.send_VRCSR_request_packet(
                node_id=int(motor_id),
                flags=0x80 | register_size,
                address=address,
                length=0,
                payload_bytes='' # It is important that the null payload xsum is ommitted!
        )
        response_bytearray = self.port.read(expected_response_length)
        assert len(response_bytearray) == expected_response_length, \
            'Expected response packet to have {} bytes, got {}'.format(expected_response_length, len(response_bytearray))

        # Unpack response packet
        response_dict = self.parse_VRCSR_response_packet(response_bytearray)
        register_bytes = response_dict['payload_bytes'][:register_size]
        register_val = struct.unpack(format_char, register_bytes)[0]

        # Validate packet
        assert response_dict['sync'] == Const.sync_response, 'Got invalid packet type: {}'.format(
            response_dict['sync'])
        assert response_dict['node_id'] == motor_id, 'Got response packet from wrong thruster: {}'.format(
            response_dict['node_id'])
        assert response_dict['address'] == address, 'Got corrupted response packet'

        return register_val, register_bytes

    def set_register(self, motor_id, register, value):
        '''
        Sets a CSR register on a thruster microcontroller
        Returns a tuple with the value and raw bytes of the recently changed register
        Note: Can only set registers that are writeable (see comms protocol)
        '''
        # Validate input
        assert register in Const.csr_address.keys(), 'Unknown register: {}'.format(register)
        address, register_size, format_char, permission = Const.csr_address[register]
        expected_response_length = Const.header_size + Const.xsum_size + 1 + register_size + Const.xsum_size
        assert 'W' in permission, 'Register ' + register + ' doesn\'t have read permission'

        # Do serial comms
        self.send_VRCSR_request_packet(
                node_id=int(motor_id),
                flags=0x80 | register_size,
                address=address,
                length=register_size,
                payload_bytes=self.checksum_struct(struct.pack('<' + format_char, value))
        )
        response_bytearray = self.port.read(expected_response_length)
        assert len(response_bytearray) == expected_response_length, \
            'Expected response packet to have {} bytes, got {}'.format(expected_response_length, len(response_bytearray))

        # Unpack response packet
        response_dict = self.parse_VRCSR_response_packet(response_bytearray)
        register_bytes = response_dict['payload_bytes'][:register_size]
        register_val = struct.unpack('<' + format_char, register_bytes)[0]

        # Validate packet
        assert response_dict['sync'] == Const.sync_response, 'Got invalid packet type: {}'.format(
            response_dict['sync'])
        assert response_dict['node_id'] == motor_id, 'Got response packet from wrong thruster: {}'.format(
            response_dict['node_id'])
        assert response_dict['address'] == address, 'Got corrupted response packet'

        return register_val, register_bytes

    def send_thrust_msg(self, motor_id, effort):
        ''' Construct and send a message to set motor effort '''
        self.send_VRCSR_request_packet(
                node_id=int(motor_id),
                flags=Const.response_thruster_standard,
                address=Const.addr_custom_command,
                length=6,  # 2 bytes and a float: (propulsion_command, motor_id, effort)
                payload_bytes=self.make_thrust_payload(int(motor_id), effort)
        )

    def make_hex(self, msg):
        '''
        Return a bytearray formatted as a string of hexadecimal characters
        Useful for packet debugging
        '''
        if type(msg) == str:
            msg = bytearray(msg)
        return ":".join("{:02x}".format(c) for c in msg)

    def check_for_thruster(self, motor_id):
        ''' Determines if a thruster with the given motor_id is responsive on this port '''
        try:
            return self.read_register(motor_id, 'NODE_ID')[0] == motor_id
        except AssertionError:
            return False

    def get_motor_ids_on_port(self, start_id, end_id):
        '''
        Polls for thrusters with motor ids within provided range and returns
        a list with the ids of thrusters that responded
        Also returns the average time between sending the poll packet and
        receiving a response for all detected thrusters
        '''
        to_check = range(start_id, end_id + 1)
        found_ids = []
        turnaround_times = [] # seconds
        for i in to_check:
            t0 = rospy.Time.now()
            if self.check_for_thruster(i):
                found_ids.append(i)
                turnaround_times.append((rospy.Time.now() - t0).to_sec())
        return found_ids, np.mean(turnaround_times)

    def parse_thrust_response(self, payload_bytes):
        ''' Parses the response to the standard propulsion command (0xAA) '''
        names = ['rpm', 'bus_v', 'bus_i', 'temp', 'fault']
        fields = struct.unpack('<ffffB', payload_bytes[:-Const.xsum_size])
        return dict(zip(names, fields))

    def command_thruster(self, name, effort):
        ''' Sets effort value on motor controller, effort should be in [-1.0, 1.0] '''
        
        # Perform availability checks
        if name not in self.thruster_info.keys():
            raise UndeclaredThrusterException(name=name)
        motor_id = self.thruster_info[name].motor_id

        # Don't try to send command packet if line is busy
        t0 = rospy.Time.now()
        while self.serial_busy and rospy.Time.now() - t0 < rospy.Duration(self._wait_for_line_timeout):
            rospy.sleep(0.001)
        if self.serial_busy and name in self.online_thruster_names:  # Don't raise if thruster is offline
            raise serial.SerialTimeoutException('{} timed out waiting on busy serial line (waited {} s)'.
                                                format(name, (rospy.Time.now() - t0).to_sec()))

        # Might not need to do this mutex around tx and rx together
        self.serial_busy = True  # Take ownership of line
        t0 = rospy.Time.now()

        # Send thrust command
        self.send_thrust_msg(motor_id, effort)
        self.command_tx_count[name] = self.command_tx_count[name] + 1

        # Parse thrust response
        response_bytearray = self.port.read(Const.thrust_response_length)
        t1 = rospy.Time.now()
        self.serial_busy = False  # Release line
        valid = len(response_bytearray) == Const.thrust_response_length
        response_dict = self.parse_VRCSR_response_packet(response_bytearray) if valid else None
        valid = valid and response_dict['sync'] == Const.sync_response and response_dict['node_id'] == motor_id \
                and response_dict['address'] == Const.addr_custom_command

        # Keep track of thrusters going offline or coming back online
        if not valid:
            if name in self.online_thruster_names:
                self.online_thruster_names.remove(name)
        else:
            if name not in self.online_thruster_names:
                self.online_thruster_names.add(name)

            # Keep track of latency
            total_latency = (t1 - t0) + self.command_latency_avg[name] * self.status_rx_count[name]
            self.status_rx_count[name] = self.status_rx_count[name] + 1
            self.command_latency_avg[name] = total_latency / self.status_rx_count[name]

        return self.parse_thrust_response(response_dict['payload_bytes']) if valid else None


if __name__ == '__main__':
    '''
    Module test code - this requires hardware, and as such is not a unit test
    '''
    import rospkg
    import rosparam
    import numpy.random as npr  # haha
    sub8_thruster_mapper = rospkg.RosPack().get_path('sub8_thruster_mapper')
    thruster_layout = rosparam.load_file(sub8_thruster_mapper + '/config/thruster_layout.yaml')[0][0]
    print thruster_layout

    port_info = npr.choice(thruster_layout['thruster_ports'])
    print "port_info", port_info
    thruster_definitions = thruster_layout['thrusters']

    thruster_name = npr.choice(port_info['thruster_names'])
    motor_id = thruster_definitions[thruster_name]['motor_id']

    print'Test thruster comm over port {}, node_id {}, thruster name {}'.format(
        port_info['port'],
        motor_id,
        thruster_name
    )

    tp = ThrusterPort(port_info, thruster_definitions)
    print tp.command_thruster(thruster_name, 0.04)
