#!/usr/bin/env python
from __future__ import division
import numpy as np
import scipy.interpolate
import struct
import binascii
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
            bounds:    [-90.0, 90.0]
          }
          <other_thruster_definitions_continue_here>

        Note:
            - The thrusters automatically shut down if they do not recieve a
             command for ~2 seconds

        TODO:
            --> 0x88, 0x8c: Set slew-rate up and down to something tiny
            --> Send messages without getting status
            --> Get status without sending thrust
            --> Determine which thrusters are on a port
        '''
        self.port_name = port_info['port']
        self.port = self.connect_port(self.port_name)

        # Mapping from name to motor_id for all thrusters responsive at startup
        self.active_motor_ids_from_names = {}

        # Flag to avoid simultaneous sharing of the serial line b/w thrusters
        self._serial_busy = False

        # Load thruster configurations and check if the requested thruster exists on this port
        self.missing_thrusters = []
        for thruster_name in port_info['thruster_names']:
            try:
                # Note: will only try to detect thrusters as listed in the layout. That means
                # that if a thruster is connected to the wrong port, IT WILL NOT BE FOUND.
                self.load_thruster(thruster_name, thruster_definitions)
            except UnavailableThrusterException:
                self.missing_thrusters.append(thruster_name)
                continue

        # Setup infrastructure to monitor comms quality
        num_thrusters = len(self.active_motor_ids_from_names)
        names = self.active_motor_ids_from_names.keys()
        self._command_tx_count = dict.fromkeys(names, 0)  # Commands sent per thruster
        self._status_rx_count = dict.fromkeys(names, 0)  # Statuses received per thruster
        self._command_latency_avg = dict.fromkeys(names, rospy.Duration(0))  # Average latency tx rx per thruster

    def connect_port(self, port_name):
        '''Connect to and return a serial port'''
        try:
            serial_port = serial.Serial(port_name, baudrate=self._baud_rate, timeout=self._read_timeout)
        except IOError as e:
            rospy.logerr("Could not connect to thruster port {}".format(port_name))
            raise(e)
        return serial_port

    def load_thruster(self, thruster_name, thruster_definitions):
        '''
        Looks for thruster definition in the thruster layout yaml and loads motor_id and
        effort to thrust calibration if the thruster responds through this serial port
        '''
        # Get our humungous error string ready
        errstr = "Could not get a response from motor_id {} (Called {}) on port {}".format(
            thruster_definitions[thruster_name]['motor_id'], thruster_name, self.port_name)

        # Check if we can actually find the thruster on this port
        motor_id = int(thruster_definitions[thruster_name]["motor_id"])
        if not self.check_for_thruster(motor_id):
            rospy.logerr(errstr)
            raise UnavailableThrusterException(motor_id=motor_id, name=thruster_name)
        self.active_motor_ids_from_names[thruster_name] = motor_id

        # Load effort to force calibration from file
        calib_dir = rospy.get_param('/thrusters/calibration_dir')
        calib_file = calib_dir + thruster_definitions[thruster_name]['calib']
        if not hasattr(self, 'effort_to_thrust'):
            self.effort_to_thrust = {}
        self.effort_to_thrust[thruster_name] = self.load_effort_to_thrust_map(calib_file)

    def load_effort_to_thrust_map(self, path):
        '''
        Load the effort to thrust mapping from a yaml file:
        - Map force inputs from Newtons to [-1, 1] required by the thruster
        '''
        try:
            rospy.logdebug("Loading thruster calibration from {}".format(path))
            _file = file(path)
        except IOError as e:
            rospy.logerr("Could not find thruster configuration file at {}".format(path))
            raise(e)

        json_data = json.load(_file)
        newtons = json_data['calibration_data']['newtons']
        thruster_input = json_data['calibration_data']['thruster_input']
        return scipy.interpolate.interp1d(newtons, thruster_input)

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
        '''
        Get a register (and optionally try to set it)
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
        self.port.write(bytes(packet))
        response_bytearray = self.port.read(Const.response_normal_length + return_size)
        if len(response_bytearray) == 0:
            return None

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
        '''
        Return a bytearray formatted as a string of hexadecimal characters
        Useful for packet debugging
        '''
        return ":".join("{:02x}".format(c) for c in msg)

    def check_for_thruster(self, motor_id):
        self.send_poll_msg(motor_id)
        response_dict = self.read_status()
        if response_dict is None:
            return False
        else:
            # We have the thruster!
            found = response_dict['response_node_id'] == motor_id
            return found

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

    def read_status(self):
        response_bytearray = self.port.read(Const.thrust_response_length)
        # We should always get $Const.thrust_response_length bytes, if we don't
        # we *are* failing to communicate
        if len(response_bytearray) != Const.thrust_response_length:
            return None

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

    def command_thruster(self, name, effort):
        ''' Sets effort value on motor controller, effort should be in [-1.0, 1.0] '''
        # Confirm thruster is available
        if name not in self.active_motor_ids_from_names.keys():
            raise UnavailableThrusterException(name=name)

        motor_id = self.active_motor_ids_from_names[name]

        # Don't try to send command packet if line is busy
        t0 = rospy.Time.now()
        while self._serial_busy and rospy.Time.now() - t0 < rospy.Duration(_wait_for_line_timeout):
            rospy.sleep(0.001)
        if self._serial_busy:
            raise serial.SerialTimeoutException('{} timed out waiting on busy serial line'.format(name))

        # Might not need to do this mutex around tx and rx together
        self._serial_busy = True  # Take ownership of line
        t0 = rospy.Time.now()
        self.send_thrust_msg(motor_id, effort)
        self._command_tx_count[name] = self._command_tx_count[name] + 1
        thruster_status = self.read_status()
        t1 = rospy.Time.now()
        self._serial_busy = False  # Release line

        # Keep track of latency
        if thruster_status is not None:
            total_latency = (t1 - t0) + self._command_latency_avg[name] * self._status_rx_count[name]
            self._status_rx_count[name] = self._status_rx_count[name] + 1
            self._command_latency_avg[name] = total_latency / self._status_rx_count[name]

        return thruster_status


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
