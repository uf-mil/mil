#!/usr/bin/env python
from __future__ import division
import numpy as np
import struct
import binascii
import copy
from sub8_thruster_comm.protocol import Const
from sub8_exception import SubjuGatorException
import serial
import rospy


class UnavailableThrusterException(SubjuGatorException):
    ''' Indicates that a thruster is not available to be commanded '''

    def __init__(self, node_id=None, name=None):
        self.thruster_name = name
        self.node_id = node_id

    def __repr__(self):
        return 'Thruster ({}, {}) is not available to be commanded'.format(self.node_id, self.thruster_name)

    __str__ = __repr__


class UndeclaredThrusterException(SubjuGatorException):
    ''' Indicates that a thruster was not declared in the thruster layout '''

    def __init__(self, node_id=None, name=None):
        self.thruster_name = name
        self.node_id = node_id

    def __repr__(self):
        return 'Thruster ({}, {}) was not declared in the layout'.format(self.node_id, self.thruster_name)

    __str__ = __repr__


class VRCSRException(SubjuGatorException):
    ''' Identifies exceptional states that are related to VideoRay CSR comms
    Takes an arbitrary number of ordered and keyword params
    '''
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs


class Sub8SerialException(SubjuGatorException):
    ''' Identifies an exceptional state of the comms port
    Takes an arbitrary number of ordered and keyword params
    '''
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs


class ThrusterModel(object):
    ''' Holds the pose and effort to thrust mapping for a thruster '''

    def __init__(self, thruster_definition):
        # VideoRay comms node_id (know in VideoRay firmware as node_id)
        node_id = thruster_definition['node_id']

        # Get thruster geometry
        position = thruster_definition['position']
        direction = thruster_definition['direction']

        # Minimum and Maximum thrust in newtons
        thrust_bounds = thruster_definition['thrust_bounds']

        # Loads forward and backward thrust to effort mappings (4th order polynomial)
        calib = thruster_definition['calib']

        # Validate config yaml
        assert type(node_id) == int
        assert len(position) == 3, len(direction) == 3
        assert len(thrust_bounds) == 2, 'Need positive and backwards bounds'
        assert len(calib) == 2, 'Calib should have 2 fields: (forward, backward)'
        assert len(calib['forward']) == 4, 'Expected coefficients of 4th order model'
        assert len(calib['backward']) == 4, 'Expected coefficients of 4th order model'

        # Everything's good, assign to object
        self.node_id = node_id
        self.position = position
        self.direction = direction
        self.thrust_bounds = thrust_bounds
        self.calib = calib

    def get_effort_from_thrust(self, thrust):
        '''
        Uses thruster calibration to determine what effort should be sent to the motor
        to produce the desired thrust (clips output to [-1, 1])
        '''
        t = np.array((thrust**4, thrust**3, thrust**2, thrust, 1))
        calib = np.hstack((self.calib['forward'], [0])) if thrust >= 0 else \
            np.hstack((self.calib['backward'], [0]))
        return np.clip(np.dot(t, calib), -1.0, 1.0)


class ThrusterPort(object):
    _baud_rate = 115200
    _read_timeout = 0.03           # About twice of the average responce latency
    _wait_for_line_timeout = 0.02  # How long to wait for the line to become available

    def __init__(self, port_info, thruster_definitions):
        '''Communicate on a single port with some thrusters
        @param port_info Dictionary that associates a serial port name with a list of thruster names.
        YAML form: (part of list)

        - port: /dev/serial/by-id/usb-MIL_Data_Merge_Board_Ports_1_to_4_DMBP14-if00-port0
          thruster_names: [FLH, FLV]
          CONFIDENT_MODE: False

        @param thruster_definitions Dictionary mapping thruster names to thruster properties.
        YAML form:

        thrusters:
          FLH: {
            node_id:  0
            position:  [0.2678, 0.2795, 0.0],
            direction: [-0.866, 0.5, 0.0],
            bounds:    [-90.0, 90.0],
            calib:  {
                forward:  [0.0, 0.0, -0.000135, 0.0228],
                backward: [0.0, 0.0, 0.0000531, 0.0151]

            }
          }
          <other_thruster_definitions_continue_here>

        Note:
            - The thrusters automatically shut down if they do not recieve a
             command for ~2 seconds
        '''
        self.port_name = port_info['port']
        self.port = self.connect_port(self.port_name)
        self.CONFIDENT_MODE = port_info['CONFIDENT_MODE']  # Dangerous mode that will not detect thruster losses

        self.thruster_info = {}  # Information about all thrusters declared in the layout
        self.online_thruster_names = set()  # Keeps track of all thrusters that can be allocated thrust

        # Flag to avoid simultaneous sharing of the serial line b/w thrusters
        self.serial_busy = False

        # Load thruster configurations and check if the requested thruster exists on this port
        for thruster_name in port_info['thruster_names']:
            self.load_thruster(thruster_name, thruster_definitions)

        # Setup infrastructure to monitor comms quality
        names = self.get_declared_thruster_names()
        self.command_tx_count = dict.fromkeys(names, 0)  # Commands sent per thruster
        self.status_rx_count = dict.fromkeys(names, 0)  # Statuses received per thruster
        self.command_latency_avg = dict.fromkeys(names, rospy.Duration(0))  # Average latency tx rx per thruster

    def connect_port(self, port_name):
        '''Connect to and return a serial port'''
        try:
            serial_port = serial.Serial(port_name, baudrate=self._baud_rate, timeout=self._read_timeout)
            serial_port.flushInput()
            return serial_port
        except serial.serialutil.SerialException:
            rospy.logwarn("Could not connect to thruster port {}".format(port_name))
            raise Sub8SerialException('Unable to connect to serial port', port_name=port_name)

    def load_thruster(self, thruster_name, thruster_definitions):
        '''
        Checks for ability to communicate with a thruster on this port. If the thruster is responsive,
        it will load the thruster definition from the thruster layout yaml (including pose and calibration)
        '''
        # Get our humongous error string ready
        errstr = "Could not get a response from node_id {} (Called {}) on port {}".format(
            thruster_definitions[thruster_name]['node_id'], thruster_name, self.port_name)

        # Load thruster info regardless of wether thruster is responsive
        self.thruster_info[thruster_name] = ThrusterModel(thruster_definitions[thruster_name])
        if self.CONFIDENT_MODE:
            self.online_thruster_names.add(thruster_name)  # Online to begin with, no response needed

        # See if thruster is responsive on this port
        if self.port and not self.check_for_thruster(self.thruster_info[thruster_name].node_id):
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

    def check_for_thruster(self, node_id):
        ''' Checks for a thruster with a given node_id on this port
        @param node_id Node_id of the thruster to check for

        @rtype bool
        @return Whether a valid response from the given thruster was received.
        '''
        try:
            res = self.read_register(node_id, 'NODE_ID')
            return res[0] == node_id
        except VRCSRException as e:
            rospy.logdebug(e)
            return False

    def get_node_ids_on_port(self, start_id=0, end_id=9):
        ''' Polls for node_id's on a port
        @type start_id: int
        @param start_id Beggining of node_id range to search for

        @type end_id: int
        @param end_id End of node_id range to search for

        @rtype (list, float)
        @return A tupele with a list of the node_id's of thrusters that responded
            and the average response latency for all detected thrusters.
        '''
        to_check = range(start_id, end_id + 1)
        found_ids = []
        turnaround_times = []  # seconds
        for i in to_check:
            t0 = rospy.Time.now()
            if self.check_for_thruster(i):
                found_ids.append(i)
                turnaround_times.append((rospy.Time.now() - t0).to_sec())
        return found_ids, np.mean(turnaround_times) if len(turnaround_times) > 0 else 0

    def make_hex(self, msg):
        ''' Return a bytearray formatted as a string of hexadecimal characters
        Useful for packet debugging
        @type msg: string|bytearray
        @param msg: Input to format as a hex string
        '''
        if type(msg) == str:
            msg = bytearray(msg)
        return ":".join("{:02x}".format(c) for c in msg)

    def checksum_struct(self, _struct):
        ''' Converts a struct to a bytearray, and appends its crc32 checksum
        @type _struct: struct
        @param _struct: Input struct for which  to calculate and append the crc32 checksum
        '''
        struct_bytearray = bytearray(_struct)
        struct_checksum = bytearray(struct.pack('<I', binascii.crc32(struct_bytearray) & 0xffffffff))
        return struct_bytearray + struct_checksum

    def validate_packet_integrity(self, response_bytearray):
        ''' Validates the integrity of a VRCSR response packet using crc32 checksums
        @type response_bytearray: bytearray
        @param response_bytearray: The response to a VRCSR packet

        @rtype: (bool, string)
        @return: Packet validity, reason for packet being invalid if it is
        '''
        size = len(response_bytearray)
        if size < Const.header_size + 1 + 2 * Const.xsum_size:  # Minimum size is header + device_type + 2 xsums
            return False, 'Packet size ({}) less than minimum for a valid packet'.format(size)
        payload_start_idx = Const.header_size + Const.xsum_size
        header_bytes = response_bytearray[:Const.header_size]
        header_xsum_bytes = response_bytearray[Const.header_size: payload_start_idx]
        expected_header_xsum = struct.unpack('<I', header_xsum_bytes)[0]
        actual_header_xsum = binascii.crc32(header_bytes) & 0xffffffff
        if expected_header_xsum != actual_header_xsum:
            return False, 'Packet has invalid header checksum'
        if len(response_bytearray) > payload_start_idx:  # Null checksums are usually ommited
            remaining_bytes = response_bytearray[payload_start_idx:]
            payload_bytes = remaining_bytes[:-Const.xsum_size]
            payload_xsum_bytes = remaining_bytes[-Const.xsum_size:]
            expected_header_xsum = struct.unpack('<I', payload_xsum_bytes)[0]
            actual_header_xsum = binascii.crc32(payload_bytes) & 0xffffffff
            if expected_header_xsum != actual_header_xsum:
                return False, 'Packet has invalid payload checksum'
        return True, ''

    def save_settings(self, node_id):
        ''' Saves the settings on a thruster motorcontroller
        @type node_id: int
        @param node_id: Node_id of the thruster whose settings will be saved
        '''
        self.set_register(node_id, 'save_settings', 0x1234)  # 0x1234 is the save settings password

    def reboot_thruster(self, node_id):
        ''' Reboots a thruster
        @type node_id: int
        @param node_id: Node_id of the thruster to reboot
        '''
        address, register_size, format_char, permission = Const.csr_address['UTILITY']
        self.send_VRCSR_request_packet(
            node_id=int(node_id),
            flags=0x80 | register_size,
            address=address,
            length=register_size,
            payload_bytes=self.checksum_struct(struct.pack('<' + format_char, 0xDEAD))
        )

    def enter_config_mode(self, node_id, close_port=True):
        ''' Enters the m5 thruster's configuration mode
        @type node_id: int
        @param node_id: Node_id of the thruster of interest

        @type close_port: bool
        @param close_port: If true, this object's connection to the serial port will be closed.
            This allows the user to open a serial terminal to use the configuration mode menu.
        '''
        self.reboot_thruster(node_id)
        rospy.sleep(3)   # Time waiting for thruster reboot to take effect
        [self.port.write('+') for i in range(5)]  # 5 plusses are used to trigger entry into config mode

        if close_port:
            self.port.close()
            msg = 'Run the following command to open a serial terminal in configuration mode:\n\'screen {} {}\''
            rospy.loginfo(msg.format(self.port_name, self._baud_rate) + '\nThen press \'?\' to see the menu')
            rospy.loginfo(
                'To continue using this ThrusterPort, call \'{this}.port.open()\' after closing screen.')

    def send_VRCSR_request_packet(self, node_id, flags, address, length, payload_bytes):
        ''' Writes a VideoRay CSR (Control Service Register) packet to the serial port
        Refer to the VRCSR protocol documentation for the specific meaning of each of these
        fields:
        https://github.com/videoray/VRCommsProtocol_doc/raw/master/VR_CSR_Communication_Protocol.doc
        '''
        # Create VR_CSR packet
        header = self.checksum_struct(
            struct.pack('<HBBBB',
                        Const.sync_request,
                        int(node_id),
                        flags,
                        address,
                        length)
        )
        packet = header + payload_bytes

        # Write packet
        self.port.flushInput()
        self.port.write(bytes(packet))

    def parse_VRCSR_response_packet(self, packet):
        ''' Parses the fields of a VRCSR packet
        @type packet: bytearray|string
        @param packet: The response to a VRCSR packet

        @rtype: dict
        @return: Dictionary holding the names and parsed values of packet segments
            Dictionary will contain the header and payload fields excluding checksums.
            It will separate the payload into 'device_type' and 'payload_bytes' fields
        '''
        payload_start_idx = Const.header_size + Const.xsum_size + 1
        header_bytes = packet[:payload_start_idx]
        payload_bytes = packet[payload_start_idx:]
        header_names = ['sync', 'node_id', 'flag', 'address', 'length', 'header_xsum', 'device_type']
        header_fields = struct.unpack('<HBBBB I B', header_bytes)
        response_dict = dict(zip(header_names, header_fields))
        response_dict.pop('header_xsum')
        response_dict['payload_bytes'] = payload_bytes[:-Const.xsum_size]  # Exclude payload xsum
        return response_dict

    def read_register(self, node_id, register):
        ''' Reads a CSR register on a thruster microcontroller
        @type node_id: int
        @param node_id: Node id of the thruster to send the read request

        @type register: string
        @param register Name of the register to read

        @rtype: (int, bytearray)
        @return The interpreted value and raw bytes read from register
        '''
        # Validate input
        if register not in Const.csr_address.keys():
            raise VRCSRException('Unknown register', node_id=node_id, register=register)
        address, register_size, format_char, permission = Const.csr_address[register]
        if 'R' not in permission:
            raise VRCSRException('Register doesn\'t have read permission', node_id=node_id, register=register)

        # Do serial comms
        self.send_VRCSR_request_packet(
            node_id=int(node_id),
            flags=0x80 | register_size,
            address=address,
            length=0,
            payload_bytes=''  # It is important that the null payload xsum is ommitted!
        )
        response_bytearray = self.port.read(Const.header_size + 2 * Const.xsum_size + 1 + register_size)
        rospy.logdebug('Received packet: "' + self.make_hex(response_bytearray), '" on port ' + self.port_name)
        valid, reason = self.validate_packet_integrity(response_bytearray)
        if not valid:
            raise VRCSRException('Response packet invalid', node_id=node_id, register=register, reason=reason)

        # Unpack response packet
        response_dict = self.parse_VRCSR_response_packet(response_bytearray)
        register_bytes = response_dict['payload_bytes']
        register_val = struct.unpack(format_char, register_bytes)[0]

        # Validate packet
        if response_dict['node_id'] != node_id:
            raise VRCSRException('Got response packet from wrong thruster', sent_to=node_id,
                                 received_from=response_dict['node_id'])

        return register_val, self.make_hex(register_bytes)

    def set_register(self, node_id, register, value):
        ''' Sets a CSR register on a thruster microcontroller
        Returns a tuple with the value and raw bytes of the recently changed register
        Note: Can only set registers that are writeable (see comms protocol)
        @type node_id: int
        @param node_id: Node id of the thruster to send a set register request

        @type register: string
        @param register Name of the register to write

        @param value Value to be written to the register

        @rtype: (int, bytearray)
        @return The value and raw bytes read from register after the set request
        '''
        # Validate input
        if register not in Const.csr_address.keys():
            raise VRCSRException('Unknown register', node_id=node_id, register=register)
        address, register_size, format_char, permission = Const.csr_address[register]
        expected_response_length = Const.header_size + Const.xsum_size + 1 + register_size + Const.xsum_size
        if 'W' not in permission:
            raise VRCSRException('Register doesn\'t have write permission', node_id=node_id, register=register)

        # Do serial comms
        self.send_VRCSR_request_packet(
            node_id=int(node_id),
            flags=0x80 | register_size,
            address=address,
            length=register_size,
            payload_bytes=self.checksum_struct(struct.pack('<' + format_char, value))
        )
        response_bytearray = self.port.read(expected_response_length)
        rospy.logdebug('Received packet: "' + self.make_hex(response_bytearray), '" on port ' + self.port_name)
        valid, reason = self.validate_packet_integrity(response_bytearray)
        if not valid:
            raise VRCSRException('Response packet invalid', node_id=node_id, register=register, reason=reason)

        # Unpack response packet
        response_dict = self.parse_VRCSR_response_packet(response_bytearray)
        register_bytes = response_dict['payload_bytes']
        register_val = struct.unpack('<' + format_char, register_bytes)[0]

        # Validate packet
        if response_dict['node_id'] != node_id:
            raise VRCSRException('Got response packet from wrong thruster', sent_to=node_id,
                                 received_from=response_dict['node_id'])

        return register_val, self.make_hex(register_bytes)

    def set_registers_from_dict(self, node_id, reg_dict):
        ''' Sets thruster registers specified in a dictionary
        @type node_id: int
        @param node_id: Node id of the thruster to send a set register requests

        @type reg_dict: dict
        @param reg_dict Dictionary storing register name, register value pairs
            Register names must are specified in the protocol.py file in this directory.
        '''
        assert isinstance(reg_dict, dict)

        for register, value in reg_dict.items():
            try:
                self.set_register(node_id, register, value)

                value_after = self.read_register(node_id, register)[0]
                if not np.isclose(value, value_after):
                    msg = 'Tried to set register "{}" on thruster {} to {}. It remained at value {}.'
                    rospy.logwarn(msg.format(register, node_id, value, value_after))
            except VRCSRException as e:
                msg = 'VRCSRException raised while attempting to set register {} on thruster {}. {}'
                rospy.logwarn(msg.format(register, node_id, e))

    def _make_thrust_payload(self, node_id, effort):
        '''Construct a payload that commands a thrust
        @type node_id: int
        @param node_id: Node id of the thruster to send a set register requests

        @type thrust: float
        @param thrust: Normalized power value in [-1, 1]
        '''
        send_effort = np.clip(effort, -1, 1)
        payload = self.checksum_struct(
            struct.pack(
                '<BBf',
                Const.propulsion_command,
                node_id,
                send_effort
            )
        )
        return payload

    def send_thrust_msg(self, node_id, effort):
        ''' Construct and send a message to set motor effort
        @type node_id: int
        @param node_id: Node id of the thruster to send a set register requests

        @type thrust: float
        @param thrust: Normalized power value in [-1, 1]
        '''
        self.send_VRCSR_request_packet(
            node_id=int(node_id),
            flags=Const.response_thruster_standard,
            address=Const.addr_custom_command,
            length=6,  # 2 bytes and a float: (propulsion_command, node_id, effort)
            payload_bytes=self._make_thrust_payload(int(node_id), effort)
        )

    def parse_thrust_response(self, payload_bytes):
        ''' Parses the response to the standard propulsion command (0xAA)
        @type payload_bytes: bytearray
        @param payload_bytes: The payload portion of a response to a propulsion command
        '''
        names = ['rpm', 'bus_v', 'bus_i', 'temp', 'fault']
        fields = struct.unpack('<ffffB', payload_bytes)
        return dict(zip(names, fields))

    def command_thruster(self, name, effort):
        ''' Sets effort value on motor controller
        @type name: string
        @param name Name of the thruster to be commanded as specified on the thruster layout

        @type effort: float
        @param effort Normalized power value ([-1.0, 1.0]) to set on the motor controller
        '''

        # Perform availability checks
        if name not in self.thruster_info.keys():
            raise UndeclaredThrusterException(name=name)
        node_id = self.thruster_info[name].node_id

        # Don't try to send command packet if line is busy
        t0 = rospy.Time.now()
        while self.serial_busy and rospy.Time.now() - t0 < rospy.Duration(self._wait_for_line_timeout):
            rospy.sleep(0.001)
        if self.serial_busy and name in self.online_thruster_names:  # Don't raise if thruster is offline
            rospy.logwarn('{} timed out waiting on busy serial line (waited {} s)'.
                          format(name, (rospy.Time.now() - t0).to_sec()))

        # Might not need to do this mutex around tx and rx together
        self.serial_busy = True  # Take ownership of line
        t0 = rospy.Time.now()

        # Send thrust command
        self.send_thrust_msg(node_id, effort)
        self.command_tx_count[name] = self.command_tx_count[name] + 1

        # Parse thrust response
        response_bytearray = self.port.read(Const.thrust_response_length)
        rospy.logdebug('Received packet: "' + self.make_hex(response_bytearray), '" on port ' + self.port_name)
        t1 = rospy.Time.now()
        self.serial_busy = False  # Release line
        valid, reason = self.validate_packet_integrity(response_bytearray)
        response_dict = self.parse_VRCSR_response_packet(response_bytearray) if valid else None

        # Keep track of thrusters going offline or coming back online
        if not valid:
            if name in self.online_thruster_names:
                if self.CONFIDENT_MODE:
                    rospy.logdebug_throttle(1, 'Got invalid response from thruster {} (packet: {})'.format(node_id,
                                            self.make_hex(response_bytearray)))
                else:
                    self.online_thruster_names.remove(name)
        else:
            if name not in self.online_thruster_names:
                self.online_thruster_names.add(name)

            # Keep track of latency
            total_latency = (t1 - t0) + self.command_latency_avg[name] * self.status_rx_count[name]
            self.status_rx_count[name] = self.status_rx_count[name] + 1
            self.command_latency_avg[name] = total_latency / self.status_rx_count[name]

        ret = self.parse_thrust_response(response_dict['payload_bytes']) if valid else None
        if ret is not None:
            ret['command_tx_count'] = self.command_tx_count[name]
            ret['status_rx_count'] = self.status_rx_count[name]
            ret['command_latency_avg'] = self.command_latency_avg[name].to_sec()
        return ret


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
    node_id = thruster_definitions[thruster_name]['node_id']

    print'Test thruster comm over port {}, node_id {}, thruster name {}'.format(
        port_info['port'],
        node_id,
        thruster_name
    )

    tp = ThrusterPort(port_info, thruster_definitions)
    print tp.command_thruster(thruster_name, 0.04)
