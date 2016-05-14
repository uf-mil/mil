#!/usr/bin/env python
import rospy
import rosparam
import rospkg

from sub8_msgs.srv import SetValve
from sub8_ros_tools import thread_lock
from sub8_alarm import AlarmBroadcaster

import threading
import serial
import binascii
import struct
import time
import yaml
import os

rospack = rospkg.RosPack()
VALVES_FILE = os.path.join(rospack.get_path('sub8_actuator_driver'), 'valves.yaml')
lock = threading.Lock()


class ActuatorDriver():
    '''
    Allows high level ros code to interface with Daniel's pneumatics board.

    For dropper and grabber, call service with True or False to open or close.
    For shooter, sending a True signal will pulse the valve.

    TODO: Add a function to try and reconnect to the serial port if we lose connection.
    '''
    def __init__(self, port, baud=9600):
        self.load_yaml()

        rospy.init_node("actuator_driver")

        alarm_broadcaster = AlarmBroadcaster()
        self.disconnection_alarm = alarm_broadcaster.add_alarm(
            name='actuator_board_disconnect',
            action_required=True,
            severity=0
        )
        self.packet_error_alarm = alarm_broadcaster.add_alarm(
            name='packet_error',
            action_required=False,
            severity=2
        )

        self.ser = serial.Serial(port=port, baudrate=baud, timeout=None)
        self.ser.flushInput()

        # Reset all valves
        rospy.loginfo("Resetting valves.")
        for actuator_key in self.actuators:
            actuator = self.actuators[actuator_key]
            for valve_ports in actuator['ports']:
                valve_port = actuator['ports'][valve_ports]
                self.send_data(valve_port['id'], valve_port['default'])

        rospy.loginfo("Valves ready to go.")

        rospy.Service('~actuate', SetValve, self.got_service_request)
        rospy.Service('~actuate_raw', SetValve, self.set_raw_valve)

        r = rospy.Rate(.2)  # hz
        while not rospy.is_shutdown():
            # Heartbeat to make sure the board is still connected.
            r.sleep()
            self.ping()

    @thread_lock(lock)
    def set_raw_valve(self, srv):
        '''
        Set the valves manually so you don't have to have them defined in the YAML.

        Service parameters:
            actuator: PORT_ID
            opened: OPENED
        '''
        self.send_data(int(srv.actuator), srv.opened)
        return True

    @thread_lock(lock)
    def got_service_request(self, srv):
        '''
        Find out what actuator needs to be changed and how to change it with the valves.yaml file.
        '''
        try:
            this_valve = self.actuators[srv.actuator]
        except:
            rospy.logerr("'%s' not found in valves.yaml so no configuration has been set for that actuator." % srv.actuator)
            return False

        if this_valve['type'] == 'pulse':
            # We want to pulse the port from the default value for the desired pulse_time then go back to the default value.
            open_port = this_valve['ports']['open_port']
            open_id = open_port['id']
            open_default_value = open_port['default']

            close_port = this_valve['ports']['close_port']
            close_id = close_port['id']
            close_default_value = close_port['default']

            open_pulse_value = not open_default_value
            close_pulse_value = not close_default_value

            pulse_time = this_valve['pulse_time']

            self.send_data(open_id, open_pulse_value)
            self.send_data(close_id, close_pulse_value)
            rospy.sleep(pulse_time)
            self.send_data(open_id, open_default_value)
            self.send_data(close_id, close_default_value)

        elif this_valve['type'] == 'set':
            # If the desired action is to open, set the open valve to true and the closed false (and visa versa for closing).
            open_port = this_valve['ports']['open_port']
            open_id = open_port['id']

            close_port = this_valve['ports']['close_port']
            close_id = close_port['id']

            if srv.opened:
                self.send_data(open_id, True)
                self.send_data(close_id, False)
            else:
                self.send_data(open_id, False)
                self.send_data(close_id, True)

        return True

    @thread_lock(lock)
    def ping(self):
        rospy.loginfo("ping")

        ping = 0x10
        chksum = ping ^ 0xFF
        data = struct.pack("BB", ping, chksum)
        self.ser.write(data)
        if not self.parse_response(None):
            rospy.logwarn("The board appears to be disconnected, trying again in 3 seconds.")
            self.disconnection_alarm.raise_alarm(
                problem_description='The board appears to be disconnected.'
            )
            rospy.sleep(3)

    def send_data(self, port, state):
        '''
        Infomation on communication protcol:

        Sending bytes: send byte (command), then send byte XOR w/ 0xFF (checksum)
        Receiving bytes: receive byte (response), then receive byte XOR w/ 0xFF (checksum)

        Base opcodes:
        - 0x10 ping
        - 0x20 open valve (allow air flow)
        - 0x30 close valve (prevent air flow)
        - 0x40 read switch

        - To 'ping' board (check if board is operational): send 0x10, if operational, will reply with 0x11.
        - To open valve (allow air flow): send 0x20 + valve number (ex. 0x20 + 0x04 (valve #4) = 0x24 <-- byte to send),
        will reply with 0x01.
        - To close valve (prevent air flow): send 0x30 + valve number (ex. 0x30 + 0x0B (valve #11) = 0x3B <-- byte to send),
        will reply with 0x00.
        - To read switch: send 0x40 + switch number (ex. 0x40 + 0x09 (valve #9) = 0x49 <-- byte to send), will reply with 0x00
        if switch is open (not pressed) or 0x01 if switch is closed (pressed).
        '''
        if port == -1:
            return

        # Calculate checksum and send data to board.
        # A true state tells the pnuematics controller to allow air into the tube.
        open_base_code = 0x20
        closed_base_code = 0x30
        op_code = port
        op_code += open_base_code if state else closed_base_code

        chksum = op_code ^ 0xFF
        data = struct.pack("BB", op_code, chksum)
        rospy.loginfo("Writing: %s. Chksum: %s." % (hex(op_code), hex(chksum)))

        try:
            self.ser.write(data)
        except:
            self.disconnection_alarm.raise_alarm(
                problem_description="Actuator board serial connection has been terminated."
            )
            return False

        self.parse_response(state)

    def parse_response(self, state):
        '''
        State can be True, False, or None for Open, Closed, or Ping signals respectively.
        This will return True or False based on the correctness of the message.

        TODO: Test with board and make sure all serial signals can be sent and received without need delays.
        '''
        try:
            response = struct.unpack("BB", self.ser.read(2))
        except:
            self.disconnection_alarm.raise_alarm(
                problem_description="Actuator board serial connection has been terminated."
            )
            return False

        data = response[0]
        chksum = response[1]
        rospy.loginfo("Received: %s. Chksum: %s." % (hex(data), hex(chksum)))

        # Check if packet is intact.
        if data != chksum ^ 0xFF:
            rospy.logerr("CHKSUM NOT MATCHING.")

            self.packet_error_alarm.raise_alarm(
                problem_description="Actuator board checksum error.",
                parameters={
                    'fault_info': {'expected': data ^ 0xFF, 'got': response[1]}
                }
            )
            return False

        # Check if packet data is correct.
        if state is None:
            if data == 0x11:
                return True
            return False
        if data == state:
            return True

        self.packet_error_alarm.raise_alarm(
            problem_description="Incorrect response to command.",
            parameters={
                'fault_info': {'expected_state': state, 'got_state': data == 1},
                'note': 'True indicates an open port, false indicates a closed port.'
            }
        )
        return False

    def load_yaml(self):
        with open(VALVES_FILE, 'r') as f:
            self.actuators = yaml.load(f)

if __name__ == "__main__":
    a = ActuatorDriver(rospy.get_param('~/actuator_driver/port'), 9600)
