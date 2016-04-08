#!/usr/bin/env python
import rospy
import rosparam
import rospkg
from sub8_actuator_driver.srv import SetValve

import serial
import binascii
import struct
import time
import yaml
import os

rospack = rospkg.RosPack()
VALVES_FILE = os.path.join(rospack.get_path('sub8_actuator_driver'), 'valves.yaml')

class ActuatorDriver():
    '''
    Allows high level ros code to interface with Daniel's pneumatics board.

    For dropper and grabber, call service with True or False to open or close.
    For shooter, sending a True signal will pulse the valve.

    TODO: Wait for response to return True or False. The board does not actually send
        back responses right now.
    '''
    def __init__(self, port, baud=9600):
        self.load_yaml()

        rospy.init_node("actuator_driver")

        #print self.actuators

        self.ser = serial.Serial(port=port, baudrate=baud, timeout=.3)

        # Reset all valves
        rospy.loginfo("Reseting valves.")
        for actuator_key in self.actuators:
            actuator = self.actuators[actuator_key]
            for valve_ports in actuator['ports']:
                valve_port = actuator['ports'][valve_ports]
                self.send_data(valve_port['id'],valve_port['default'])

        rospy.loginfo("Valves ready to go.")
        
        rospy.Service('~actuate', SetValve, self.got_service_request)

        rospy.spin()

    def got_service_request(self, srv):
        '''
        Find out what actuator needs to be changed and how to change it with the valves.yaml file.
        '''
        try:
            this_valve = self.actuators[srv.actuator]
        except:
            rospy.logerr("'%s' not found in valves.yaml and therefore no configuration has been set for that actuator."%srv.actuator)
            return False

        if this_valve['type'] == 'pulse':
            # We want to pulse the port from the default value for the desired pulse_time then go back to the default value.
            port_to_pulse = this_valve['ports']['port']
            
            port_id = port_to_pulse['id']
            default_value = port_to_pulse['default']
            pulse_value = not default_value

            pulse_time = this_valve['pulse_time']

            self.send_data(port_id,pulse_value)
            rospy.sleep(pulse_time)
            self.send_data(port_id,default_value)

        elif this_valve['type'] == 'set':
            # If the desired action is to open, set the open valve to true and the closed false (and visa versa for closing).
            set_open = srv.opened

            open_port = this_valve['ports']['open_port']
            open_id = open_port['id']

            close_port = this_valve['ports']['close_port']
            close_id = close_port['id']

            if set_open:
                # This may be too fast for the serial interface, maybe add a break if there are problems.
                self.send_data(open_id, True)
                self.send_data(close_id, False)
            else:
                self.send_data(open_id, False)
                self.send_data(close_id, True)

        return True

    def send_data(self, port, state):
        # Calculate checksum and send data to board.
        # A true state tells the pnuematics controller to allow air into the tube.
        base_code = 0x20
        op_code = port * 2 + base_code
        if state: op_code += 1
        chksum = op_code ^ 0xFF

        data = struct.pack("BB", op_code, chksum)
        
        rospy.loginfo("Writing: %s. Chksum: %s." % (hex(op_code), hex(chksum)))
        #self.ser.write(data)

    def load_yaml(self):
        with open(VALVES_FILE, 'r') as f:
            self.actuators = yaml.load(f)

    def parse_response(self):
        # The board doesn't currently respond with data.
        response = struct.unpack("BB", self.ser.read(2))
    
        data = response[0]
        chksum = response[1] ^ 0xFF
        
        if data != chksum:
            print "CHKSUM NOT MATCHING."
            return False
        
        return True

    def ping(self):
        # Not implmented
        print "ping"
        self.ser.write(bytes(0x1234))

if __name__ == "__main__":
    a = ActuatorDriver(rospy.get_param('~/actuator_driver/port'), 9600)