#!/usr/bin/env python
import numpy as np
import sys
import rospy
import rospkg
import rosparam
from sub8_thruster_comm import thruster_comm_factory
from mil_misc_tools.text_effects import *
from mil_misc_tools.terminal_input import get_ch

__author__ = 'David Soto'

rospack = rospkg.RosPack()

fprint = FprintFactory(title='thruster_spin_test', auto_bold=False).fprint

def ports_from_layout(layout):
    '''Load and handle the thruster bus layout'''
    port_dict = {}
    thruster_definitions = layout['thrusters']

    for port_info in layout['thruster_ports']:
        port = port_info['port']
        thruster_names = port_info['thruster_names']
        msg = "Instantiating thruster_port:\n\tName: {}".format(port)
        thruster_port = thruster_comm_factory(port_info, thruster_definitions, fake=False)

        # Add the thrusters to the thruster dict
        for name in thruster_names:
            port_dict[name] = thruster_port

        msg += "\n\tMotor id's on port: {}".format(thruster_port.motor_ids_on_port)
        fprint(msg)

    return port_dict

rospy.init_node('thruster_spin_test')

sub8_thruster_mapper = rospack.get_path('sub8_thruster_mapper')
thruster_layout = rosparam.load_file(sub8_thruster_mapper + '/config/thruster_layout.yaml')[0][0]
thruster_ports = ports_from_layout(thruster_layout)
names_from_motor_id = {0: 'FLH', 1: 'FLV', 2: 'FRH', 3: 'FRV',
                       4: 'BLH', 5: 'BLV', 6: 'BRH', 7: 'BRV'}
active_thrusters = set()

usage_msg = \
'''
Welcome to David's thruster_spin_test tool.
\tInstructions:
\t* press up or down to control the thrust commanded
\t* press the thruster id to toggle spinning for that thruster (0-indexed, 8 for all)
\t* press 'Q' to exit
'''
fprint(usage_msg)

thrust = 0.5  # Normalized thrust in [0, 1]
key = None

def command_thrusters(timer_event):
    thrusters_to_command = active_thrusters.copy()
    for motor_id in thrusters_to_command:
        name = names_from_motor_id[motor_id]
        thruster_ports[name].command_thruster(name, thrust)

timer = rospy.Timer(period=rospy.Duration(0.01), callback=command_thrusters)

interesting = rospy.Rate(20)
while not rospy.is_shutdown():
    interesting.sleep()
    key = get_ch()

    # quitting
    if key == 'q':
        fprint("Quitting")
        sys.exit()

    # modify thrust
    if key == '\x1b':              # ESC sequence
        if get_ch() == '[':        # Got an arrow key
            arrow_type = get_ch()
            if arrow_type == 'A':  # UP key
                thrust = thrust + 0.1
            if arrow_type == 'B':  # DOWN key
                thrust = thrust - 0.1
        else:                      # Got other ESC sequence
            continue
        thrust = np.clip(thrust, 0.0, 1.0)
        fprint("Thrust: {}".format(thrust))
        continue

    # toggle active thrusters
    if ord(key) in range(ord('0'), ord('9')):
        motor_id = int(key)
        if motor_id is 8:
            if active_thrusters == {0, 1, 2, 3, 4, 5, 6, 7}:
                active_thrusters = set()
            elif active_thrusters == set():
                active_thrusters = {0, 1, 2, 3, 4, 5, 6, 7}
            else:
                active_thrusters = set()
        else:
            if motor_id in active_thrusters:
                active_thrusters.remove(motor_id)
            else:
                active_thrusters.add(motor_id)
        fprint("Active thrusters: {}".format(list(active_thrusters)))

