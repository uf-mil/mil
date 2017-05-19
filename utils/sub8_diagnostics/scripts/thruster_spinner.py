#!/usr/bin/env python
import numpy as np
import sys
import serial
import rospy
import rospkg
import rosparam
from sub8_thruster_comm import thruster_comm_factory
import mil_misc_tools.text_effects as te
from mil_misc_tools.terminal_input import get_ch

__author__ = 'David Soto'

rospack = rospkg.RosPack()

fprint = te.FprintFactory(title='ThrusterSpinner').fprint


def ports_from_layout(layout):
    '''Load and handle the thruster bus layout'''
    port_dict = {}
    thruster_definitions = layout['thrusters']

    msg = te.Printer('Instantiating thruster_port:')
    for port_info in layout['thruster_ports']:
        try:
            port = port_info['port']
            thruster_names = port_info['thruster_names']
            thruster_port = thruster_comm_factory(port_info, thruster_definitions, fake=False)

            # Add the thrusters to the thruster dict
            for name in thruster_names:
                port_dict[name] = thruster_port

            s = str(np.sort([x[1] for x in thruster_port.thruster_dict.items()]).tolist())
            fprint(msg.reset.text("\n\tName: ").set_yellow.text(port).reset
                   .text("\n\tMotor id's on port: ").set_cyan.bold(s).reset)

        except serial.serialutil.SerialException as e:
            pass

    return port_dict

rospy.init_node('thruster_spinner')

sub8_thruster_mapper = rospack.get_path('sub8_thruster_mapper')
thruster_layout = rosparam.load_file(sub8_thruster_mapper + '/config/thruster_layout.yaml')[0][0]
thruster_ports = ports_from_layout(thruster_layout)
if thruster_ports == {}:
    fprint('Unable to connect to any thruster ports. Quitting.')
    sys.exit()
names_from_motor_id = {0: 'FLH', 1: 'FLV', 2: 'FRH', 3: 'FRV',
                       4: 'BLH', 5: 'BLV', 6: 'BRH', 7: 'BRV'}
usage_msg = \
    '''
Welcome to David's thruster_spin_test tool.
\tInstructions:
\t* press up or down to control the thrust commanded
\t* press the thruster id to toggle spinning for that thruster (0-indexed, 8 for all)
\t* press 'C' to check for thrusters on a specific port (even if not in layout)
\t* press 'H' to see this usage message again
\t* press 'Q' to exit
'''
fprint(usage_msg)

active_thrusters = set()

thrust = 0.0  # Normalized thrust in [-1, 1]
step = 0.05

key = None


def check_for_thrusters():
    ''' Checks for a response from thrusters on a given port '''
    active_thrusters = set()
    fprint("Active thrusters: {}".format(list(active_thrusters)))

    declared_ports = []
    for port in thruster_ports.values():
        if port not in declared_ports:
            declared_ports.append(port)

    declared_ports.sort(key=lambda p: p.port_name)

    fprint('Declared ports:')
    for i, p in enumerate(declared_ports):
        print "{})".format(i), p.port_name

    fprint("Select a port by index: ")
    sel = int(raw_input())

    start_id = 0
    end_id = 10
    fprint("Checking for thrusters with id's in [{}, {}]".format(start_id, end_id))

    found_motor_ids, avg_turnaround_time = \
        declared_ports[sel].get_motor_ids_on_port(start_id, end_id)
    fprint("Responding motor ids:\t\t\t{}".format(te.Printer().set_cyan.bold(found_motor_ids)))
    fprint("Average packet turnaround time:\t{} seconds".format(te.Printer().set_cyan.bold(avg_turnaround_time)))


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
                thrust = thrust + step
            if arrow_type == 'B':  # DOWN key
                thrust = thrust - step
        else:                      # Got other ESC sequence
            continue
        thrust = np.clip(thrust, -1.0, 1.0)
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

    # Perform thruster check
    if key == 'c':
        try:
            check_for_thrusters()
        except BaseException as e:
            fprint(e)
        continue

    # Display help
    if key == 'h':
        fprint(usage_msg)
