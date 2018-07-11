#!/usr/bin/env python
from __future__ import division
import numpy as np
import sys
import serial
import rospy
import rospkg
import rosparam
import sub8_thruster_comm as thrust_comm
from sub8_exception import SubjuGatorException
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
            thruster_port = thrust_comm.thruster_comm_factory(port_info, thruster_definitions, fake=False)

            # Add the thrusters to the thruster dict
            for name in thruster_names:
                port_dict[name] = thruster_port

            s = list(thruster_port.online_thruster_names)
            fprint(msg.reset.text('\n\tName: ').set_yellow.text(port).reset
                   .text('\n\tMotor id\'s on port: ').set_cyan.bold(s).reset)

        except SubjuGatorException:
            pass

    return port_dict


rospy.init_node('thruster_spinner')

# Connect to thrusters
sub8_thruster_mapper = rospack.get_path('sub8_thruster_mapper')
thruster_layout = rosparam.load_file(sub8_thruster_mapper + '/config/thruster_layout.yaml')[0][0]
thruster_ports = ports_from_layout(thruster_layout)
if thruster_ports == {}:
    fprint('Unable to connect to any thruster ports. Quitting.')
    sys.exit()

help_msg = \
    '''
Welcome to David's ThrusterSpinner.\n
Help:
* press 'i' to toggle Individual mode on or off
  + Individual mode allows setting thrust values for individual thrusters:
    - press a motor_id to select a thruster
    - press up or down to change the thrust for that specific thruster
    - press '*' to clear reset all thrusts to 0
  + Having Individual mode off allows all thrusters to be commanded with a single thrust value:
    - press a motor_id to toggle spinning for that thruster (* for all)
    - press up or down to control the thrust commanded
* press 'd' for thruster discovery on a specific port (even if not in layout)
* press 'h' to see this help text again
* press 'q' to exit
'''
fprint(help_msg)

names_from_motor_id = {0: 'FLH', 1: 'FLV', 2: 'FRH', 3: 'FRV',
                       4: 'BLH', 5: 'BLV', 6: 'BRH', 7: 'BRV'}
thruster_keys = [ord(str(x)) for x in names_from_motor_id]  # Will break if there is a motor_if not in [0, 9]
unavailable_thrusters = []

individual_mode = False
key = None
STEP = 1 / 64  # Effort
ALL_KEY = '*'

# For individual mode on
selected_idx = 0
individual_thrusts = dict(zip(names_from_motor_id.keys(), np.zeros(len(names_from_motor_id))))

# For individual mode off
active_thrusters = set()
common_thrust = 0.0  # Normalized thrust in [-1, 1]


def check_for_thrusters():
    ''' Checks for a response from thrusters on a given port '''
    # Stop all thrusters
    for key in individual_thrusts.keys():
        individual_thrusts[key] = 0

    declared_ports = []
    for port in thruster_ports.values():
        if port not in declared_ports:
            declared_ports.append(port)

    declared_ports.sort(key=lambda p: p.port_name)

    fprint('Declared ports:')
    for i, p in enumerate(declared_ports):
        print '{})'.format(i), p.port_name

    fprint('Select a port by index: ')
    sel = int(raw_input())

    start_id = 0
    end_id = 127
    fprint('Checking for thrusters with id\'s in [{}, {}]'.format(start_id, end_id))

    found_motor_ids, avg_turnaround_time = declared_ports[sel].get_node_ids_on_port(start_id, end_id)
    fprint('Responding motor ids:\t\t\t{}'.format(te.Printer().set_cyan.bold(found_motor_ids)))
    fprint('Average packet turnaround time:\t{} seconds'.format(te.Printer().set_cyan.bold(avg_turnaround_time)))


def command_thrusters(timer_event):
    ''' Sets the effort values for the motorcontrollers if comms are available '''
    thrusters_to_command = names_from_motor_id.keys() if individual_mode else active_thrusters.copy()
    for motor_id in thrusters_to_command:
        name = names_from_motor_id[motor_id]
        try:
            if individual_mode:
                thruster_ports[name].command_thruster(name, individual_thrusts[motor_id])
            else:
                thruster_ports[name].command_thruster(name, common_thrust)
        except thrust_comm.UnavailableThrusterException as e:
            if e.thruster_name not in unavailable_thrusters:  # Only print once
                unavailable_thrusters.append(e.thruster_name)
                fprint(e, msg_color='red')
            else:
                pass
        except serial.SerialException as e:
            fprint(e)

# Command thrusters at a regular interval
timer = rospy.Timer(period=rospy.Duration(0.01), callback=command_thrusters)

interesting = rospy.Rate(20)  # As per Forrest Voight
while not rospy.is_shutdown():
    interesting.sleep()
    key = get_ch()

    # Quitting
    if key == 'q':
        fprint('Quitting')
        sys.exit()

    # Toggle individual mode
    if key == 'i':
        individual_mode = not individual_mode
        active_thrusters = set()
        for key in individual_thrusts.keys():
            individual_thrusts[key] = 0
        fprint('Individual mode: {}'.format('ON' if individual_mode else 'OFF'))
        continue

    # Modify thrust
    if key == '\x1b':              # ESC sequence
        if get_ch() == '[':        # Got an arrow key
            arrow_type = get_ch()
            if arrow_type == 'A':  # UP key
                if individual_mode:
                    individual_thrusts[selected_idx] = np.clip(individual_thrusts[selected_idx] + STEP, -1, 1)
                    fprint('Thrusts: {}'.format(individual_thrusts))
                else:
                    common_thrust = np.clip(common_thrust + STEP, -1, 1)
                    fprint('Common Thrust: {}'.format(common_thrust))
            if arrow_type == 'B':  # DOWN key
                if individual_mode:
                    individual_thrusts[selected_idx] = np.clip(individual_thrusts[selected_idx] - STEP, -1, 1)
                    fprint('Thrusts: {}'.format(individual_thrusts))
                else:
                    common_thrust = np.clip(common_thrust - STEP, -1, 1)
                    fprint('Common Thrust: {}'.format(common_thrust))
        else:                      # Got other ESC sequence
            continue

        continue

    # Active or select thrusters
    if ord(key) in thruster_keys or ord(key) == ord(ALL_KEY):
        if ord(key) == ord(ALL_KEY):  # Toggling all thrusters
            if individual_mode:
                for key in individual_thrusts.keys():
                    individual_thrusts[key] = 0
                fprint('Thrusts: {}'.format(individual_thrusts))
                continue
            else:
                if active_thrusters == set():
                    active_thrusters = {0, 1, 2, 3, 4, 5, 6, 7}
                else:
                    if not individual_mode:
                        active_thrusters = set()
        else:  # Toggling or selecting a thruster
            motor_id = int(key)
            if individual_mode:
                if selected_idx in names_from_motor_id.keys():
                    selected_idx = motor_id
            else:
                if motor_id in active_thrusters:
                    active_thrusters.remove(motor_id)
                else:
                    active_thrusters.add(motor_id)
        if individual_mode:
            fprint('Selected motor_id {}'.format(selected_idx))
        else:
            fprint('Active thrusters: {}'.format(list(active_thrusters)))
        continue

    # Perform thruster discovery
    if key == 'd':
        try:
            check_for_thrusters()
        except BaseException as e:
            fprint(e)
        continue

    # Display help
    if key == 'h':
        fprint(help_msg)
