#!/usr/bin/env ipython
# Using ipython because tab completion makes using the shell much easier

# flake8: noqa
# Not doing formatting checks because there will be many unused declared objects and
# imports since this is an interactive shell

print "\nWelcome to David's M5 Thruster Shell\n"
import rospy
print 'Imported rospy'
import sub8_thruster_comm.thruster_comm as tc
print 'Imported sub8_thruster_comm.thruster_comm as tc'
import sub8_thruster_comm.protocol as tp
print 'Imported sub8_thruster_comm.protocol as tp'
import rosparam
print 'Imported rosparam'
import numpy as np
print 'Imported numpy as np'
import os
print 'Imported os'

rospy.init_node('m5_debug_shell')
rosparam.load_file(os.environ['CATKIN_DIR'] + '/src/SubjuGator/gnc/sub8_thruster_mapper/config/thruster_layout.yaml')[0][0]
layout = rosparam.load_file(os.environ['CATKIN_DIR'] + '/src/SubjuGator/gnc/sub8_thruster_mapper/config/thruster_layout.yaml')[0][0]

port_defs, thruster_defs = layout['thruster_ports'], layout['thrusters']
print "Port definitions and thruster definitions available as 'port_defs' and 'thruster_defs' respectively"
tports = []
for i in range(4):
    t = None
    try:
        t = tc.ThrusterPort(port_defs[i], thruster_defs)
    except tc.VRCSRException as e:
        print e
    tports.append(t)

t0, t1, t2, t3 = tports

print "Thrusters may be accessed through the list 'tports' or individually through 't0', 't1', t2', 't3'"
print "If unsure of how to use a method, use the '?' operator on it instead of the call operator"
