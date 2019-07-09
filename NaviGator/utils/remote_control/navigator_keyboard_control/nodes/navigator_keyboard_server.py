#!/usr/bin/env python

'''
Keyboard Server: The keyboard server enables a client to control NaviGator
using a remote keyboard. It runs a service that responds to KeyboardControl
service requests. It manages the currently locked client through the use of
UUIDs. Only keys received from the client with the locked UUID will be
executed.
'''


import curses
import uuid

from navigator_msgs.srv import KeyboardControl
from remote_control_lib import RemoteControl
import rospy


__author__ = "Anthony Olive"
__maintainer__ = "Anthony Olive"
__email__ = "anthony@iris-systems.net"
__copyright__ = "Copyright 2016, MIL"
__license__ = "MIT"


rospy.init_node("keyboard_server")


class KeyboardServer(object):

    def __init__(self):
        self.force_scale = rospy.get_param("/joystick_wrench/force_scale", 600)
        self.torque_scale = rospy.get_param("/joystick_wrench/torque_scale", 500)

        self.remote = RemoteControl("keyboard", "/wrench/keyboard")
        rospy.Service("/keyboard_control", KeyboardControl, self.key_recieved)

        # Initialize this to a random UUID so that a client without a UUID cannot authenticate
        self.locked_uuid = uuid.uuid4().hex

        # This maps each key to a remote control function
        self.key_mappings = {ord('k'): lambda: self.remote.toggle_kill(),
                             ord('K'): lambda: self.remote.kill(),
                             ord('h'): lambda: self.remote.station_hold(),
                             ord('j'): lambda: self.remote.select_rc_control(),
                             ord('e'): lambda: self.remote.select_emergency_control(),
                             ord('b'): lambda: self.remote.select_keyboard_control(),
                             ord('u'): lambda: self.remote.select_autonomous_control(),
                             ord('c'): lambda: self.remote.select_next_control(),
                             ord('r'): lambda: self.remote.shooter_load(),
                             ord('f'): lambda: self.remote.shooter_fire(),
                             ord('t'): lambda: self.remote.shooter_cancel(),
                             ord('w'): lambda: self.remote.publish_wrench(self.force_scale, 0, 0),
                             ord('s'): lambda: self.remote.publish_wrench(-self.force_scale, 0, 0),
                             ord('a'): lambda: self.remote.publish_wrench(0, self.force_scale, 0),
                             ord('d'): lambda: self.remote.publish_wrench(0, -self.force_scale, 0),
                             curses.KEY_LEFT: lambda: self.remote.publish_wrench(0, 0, self.torque_scale),
                             curses.KEY_RIGHT: lambda: self.remote.publish_wrench(0, 0, -self.torque_scale)
                             }
        self.movement_keys = [ord('w'), ord('s'), ord('a'), ord('d'), curses.KEY_LEFT, curses.KEY_RIGHT]

    def key_recieved(self, req):
        '''
        This function handles the process of locking control of the service to
        one client. If an 'L' is received, a UUID is generated for the client
        and control of the service is locked to it.
        '''

        # If the key pressed was L, locks control of the service to the clinet's UUID
        if (req.keycode == 76):

            # Generates a new UUID for the client if it does not already have one
            if (req.uuid is ''):
                self.locked_uuid = uuid.uuid4().hex
            else:
                self.locked_uuid = req.uuid
            return {"generated_uuid": self.locked_uuid, "is_locked": True}

        # If the key was from the client with locked control, pass it to execution
        elif (req.uuid == self.locked_uuid):
            self.execute_key(req.keycode)
            return {"generated_uuid": '', "is_locked": True}

        # Ignore keys sent by a client that has not locked control of the service
        else:
            return {"generated_uuid": '', "is_locked": False}

    def execute_key(self, key):
        '''
        Executes a remote control action based on the key that was received.
        '''
        if (key in self.key_mappings):
            self.key_mappings[key]()

        # If no motion key was received, clear the wrench
        if (key not in self.movement_keys):
            self.remote.clear_wrench()


if __name__ == "__main__":
    keyboard = KeyboardServer()
    rospy.spin()
