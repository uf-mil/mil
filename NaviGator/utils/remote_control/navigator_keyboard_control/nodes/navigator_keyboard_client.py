#!/usr/bin/env python

'''
Keyboard Client: The keyboard client connects to the keyboard server in order
to control NaviGator using a remote keyboard. It is able to lock control of the
keyboard service to itself by obtaining a UUID that identifies it's service
calls. Curses is used to display a basic UI in the terminal that gives the user
useful feedback and captures key presses to be sent to the server.
'''


from __future__ import division

import curses

from navigator_msgs.srv import KeyboardControl
import rospy


__author__ = "Anthony Olive"
__maintainer__ = "Anthony Olive"
__email__ = "anthony@iris-systems.net"
__copyright__ = "Copyright 2016, MIL"
__license__ = "MIT"


rospy.init_node("keyboard_client", anonymous=True)


class KeyboardClient():

    def __init__(self, stdscr):
        self.screen = stdscr
        self.num_lines = 10
        self.screen.nodelay(True)
        curses.curs_set(0)

        self.uuid = ''
        self.is_locked = False

        self.keyboard_server = rospy.ServiceProxy("/keyboard_control", KeyboardControl)

        self.help_menu = ["Lock:                 L          ",
                          "Quit:                 q          ",
                          "Toggle Kill:          k          ",
                          "Force Kill:           K          ",
                          "Station Hold:         h          ",
                          "Autonomous Control:   u          ",
                          "Joystick Control:     j          ",
                          "Keyboard Control:     b          ",
                          "Cycle Control Device: c          ",
                          "Shooter Load:         r          ",
                          "Shooter Fire:         f          ",
                          "Shooter Cancel:       t          ",
                          "The Big H:            H          ",
                          "Move Forward:         w          ",
                          "Move Backward:        s          ",
                          "Move Port:            a          ",
                          "Move Starboard:       d          ",
                          "Yaw Counterclockwise: arrow up   ",
                          "Yaw Clockwise:        arrow down "
                          ]

    def read_key(self):
        '''
        Reads the newest key from the buffer and discards all old keys that
        were not read.
        '''
        keycode = -1
        new_keycode = self.screen.getch()

        # This eliminates building a buffer of keys that takes forever to process
        while ((new_keycode != -1) and (not rospy.is_shutdown())):
            keycode = new_keycode
            new_keycode = self.screen.getch()

        # The 'q' key can be used to quit the program
        if (keycode == ord('q')):
            rospy.signal_shutdown("The user has closed the keyboard client")
        elif (keycode == ord('H')):
            raise NotImplementedError("Kevin, you just threw away your shot!")

        return keycode if keycode != -1 else None

    def send_key(self, event):
        '''
        Sends the key to the keyboard server and stores the returned locked
        status and generated UUID (if one was received).
        '''
        keycode = self.read_key()
        service_reply = self.keyboard_server(self.uuid, keycode)

        # Flashes the interface if the locked state has changed
        if (self.is_locked != service_reply.is_locked):
            self.flash()
            self.refresh_status_text()

        self.is_locked = service_reply.is_locked
        if (service_reply.generated_uuid != ''):
            self.uuid = service_reply.generated_uuid
            self.refresh_status_text()

        self.refresh_status_text()

    def refresh_status_text(self):
        '''
        Updates the status bar text, which consists of the UUID and the current
        locked state, and a help menu of which key is used for what.
        '''
        uuid_string = "UUID: {}".format(self.uuid)
        locked_string = "Locked: {}".format(self.is_locked)
        height, width = self.screen.getmaxyx()
        self.clear()

        # Prints the status bar text
        if ((height >= 3) and (width >= 3 + len(uuid_string) + len(locked_string))):
            y, x = 1, int((width - len(uuid_string) - len(locked_string)) / 3)
            self.screen.addstr(y, x, uuid_string)
            self.screen.addstr(y, 2 * x + len(uuid_string), locked_string)

        index = 0
        help_menu_rows = height - 3
        help_menu_columns = int((width - 1) / len(self.help_menu[0]))

        # Prints the help menu based on how many rows and columns are available in the space of the window
        for row in range(help_menu_rows):
            for column in range(help_menu_columns):
                if (index < len(self.help_menu)):
                    y, x = row + 3, 1 + column * len(self.help_menu[0])
                    self.screen.addstr(y, x, self.help_menu[index])
                    index += 1

    def flash(self):
        '''
        Flashes the color of the screen to white for a short time.
        '''
        curses.flash()

    def clear(self):
        '''
        Clears all text on the screen.
        '''
        self.screen.clear()


def main(stdscr):
    rospy.wait_for_service("/keyboard_control")
    tele = KeyboardClient(stdscr)
    rospy.Timer(rospy.Duration(0.05), tele.send_key, oneshot=False)
    rospy.spin()


if __name__ == "__main__":
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass
