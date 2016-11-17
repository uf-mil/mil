#!/usr/bin/env python

'''
Shooter: A graphical interface to view the status of and control the shooter
mechanism.
'''


from __future__ import division

import os

from navigator_alarm import AlarmListener
from python_qt_binding import QtGui
from python_qt_binding import loadUi
from qt_gui.plugin import Plugin
from remote_control_lib import RemoteControl
import rospkg
import rospy
from std_msgs.msg import String


__author__ = "Anthony Olive"
__maintainer__ = "Anthony Olive"
__email__ = "anthony@iris-systems.net"
__copyright__ = "Copyright 2016, MIL"
__license__ = "MIT"


class Shooter(Plugin):

    def __init__(self, context):
        super(Shooter, self).__init__(context)

        # Create the widget and name it
        self._widget = QtGui.QWidget()
        self._widget.setObjectName("Shooter")
        self.setObjectName("Shooter")

        # Extend the widget with all attributes and children in the UI file
        ui_file = os.path.join(rospkg.RosPack().get_path("navigator_gui"), "resource", "shooter.ui")
        loadUi(ui_file, self._widget)

        self.is_killed = False
        self.remote = RemoteControl("shooter gui")
        self.remote.is_timed_out = True

        self.shooter_status = {
            "received": "Unknown",
            "cached": "Unknown"
        }
        self.disc_speed_setting = 0

        self.connect_ui()

        self.kill_listener = AlarmListener("kill", self.update_kill_status)
        rospy.Subscriber("/shooter/status", String, self.update_shooter_status)

        # Show _widget.windowTitle on left-top of each plugin (when it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (" (%d)" % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

    def connect_ui(self):
        '''
        Links objects in the shooter GUI to variables in the backend
        shooter object.
        '''

        # Shooter status indicator
        self.shooter_status_frame = self._widget.findChild(QtGui.QFrame, "shooter_status_frame")
        self.shooter_status_message = self._widget.findChild(QtGui.QLabel, "shooter_status_message")

        # Control panel buttons
        load_button = self._widget.findChild(QtGui.QPushButton, "load_button")
        load_button.clicked.connect(self.remote.shooter_load)
        fire_button = self._widget.findChild(QtGui.QPushButton, "fire_button")
        fire_button.clicked.connect(self.remote.shooter_fire)
        cancel_button = self._widget.findChild(QtGui.QPushButton, "cancel_button")
        cancel_button.clicked.connect(self.remote.shooter_cancel)
        reset_button = self._widget.findChild(QtGui.QPushButton, "reset_button")
        reset_button.clicked.connect(self.remote.shooter_reset)
        linear_extend_button = self._widget.findChild(QtGui.QPushButton, "linear_extend_button")
        linear_extend_button.clicked.connect(self.remote.shooter_linear_extend)
        linear_retract_button = self._widget.findChild(QtGui.QPushButton, "linear_retract_button")
        linear_retract_button.clicked.connect(self.remote.shooter_linear_retract)
        disc_speed_slider = self._widget.findChild(QtGui.QSlider, "disc_speed_slider")
        disc_speed_slider.valueChanged[int].connect(self.cache_disc_speed_setting)
        set_disc_speed_button = self._widget.findChild(QtGui.QPushButton, "set_disc_speed_button")
        set_disc_speed_button.clicked.connect(self.set_disc_speed)

        # Defines the color scheme as QT style sheets
        self.colors = {
            "red": "QWidget {background-color:#FF432E;}",
            "green": "QWidget {background-color:#B1EB00;}",
            "yellow": "QWidget {background-color:#FDEF14;}"
        }
        self.status_colors = {"green": ["Standby", "Loaded", "Fired"],
                              "yellow": ["Loading", "Firing", "Canceling"],
                              "red": ["Unknown", "Error", "Canceled"]
                              }

    def cache_disc_speed_setting(self, speed):
        '''
        Caches the shooter acceleration disc speed whenever it is updated on
        the GUI.
        '''
        self.disc_speed_setting = speed

    def set_disc_speed(self):
        '''
        Sets the shooter's accelerator disc speed to the cached value.
        '''
        self.remote.set_disc_speed(self.disc_speed_setting)

    def update_kill_status(self, alarm):
        '''
        Updates the kill status display when there is an update on the kill
        alarm.
        '''
        self.is_killed = not alarm.clear

    def update_shooter_status(self, msg):
        '''
        Updates the shooter status display when a message is published to the
        shooter_status topic. Caches the last displayed status to avoid
        updating the display with the same information twice.
        '''
        self.shooter_status["received"] = msg.data

        if (self.shooter_status["received"] != self.shooter_status["cached"]):
            self.shooter_status_message.setText(self.shooter_status["received"])

            # Update the status display color based on the message received
            for color, messages in self.status_colors.iteritems():
                if (self.shooter_status["received"] in messages):
                    self.shooter_status_frame.setStyleSheet(self.colors[color])

                    # Mark the remote control as timed out if an action is in progress or the status is unknown or error
                    self.remote.is_timed_out = True if ((color == "yellow") or
                                                        (self.shooter_status["received"] in ["Unknown", "Error"])) else False

            # Set the cached shooter status to the value that was just displayed
            self.shooter_status["cached"] = self.shooter_status["received"]
