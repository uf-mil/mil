#!/usr/bin/env python

'''
Shooter: A graphical interface to view the status of and control the shooter
mechanism.
'''


from __future__ import division

import os

from python_qt_binding import QtCore
from python_qt_binding import QtWidgets
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
        self._widget = QtWidgets.QWidget()
        self._widget.setObjectName("Shooter")
        self.setObjectName("Shooter")

        # Extend the widget with all attributes and children in the UI file
        ui_file = os.path.join(rospkg.RosPack().get_path("navigator_gui"), "resource", "shooter.ui")
        loadUi(ui_file, self._widget)

        self.remote = RemoteControl("shooter gui")
        self.remote.is_timed_out = True

        self.shooter_status = {
            "received": "Unknown",
            "stamp": rospy.Time.now(),
            "cached": "Unknown"
        }

        self.disc_speed_setting = 0

        self.connect_ui()

        rospy.Subscriber("/shooter/status", String, self.cache_shooter_status)

        # Deals with problem of multiple instances of same plugin
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
        self.shooter_status_frame = self._widget.findChild(QtWidgets.QFrame, "shooter_status_frame")
        self.shooter_status_message = self._widget.findChild(QtWidgets.QLabel, "shooter_status_message")

        # Control panel buttons
        load_button = self._widget.findChild(QtWidgets.QPushButton, "load_button")
        load_button.clicked.connect(self.remote.shooter_load)
        fire_button = self._widget.findChild(QtWidgets.QPushButton, "fire_button")
        fire_button.clicked.connect(self.remote.shooter_fire)
        cancel_button = self._widget.findChild(QtWidgets.QPushButton, "cancel_button")
        cancel_button.clicked.connect(self.remote.shooter_cancel)
        reset_button = self._widget.findChild(QtWidgets.QPushButton, "reset_button")
        reset_button.clicked.connect(self.remote.shooter_reset)
        linear_extend_button = self._widget.findChild(QtWidgets.QPushButton, "linear_extend_button")
        linear_extend_button.clicked.connect(self.remote.shooter_linear_extend)
        linear_retract_button = self._widget.findChild(QtWidgets.QPushButton, "linear_retract_button")
        linear_retract_button.clicked.connect(self.remote.shooter_linear_retract)
        disc_speed_slider = self._widget.findChild(QtWidgets.QSlider, "disc_speed_slider")
        disc_speed_slider.valueChanged[int].connect(self.cache_disc_speed_setting)
        set_disc_speed_button = self._widget.findChild(QtWidgets.QPushButton, "set_disc_speed_button")
        set_disc_speed_button.clicked.connect(self.set_disc_speed)

        # Defines the color scheme as QT style sheets
        self.colors = {
            "red": "QWidget {background-color:#FF432E;}",
            "green": "QWidget {background-color:#B1EB00;}",
            "yellow": "QWidget {background-color:#FDEF14;}"
        }
        self.status_colors = {"green": ["Standby", "Loaded"],
                              "yellow": ["Loading", "Firing", "Canceled", "Manual"],
                              "red": ["Unknown"]
                              }

        # Creates a monitor that update the shooter status on the GUI periodically
        self.monitor_shooter_status()

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

    def cache_shooter_status(self, msg):
        '''
        Stores the shooter status when it is published.
        '''
        self.shooter_status["received"] = msg.data
        self.shooter_status["stamp"] = rospy.Time.now()

    def monitor_shooter_status(self):
        '''
        Monitors the shooter status on a 1s interval. Only updates the display
        when the received shooter status has changed. The connection to the
        status will time out if no message has been received in 1s.
        '''
        if ((rospy.Time.now() - self.shooter_status["stamp"]) < rospy.Duration(1)):
            self.remote.is_timed_out = False

        # Sets the remote control to timed out and the shooter status to 'Unknown' if no message has been received in 1s
        else:
            self.remote.is_timed_out = True
            self.shooter_status["received"] = "Unknown"

        if (self.shooter_status["received"] != self.shooter_status["cached"]):
            self.update_shooter_status()

        # Schedules the next instance of this method with a QT timer
        QtCore.QTimer.singleShot(200, self.monitor_shooter_status)

    def update_shooter_status(self):
        '''
        Updates the displayed shooter status text and color. Sets the color of
        the status frame based on the text's relationship to it in the
        status_colors dictionary.
        '''
        self.shooter_status_message.setText(self.shooter_status["received"])

        # Update the status display color based on the message received
        for color, messages in self.status_colors.iteritems():
            if (self.shooter_status["received"] in messages):
                self.shooter_status_frame.setStyleSheet(self.colors[color])

        # Set the cached shooter status to the value that was just displayed
        self.shooter_status["cached"] = self.shooter_status["received"]
