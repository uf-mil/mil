#!/usr/bin/env python

'''
Dashboard: A graphical interface to view the status of various systems on
Navigator and to select one of the control inputs to use.
'''


import os
import socket
import subprocess

from kill_handling.broadcaster import KillBroadcaster
from navigator_alarm import AlarmBroadcaster
from navigator_alarm import AlarmListener
from navigator_msgs.srv import WrenchSelect
from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import loadUi
from qt_gui.plugin import Plugin
import rospkg
import rospy
from std_msgs.msg import Float32


__author__ = "Anthony Olive"
__maintainer__ = "Anthony Olive"
__email__ = "anthony@iris-systems.net"
__copyright__ = "Copyright 2016, MIL"
__license__ = "MIT"


class Dashboard(Plugin):

    def __init__(self, context):
        super(Dashboard, self).__init__(context)

        # Create the widget and name it
        self._widget = QtGui.QWidget()
        self._widget.setObjectName('Dashboard')
        self.setObjectName('Dashboard')

        # Extend the widget with all attributes and children in the UI file
        ui_file = os.path.join(rospkg.RosPack().get_path('navigator_gui'), 'resource', 'dashboard.ui')
        loadUi(ui_file, self._widget)
        self.connect_ui()

        # Defines the color scheme as QT style sheets
        self.colors = {
            "red": "QWidget {background-color:#FF432E;}",
            "green": "QWidget {background-color:#B1EB00;}",
            "blue": "QWidget {background-color:#4AA8DB;}",
            "yellow": "QWidget {background-color:#FDEF14;}"
        }

        # Define a dictionary of hosts that resolve to devices on navigator
        self.hosts = {
            "mil-nav-ubnt-shore": ["Unknown", "Unknown"],
            "mil-nav-ubnt-wamv": ["Unknown", "Unknown"],
            "mil-nav-wamv": ["Unknown", "Unknown"],
            "mil-com-velodyne-vlp16": ["Unknown", "Unknown"],
            "mil-com-sick-lms111": ["Unknown", "Unknown"]
        }

        # Temporary checking of the hosts on initialization
        self.resolve_hosts()
        self.check_hosts()

        # Dashboard state parameters
        self.is_killed = False
        self.cached_operating_mode = None
        self.cached_battery_alarm_level = None

        # Attempts to read the battery voltage parameters (sets them to defaults if they have not been set)
        self.battery_low_voltage = rospy.get_param("/battery_monitor/battery_low_voltage", 22.1)
        self.battery_critical_voltage = rospy.get_param("/battery_monitor/battery_critical_voltage", 20.6)
        rospy.Subscriber("/battery_monitor", Float32, self.update_battery_voltage_status)

        self.kb = KillBroadcaster(id='station_hold', description='Resets Pose')
        self.kill_listener = AlarmListener('kill', self.update_kill_status)
        # rospy.wait_for_service('/change_wrench')
        self.wrench_changer = rospy.ServiceProxy('/change_wrench', WrenchSelect)

        alarm_broadcaster = AlarmBroadcaster()
        self.kill_alarm = alarm_broadcaster.add_alarm(
            name='kill',
            action_required=True,
            severity=0
        )

        # Show _widget.windowTitle on left-top of each plugin (when it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

    def connect_ui(self):
        '''
        Connects objects in the GUI to the backend dashboard handler
        functions.
        '''

        # Kill status
        self.kill_status_frame = self._widget.findChild(QtGui.QFrame, 'kill_status_frame')
        self.kill_status_status = self._widget.findChild(QtGui.QLabel, 'kill_status_status')

        # Operating mode status
        self.operating_mode_frame = self._widget.findChild(QtGui.QFrame, 'operating_mode_frame')
        self.operating_mode_status = self._widget.findChild(QtGui.QLabel, 'operating_mode_status')

        # Battery voltage
        self.battery_voltage_frame = self._widget.findChild(QtGui.QFrame, 'battery_voltage_frame')
        self.battery_voltage_status = self._widget.findChild(QtGui.QLabel, 'battery_voltage_status')

        # Devices table
        self.device_table = self._widget.findChild(QtGui.QFrame, 'device_table')

        # Control panel buttons
        toggle_kill_button = self._widget.findChild(QtGui.QPushButton, 'toggle_kill_button')
        toggle_kill_button.clicked.connect(self.toggle_kill)
        station_hold_button = self._widget.findChild(QtGui.QPushButton, 'station_hold_button')
        station_hold_button.clicked.connect(self.station_hold)
        autonomous_control_button = self._widget.findChild(QtGui.QPushButton, 'autonomous_control_button')
        autonomous_control_button.clicked.connect(self.select_autonomous_control)
        rc_control_button = self._widget.findChild(QtGui.QPushButton, 'rc_control_button')
        rc_control_button.clicked.connect(self.select_rc_control)

    def resolve_hosts(self):
        '''
        Resolves the hosts of the devices on NaviGator to IP addresses.
        '''
        row = 0

        for host in self.hosts.keys():

            # Resolves the IP address of the hostname
            try:
                ip = socket.gethostbyname(host)
                ip_color = self.colors["green"]

            # If hostname resolution fails, the IP address is set the unknown
            except:
                ip = "Unknown"
                ip_color = self.colors["red"]

            self.hosts[host][0] = ip

            # Updates the host's IP address in the devices table
            ip_item = QtGui.QLabel(ip)
            ip_item.setAlignment(QtCore.Qt.AlignCenter)
            ip_item.setStyleSheet(ip_color)
            self.device_table.setCellWidget(row, 0, ip_item)
            row += 1

    def check_hosts(self):
        '''
        Pings the resolved hosts to check whether or not they are online.
        '''
        row = 0

        for host in self.hosts.keys():
            if (self.hosts[host][0] != "Unknown"):

                # If the host is pingable, mark it as online
                try:
                    subprocess.check_output(['ping', '-c1', host])
                    status = "Online"
                    status_color = self.colors["green"]

                # If pinging the host is unsuccessful, mark it as offline
                except:
                    status = "Offline"
                    status_color = self.colors["red"]

            # If hostname resolution failed, the status is set the unknown
            else:
                status = "Unknown"
                status_color = self.colors["red"]

            # Updates the status of the host in the devices table
            status_item = QtGui.QLabel(status)
            status_item.setAlignment(QtCore.Qt.AlignCenter)
            status_item.setStyleSheet(status_color)
            self.device_table.setCellWidget(row, 1, status_item)
            row += 1

    def update_kill_status(self, alarm):
        '''
        Updates the kill status display when there is an update on the kill
        alarm.
        '''
        if (not alarm.clear):
            self.is_killed = True
            self.kill_status_status.setText("Killed")
            self.kill_status_frame.setStyleSheet(self.colors["red"])
        else:
            self.is_killed = False
            self.kill_status_status.setText("Alive")
            self.kill_status_frame.setStyleSheet(self.colors["green"])

    def update_operating_mode_status(self):
        '''
        Updates the operating mode display based on the value returned by the
        wrench arbiter. Caches the current battery warning level to reduce
        system resource consumption.
        '''

        # Attempting to set the wrench to an empty string will return the current selected input
        current_operating_mode = self.wrench_changer("").str

        # Only changes the display if the selected input has changed
        if (current_operating_mode != self.cached_operating_mode):
            if (current_operating_mode == "rc"):
                self.operating_mode_status.setText("Remote")
                self.operating_mode_frame.setStyleSheet(self.colors["blue"])
                self.cached_operating_mode = "rc"
            elif (current_operating_mode == "autonomous"):
                self.operating_mode_status.setText("Autonomous")
                self.operating_mode_frame.setStyleSheet(self.colors["green"])
                self.cached_operating_mode = "autonomous"

    def update_battery_voltage_status(self, msg):
        '''
        Updates the battery voltage display when there is an update on the
        /battery_monitor node. Caches the current battery warning level to
        reduce system resource consumption.
        '''
        if (msg.data > 0):
            self.battery_voltage_status.setText(str(msg.data)[:5])

            # Set the frame background color to red if the battery is at or below the critical voltage
            if (msg.data <= self.battery_critical_voltage):
                if (self.cached_battery_alarm_level != "red"):
                    self.battery_voltage_frame.setStyleSheet(self.colors["red"])
                    self.cached_battery_alarm_level = "red"

            # Set the frame background color to yellow if the battery is at or below the low voltage
            elif (msg.data <= self.battery_low_voltage):
                if (self.cached_battery_alarm_level != "yellow"):
                    self.battery_voltage_frame.setStyleSheet(self.colors["yellow"])
                    self.cached_battery_alarm_level = "yellow"

            # Set the frame background color to green if the battery is above the warning voltages
            elif (self.cached_battery_alarm_level != "green"):
                self.battery_voltage_frame.setStyleSheet(self.colors["green"])
                self.cached_battery_alarm_level = "green"

            # Takes advantage of the periodic nature of the /battery_monitor node
            self.update_operating_mode_status()

    def toggle_kill(self):
        '''
        Toggles the kill status when the toggle_kill_button is pressed.
        '''
        rospy.loginfo("Toggling Kill")

        # Responds to the kill broadcaster and checks the status of the kill alarm
        if self.is_killed:
            self.kill_alarm.clear_alarm()
        else:
            self.wrench_changer("rc")
            self.kill_alarm.raise_alarm(
                problem_description='System kill from location: dashboard'
            )

        self.is_killed = not self.is_killed

    def station_hold(self):
        '''
        Sets the goal point to the current location and switches to autonomous
        mode in order to stay at that point.
        '''
        rospy.loginfo("Station Holding")

        # Resets c3, this will change when c3 is replaced
        self.kb.send(active=True)
        self.kb.send(active=False)
        self.wrench_changer("autonomous")

    def select_autonomous_control(self):
        '''
        Selects the autonomously generated trajectory as the active controller.
        '''
        rospy.loginfo("Changing Control to Autonomous")
        self.wrench_changer("autonomous")

    def select_rc_control(self):
        '''
        Selects the XBox remote joystick as the active controller.
        '''
        rospy.loginfo("Changing Control to RC")
        self.wrench_changer("rc")
