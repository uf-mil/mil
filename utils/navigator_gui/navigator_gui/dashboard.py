#!/usr/bin/env python

'''
Dashboard: A graphical interface to view the status of various systems on
Navigator and a control panel to interact with the running system.
'''


import os
import socket
import subprocess

from navigator_alarm import AlarmBroadcaster
from navigator_alarm import AlarmListener
from navigator_msgs.srv import WrenchSelect
from python_qt_binding import QtCore
from python_qt_binding import QtGui
from python_qt_binding import loadUi
from qt_gui.plugin import Plugin
from remote_control_lib import RemoteControl
from rosgraph_msgs.msg import Clock
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

        self.is_killed = False
        self.remote = RemoteControl('dashboard')

        # Creates dictionaries that are used by the monitor functions to keep track of their node or service
        service_monitor_template = {
            "received": "Unknown",
            "cached": "Unknown"
        }
        node_monitor_template = service_monitor_template.copy()
        node_monitor_template["timeout_count"] = 0
        node_monitor_template["is_timed_out"] = False
        self.operating_mode = service_monitor_template.copy()
        self.battery_voltage = node_monitor_template.copy()
        self.battery_voltage["cached_warning_color"] = "red"
        self.system_time = node_monitor_template.copy()
        self.system_time["is_timed_out"] = True

        # Build an ordered list of host dictionaries that resolve to devices on navigator
        host_list = [
            "mil-nav-ubnt-wamv",
            "mil-nav-ubnt-shore",
            "mil-nav-wamv",
            "mil-com-velodyne-vlp16",
            "mil-com-sick-lms111"
        ]
        host_template = {
            "hostname": "",
            "ip": "Unknown",
            "status": "Unknown"
        }
        self.hosts = []
        for host in host_list:
            host_entry = host_template.copy()
            host_entry["hostname"] = host
            self.hosts.append(host_entry)

        self.connect_ui()
        self.connect_ros()

        # Show _widget.windowTitle on left-top of each plugin (when it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

        # Creates monitors that update data on the GUI periodically
        self.monitor_operating_mode()
        self.monitor_battery_voltage()
        self.monitor_system_time()
        self.monitor_hosts()

    def connect_ui(self):
        '''
        Links objects in the dashboard GUI to variables in the backend
        dashboard object.
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

        # System time
        self.system_time_frame = self._widget.findChild(QtGui.QFrame, 'system_time_frame')
        self.system_time_status = self._widget.findChild(QtGui.QLabel, 'system_time_status')

        # Devices table
        self.device_table = self._widget.findChild(QtGui.QFrame, 'device_table')

        # Control panel buttons
        toggle_kill_button = self._widget.findChild(QtGui.QPushButton, 'toggle_kill_button')
        toggle_kill_button.clicked.connect(self.remote.toggle_kill)
        station_hold_button = self._widget.findChild(QtGui.QPushButton, 'station_hold_button')
        station_hold_button.clicked.connect(self.remote.station_hold)
        autonomous_control_button = self._widget.findChild(QtGui.QPushButton, 'autonomous_control_button')
        autonomous_control_button.clicked.connect(self.remote.select_autonomous_control)
        rc_control_button = self._widget.findChild(QtGui.QPushButton, 'rc_control_button')
        rc_control_button.clicked.connect(self.remote.select_rc_control)
        keyboard_control_button = self._widget.findChild(QtGui.QPushButton, 'keyboard_control_button')
        keyboard_control_button.clicked.connect(self.remote.select_keyboard_control)

        # Defines the color scheme as QT style sheets
        self.colors = {
            "red": "QWidget {background-color:#FF432E;}",
            "green": "QWidget {background-color:#B1EB00;}",
            "blue": "QWidget {background-color:#4AA8DB;}",
            "yellow": "QWidget {background-color:#FDEF14;}"
        }

    def connect_ros(self):
        '''
        Connect ROS nodes, services, and alarms to variables and methods
        within this class.
        '''
        # Attempts to read the battery voltage parameters (sets them to defaults if they have not been set)
        self.battery_low_voltage = rospy.get_param("/battery_monitor/battery_low_voltage", 22.1)
        self.battery_critical_voltage = rospy.get_param("/battery_monitor/battery_critical_voltage", 20.6)

        rospy.Subscriber("/battery_monitor", Float32, self.cache_battery_voltage)
        rospy.Subscriber("/clock", Clock, self.cache_system_time)

        self.wrench_changer = rospy.ServiceProxy('/change_wrench', WrenchSelect)
        self.kill_listener = AlarmListener('kill', self.update_kill_status)

    def timeout_check(function):
        def decorated_function(self):
            if (not self.system_time["is_timed_out"]):
                function(self)
        return decorated_function

    def update_kill_status(self, alarm):
        '''
        Updates the kill status display when there is an update on the kill
        alarm. Caches the last displayed kill status to avoid updating the
        display with the same information twice.
        '''
        if (alarm.clear):
            if (self.is_killed):
                self.is_killed = False
                self.kill_status_status.setText("Alive")
                self.kill_status_frame.setStyleSheet(self.colors["green"])

        elif (not self.is_killed):
            self.is_killed = True
            self.kill_status_status.setText("Killed")
            self.kill_status_frame.setStyleSheet(self.colors["red"])

    def monitor_operating_mode(self):
        '''
        Monitors the selected wrench input device on a 0.5s interval using the
        wrench_changer service. Only updates the display when the selected
        wrench input has changed.
        '''
        if (not self.system_time["is_timed_out"]):

            # Attempting to set the wrench to an empty string fails and returns the selected input device.
            try:
                self.operating_mode["received"] = self.wrench_changer("").str

            # If calling the service fails, the operating mode is set to unknown
            except:
                self.operating_mode["received"] = "Unknown"

            # If a new value was received, update the display
            if (self.operating_mode["received"] != self.operating_mode["cached"]):
                self.update_operating_mode_status()

        # Schedules the next instance of this method with a QT timer
        QtCore.QTimer.singleShot(500, self.monitor_operating_mode)

    def update_operating_mode_status(self):
        '''
        Updates the displayed operating mode status text and color.
        '''
        if (self.operating_mode["received"] == "autonomous"):
            self.operating_mode_status.setText("Autonomous")
            self.operating_mode_frame.setStyleSheet(self.colors["green"])

        elif (self.operating_mode["received"] == "rc"):
            self.operating_mode_status.setText("Joystick")
            self.operating_mode_frame.setStyleSheet(self.colors["blue"])

        elif (self.operating_mode["received"] == "keyboard"):
            self.operating_mode_status.setText("Keyboard")
            self.operating_mode_frame.setStyleSheet(self.colors["yellow"])

        elif (self.operating_mode["received"] == "Unknown"):
            self.operating_mode_status.setText("Unknown")
            self.operating_mode_frame.setStyleSheet(self.colors["red"])

        # Set the cached operating mode to the value that was just displayed
        self.operating_mode["cached"] = self.operating_mode["received"]

    def cache_battery_voltage(self, msg):
        '''
        Stores the battery voltage when it is published.
        '''
        self.battery_voltage["received"] = msg.data

    def monitor_battery_voltage(self):
        '''
        Monitors the battery voltage on a 1s interval. Only updates the display
        when the received battery voltage has changed.
        '''
        if (not self.system_time["is_timed_out"]):
            if (not self.battery_voltage["is_timed_out"]):

                # Counts bad values of the received battery voltage towards the timeout count
                if ((self.battery_voltage["received"] == None) or (self.battery_voltage["received"] <= 0)):
                    self.battery_voltage["timeout_count"] += 1

                # Updates the displayed data if a new battery voltage has been received since the last timer
                elif (self.battery_voltage["received"] != self.battery_voltage["cached"]):
                    self.battery_voltage["timeout_count"] = 0
                    self.update_battery_voltage_status()

                # Assumes that the battery voltage node is dead if the timeout counter reaches 15 (~15s)
                elif (self.battery_voltage["timeout_count"] >= 15):
                    self.battery_voltage["is_timed_out"] = True
                    self.battery_voltage_status.setText("Unknown")
                    self.battery_voltage_frame.setStyleSheet(self.colors["red"])
                    self.battery_voltage["cached_warning_color"] = "red"

                # Otherwise, increments the system timeout counter
                else:
                    self.battery_voltage["timeout_count"] += 1

            else:

                # If a new battery voltage has been received after a timeout, exit the timeout state
                if (self.battery_voltage["received"] != self.battery_voltage["cached"]):
                    self.battery_voltage["is_timed_out"] = False
                    self.battery_voltage["timeout_count"] = 0
                    self.update_battery_voltage_status()

        # Schedules the next instance of this method with a QT timer
        QtCore.QTimer.singleShot(1000, self.monitor_battery_voltage)

    def update_battery_voltage_status(self):
        '''
        Updates the displayed battery voltage status text and color. Uses a
        cached warning color to make sure the color is not changed on every
        update.
        '''

        # Set the frame background color to red if the battery is at or below the critical voltage
        if (self.battery_voltage["received"] <= self.battery_critical_voltage):
            if (self.battery_voltage["cached_warning_color"] != "red"):
                self.battery_voltage_frame.setStyleSheet(self.colors["red"])
                self.battery_voltage["cached_warning_color"] = "red"

        # Set the frame background color to yellow if the battery is at or below the low voltage
        elif (self.battery_voltage["received"] <= self.battery_low_voltage):
            if (self.battery_voltage["cached_warning_color"] != "yellow"):
                self.battery_voltage_frame.setStyleSheet(self.colors["yellow"])
                self.battery_voltage["cached_warning_color"] = "yellow"

        # Set the frame background color to green if the battery is above the warning voltages
        elif (self.battery_voltage["cached_warning_color"] != "green"):
            self.battery_voltage_frame.setStyleSheet(self.colors["green"])
            self.battery_voltage["cached_warning_color"] = "green"

        self.battery_voltage_status.setText(str(self.battery_voltage["received"])[:5])

        # Set the cached battery voltage to the value that was just displayed
        self.battery_voltage["cached"] = self.battery_voltage["received"]

    def cache_system_time(self, msg):
        '''
        Stores the system time when it is published.
        '''
        self.system_time["received"] = msg.clock

    def monitor_system_time(self):
        '''
        Updates data related to the system time on a 0.1s QT timer
        '''
        if (not self.system_time["is_timed_out"]):

            # Counts bad values of the received system time towards the timeout count
            if ((self.system_time["received"] == None) or (int(str(self.system_time["received"])) <= 0) or
                    len(str(self.system_time["received"])) < 9):
                self.system_time["timeout_count"] += 1

            # Updates the displayed data if a new system time has been received since the last timer
            elif (self.system_time["received"] != self.system_time["cached"]):
                self.system_time["timeout_count"] = 0
                self.update_system_time_status()

            # Assumes that we have been disconnected if the system timeout counter reaches 50 (~5s)
            elif (self.system_time["timeout_count"] >= 50):
                self.system_time["is_timed_out"] = True
                self.kill_status_status.setText("Unknown")
                self.kill_status_frame.setStyleSheet(self.colors["red"])
                self.operating_mode["received"] = "Unknown"
                self.update_operating_mode_status()
                self.battery_voltage["is_timed_out"] = True
                self.battery_voltage_status.setText("Unknown")
                self.battery_voltage_frame.setStyleSheet(self.colors["red"])
                self.battery_voltage["cached_warning_color"] = "red"
                self.system_time_status.setText("Unknown")
                self.system_time_frame.setStyleSheet(self.colors["red"])

            # Otherwise, increments the system timeout counter
            else:
                self.system_time["timeout_count"] += 1

        else:

            # If a new system time has been received after a timeout, exit the timeout state
            if (self.system_time["received"] != self.system_time["cached"]):
                self.kill_status_status.setText("Active")
                self.kill_status_frame.setStyleSheet(self.colors["green"])
                self.system_time["is_timed_out"] = False
                self.system_time["timeout_count"] = 0
                self.update_system_time_status()
                self.system_time_frame.setStyleSheet(self.colors["green"])

        self.remote.is_timed_out = self.system_time["is_timed_out"]

        # Schedules the next instance of this method with a QT timer
        QtCore.QTimer.singleShot(100, self.monitor_system_time)

    def update_system_time_status(self):
        '''
        Updates the displayed system time status text and color.
        '''
        time_string = str(self.system_time["received"])
        self.system_time_status.setText(time_string[:-9] + "." + time_string[-9:-8] + "s")

        # Set the cached system time to the value that was just displayed
        self.system_time["cached"] = self.system_time["received"]

    def monitor_hosts(self):
        '''
        Resolves the hostnames of the devices on NaviGator to IP addresses on a
        10s QT timer. If resolution is successful, pings the IP addresses to
        check whether or not they are online.
        '''
        for host in self.hosts:

            # Resolves the IP address of the hostname
            try:
                host["ip"] = socket.gethostbyname(host["hostname"])

                # If the host is pingable, mark it as online
                try:
                    subprocess.check_output(['ping', '-c1', host["ip"]])
                    host["status"] = "Online"

                # If pinging the host is unsuccessful, mark it as offline
                except:
                    host["status"] = "Offline"

            # If hostname resolution fails, the IP address is set the unknown
            except:
                host["ip"] = "Unknown"
                host["status"] = "Unknown"

        self.update_hosts_status()

        # Schedules the next instance of this method with a QT timer
        QtCore.QTimer.singleShot(10000, self.monitor_hosts)

    def update_hosts_status(self):
        '''
        Updates the text and color of the displayed hosts table elements.
        '''
        rows = self.hosts
        columns = ["ip", "status"]

        for row in range(len(rows)):
            column_color = []

            # Assigns colors to each item in the row
            if (rows[row]["ip"] == "Unknown"):
                column_color.append(self.colors["red"])
                column_color.append(self.colors["red"])
            else:
                column_color.append(self.colors["green"])
                if (rows[row]["status"] == "Online"):
                    column_color.append(self.colors["green"])
                else:
                    column_color.append(self.colors["red"])

            # Updates the host's IP address and status in the devices table
            for column in range(len(columns)):
                entry = QtGui.QLabel(rows[row][columns[column]])
                entry.setAlignment(QtCore.Qt.AlignCenter)
                entry.setStyleSheet(column_color[column])
                self.device_table.setCellWidget(row, column, entry)
