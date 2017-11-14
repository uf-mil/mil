#!/usr/bin/env python
'''
Dashboard: A graphical interface to view the status of various systems on
NaviGator and a control panel to interact with the running system.
'''
import os
import threading
from mil_tools import thread_lock
from ros_alarms import AlarmListener
from navigator_msgs.msg import Hosts, Host
from python_qt_binding import QtCore, QtWidgets, loadUi
from qt_gui.plugin import Plugin
from remote_control_lib import RemoteControl
import rospkg
import rospy
from copy import copy
from std_msgs.msg import Float32, String


__author__ = "Anthony Olive"
__maintainer__ = "Anthony Olive"
__email__ = "anthony@iris-systems.net"
__copyright__ = "Copyright 2016, MIL"
__license__ = "MIT"

lock = threading.Lock()


class Dashboard(Plugin):

    UPDATE_MILLISECONDS = 100

    def __init__(self, context):
        super(Dashboard, self).__init__(context)

        # Create the widget and name it
        self._widget = QtWidgets.QWidget()
        self._widget.setObjectName("Dashboard")
        self.setObjectName("Dashboard")

        # Extend the widget with all attributes and children in the UI file
        ui_file = os.path.join(rospkg.RosPack().get_path("navigator_gui"), "resource", "dashboard.ui")
        loadUi(ui_file, self._widget)

        self.remote = RemoteControl("dashboard")

        # Creates dictionaries that are used by the monitor functions to keep track of their node or service
        node_monitor_template = {
            "current": "Unknown",
            "displayed": "Unknown",
        }
        self.operating_mode = copy(node_monitor_template)
        self.battery_voltage = copy(node_monitor_template)
        self.kill_status = copy(node_monitor_template)
        self.kill_status["current"] = True
        self.system_time = copy(node_monitor_template)
        self.hosts = node_monitor_template.copy()
        self.clear_hosts()

        self.connect_ui()
        self.connect_ros()

        # Deals with problem when they're multiple instances of Dashboard plugin
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (" (%d)" % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

        # Creates monitors that update data on the GUI periodically
        self.update_gui()

    @thread_lock(lock)
    def update_gui(self):
        self.system_time["current"] = rospy.Time.now()
        if (self.system_time["current"] != self.system_time["displayed"]):
            self.update_system_time_status()
        if (self.kill_status["current"] != self.kill_status["displayed"]):
            self.update_kill_status()
        if (self.operating_mode["current"] != self.operating_mode["displayed"]):
            self.update_operating_mode_status()
        if (self.battery_voltage["current"] != self.battery_voltage["displayed"]):
            self.update_battery_voltage_status()
        if (self.hosts["current"] != self.hosts["displayed"]):
            self.update_hosts_status()

        # Schedules the next instance of this method with a QT timer
        QtCore.QTimer.singleShot(self.UPDATE_MILLISECONDS, self.update_gui)

    def clear_hosts(self):
        '''
        Builds a list of host dictionaries that contain the devices hostname,
        an unknown IP address, and an unknown status in the hosts' receiving
        variable.
        '''
        self.hosts["current"] = Hosts()
        for hostname in self.hosts["current"].hostnames.split():
            host = Host()
            host.hostname = hostname
            host.ip = "Unknown"
            host.status = "Unknown"
            self.hosts["current"].hosts.append(host)

    def connect_ui(self):
        '''
        Links objects in the dashboard GUI to variables in the backend
        dashboard object.
        '''

        # Kill status
        self.kill_status_frame = self._widget.findChild(QtWidgets.QFrame, "kill_status_frame")
        self.kill_status_status = self._widget.findChild(QtWidgets.QLabel, "kill_status_status")

        # Operating mode status
        self.operating_mode_frame = self._widget.findChild(QtWidgets.QFrame, "operating_mode_frame")
        self.operating_mode_status = self._widget.findChild(QtWidgets.QLabel, "operating_mode_status")

        # Battery voltage
        self.battery_voltage_frame = self._widget.findChild(QtWidgets.QFrame, "battery_voltage_frame")
        self.battery_voltage_status = self._widget.findChild(QtWidgets.QLabel, "battery_voltage_status")

        # System time
        self.system_time_frame = self._widget.findChild(QtWidgets.QFrame, "system_time_frame")
        self.system_time_status = self._widget.findChild(QtWidgets.QLabel, "system_time_status")

        # Devices table
        self.device_table = self._widget.findChild(QtWidgets.QFrame, "device_table")

        # Control panel buttons
        toggle_kill_button = self._widget.findChild(QtWidgets.QPushButton, "toggle_kill_button")
        toggle_kill_button.clicked.connect(self.remote.toggle_kill)
        station_hold_button = self._widget.findChild(QtWidgets.QPushButton, "station_hold_button")
        station_hold_button.clicked.connect(self.remote.station_hold)
        rc_control_button = self._widget.findChild(QtWidgets.QPushButton, "rc_control_button")
        rc_control_button.clicked.connect(self.remote.select_rc_control)
        emergency_control_button = self._widget.findChild(QtWidgets.QPushButton, "emergency_control_button")
        emergency_control_button.clicked.connect(self.remote.select_emergency_control)
        keyboard_control_button = self._widget.findChild(QtWidgets.QPushButton, "keyboard_control_button")
        keyboard_control_button.clicked.connect(self.remote.select_keyboard_control)
        autonomous_control_button = self._widget.findChild(QtWidgets.QPushButton, "autonomous_control_button")
        autonomous_control_button.clicked.connect(self.remote.select_autonomous_control)

        # Defines the color scheme as QT style sheets
        self.colors = {
            "red": "QWidget {background-color:#FF432E;}",
            "green": "QWidget {background-color:#B1EB00;}",
            "blue": "QWidget {background-color:#4AA8DB;}",
            "yellow": "QWidget {background-color:#FDEF14;}",
            "orange": "QWidget {background-color:#FFA500;}"
        }

    def connect_ros(self):
        '''
        Connect ROS nodes, services, and alarms to variables and methods
        within this class.
        '''
        # Attempts to read the battery voltage parameters (sets them to defaults if they have not been set)
        self.battery_low_voltage = rospy.get_param("/battery_monitor/battery_low_voltage", 24)
        self.battery_critical_voltage = rospy.get_param("/battery_monitor/battery_critical_voltage", 20)

        rospy.Subscriber("/wrench/selected", String, self.cache_operating_mode)
        rospy.Subscriber("/battery_monitor", Float32, self.cache_battery_voltage)
        rospy.Subscriber("/host_monitor", Hosts, self.cache_hosts)

        self.kill_listener = AlarmListener("kill", callback_funct=self.cache_kill_status)

    @thread_lock(lock)
    def cache_kill_status(self, alarm):
        self.kill_status["current"] = alarm.raised

    @thread_lock(lock)
    def cache_operating_mode(self, msg):
        self.operating_mode["current"] = msg.data

    @thread_lock(lock)
    def cache_battery_voltage(self, msg):
        self.battery_voltage["current"] = msg.data

    @thread_lock(lock)
    def cache_hosts(self, msg):
        '''
        Converts a published hosts string into a hosts dictionary and stores it
        in the hosts receiving variable.
        '''
        self.hosts["current"] = msg
        self.hosts["stamp"] = rospy.Time.now()

    def update_kill_status(self):
        if self.kill_status["current"] is False:
            self.kill_status_status.setText("Alive")
            self.kill_status_frame.setStyleSheet(self.colors["green"])
        elif self.kill_status["current"] is True:
            self.kill_status_status.setText("Killed")
            self.kill_status_frame.setStyleSheet(self.colors["red"])
        else:
            self.kill_status_frame.setStyleSheet(self.colors["red"])
            self.kill_status_status.setText("Unknown")
        self.kill_status["displayed"] = self.kill_status["current"]

    def update_operating_mode_status(self):
        '''
        Updates the displayed operating mode status text and color.
        '''
        if (self.operating_mode["current"] == "Unknown"):
            self.operating_mode_status.setText("Unknown")
            self.operating_mode_frame.setStyleSheet(self.colors["red"])

        elif ('rc' in self.operating_mode["current"]):
            self.operating_mode_status.setText("Joystick")
            self.operating_mode_frame.setStyleSheet(self.colors["blue"])

        elif ('emergency' in self.operating_mode["current"]):
            self.operating_mode_status.setText("Emergency")
            self.operating_mode_frame.setStyleSheet(self.colors["orange"])

        elif ('keyboard' in self.operating_mode["current"]):
            self.operating_mode_status.setText("Keyboard")
            self.operating_mode_frame.setStyleSheet(self.colors["yellow"])

        elif ('autonomous' in self.operating_mode["current"]):
            self.operating_mode_status.setText("Autonomous")
            self.operating_mode_frame.setStyleSheet(self.colors["green"])

        # Set the cached operating mode to the value that was just displayed
        self.operating_mode["displayed"] = self.operating_mode["current"]

    def monitor_battery_voltage(self):
        '''
        Monitors the battery voltage on a 1s interval. Only updates the display
        when the current battery voltage has changed. Will time out and
        display an unknown status if it has been 15s since the last message.
        '''

        # Sets the battery voltage to 'Unknown' if no message has been current in 15s
        if (((rospy.Time.now() - self.battery_voltage["stamp"]) > rospy.Duration(15)) or
                (self.battery_voltage["current"] is None)):
            self.battery_voltage["current"] = "Unknown"

        # Updates the displayed data if a new battery voltage has been current since the last timer
        if (self.battery_voltage["current"] != self.battery_voltage["displayed"]):
            self.update_battery_voltage_status()

        # Schedules the next instance of this method with a QT timer
        QtCore.QTimer.singleShot(1000, self.monitor_battery_voltage)

    def update_battery_voltage_status(self):
        '''
        Updates the displayed battery voltage status text and color. Uses a
        cached warning color to make sure the color is not changed on every
        update.
        '''
        if (self.battery_voltage["current"] == "Unknown"):
            self.battery_voltage_status.setText("Unknown")
            self.battery_voltage_frame.setStyleSheet(self.colors["red"])
        else:
            if (self.battery_voltage["current"] <= self.battery_critical_voltage):
                self.battery_voltage_frame.setStyleSheet(self.colors["red"])
            elif (self.battery_voltage["current"] <= self.battery_low_voltage):
                self.battery_voltage_frame.setStyleSheet(self.colors["yellow"])
            else:
                self.battery_voltage["cached_warning_color"] = "green"
            self.battery_voltage_status.setText(str(self.battery_voltage["current"])[:5])

        # Set the cached battery voltage to the value that was just displayed
        self.battery_voltage["displayed"] = self.battery_voltage["current"]

    def update_system_time_status(self):
        time_string = str(self.system_time["current"])
        self.system_time_status.setText(time_string[:-9] + "." + time_string[-9:-8] + "s")
        self.system_time["displayed"] = self.system_time["current"]

    def update_hosts_status(self):
        '''
        Updates the text and color of the displayed hosts table elements.
        '''
        rows = self.hosts["current"].hosts
        columns = ["ip", "status"]

        for row in range(len(rows)):
            column_color = []

            # Assigns colors to each item in the row
            if (getattr(rows[row], "ip") == "Unknown"):
                column_color.append(self.colors["red"])
                column_color.append(self.colors["red"])
            else:
                column_color.append(self.colors["green"])
                if (getattr(rows[row], "status") == "Online"):
                    column_color.append(self.colors["green"])
                else:
                    column_color.append(self.colors["red"])

            # Updates the host's IP address and status in the devices table
            for column in range(len(columns)):
                entry = QtWidgets.QLabel(getattr(rows[row], columns[column]))
                entry.setAlignment(QtCore.Qt.AlignCenter)
                entry.setStyleSheet(column_color[column])
                self.device_table.setCellWidget(row, column, entry)

        # Set the cached host data to the host data that was just displayed
        self.hosts["displayed"] = copy(self.hosts["current"])
