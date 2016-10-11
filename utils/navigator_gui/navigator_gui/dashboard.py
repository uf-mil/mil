#!/usr/bin/env python

'''
Dashboard: A graphical interface to view the status of various systems on
Navigator and to select one of the control inputs to use.
'''


import os

from kill_handling.broadcaster import KillBroadcaster
from navigator_alarm import AlarmBroadcaster
from navigator_alarm import AlarmListener
from navigator_msgs.srv import WrenchSelect
from python_qt_binding import QtGui
from python_qt_binding import loadUi
from qt_gui.plugin import Plugin
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
        self.connect_ui()

        # Dashboard state parameters
        self.is_killed = False
        self.cached_operating_mode = None
        self.is_battery_voltage_available = False
        self.battery_voltage_frame_color = None

        # Subscribes to topics that are monitored on the GUI
        self.kill_listener = AlarmListener('kill', self.update_kill_status)
        rospy.Subscriber("/clock", Clock, self.update_system_time_status)
        self.system_time_frame.setStyleSheet("QWidget {background-color:#B1EB00;}")

        # Attempts to read the battery voltage parameters (requires the motor controller to have been launched)
        try:
            self.battery_low_voltage = rospy.get_param('/navigator_battery_monitor/battery_low_voltage')
            self.battery_critical_voltage = rospy.get_param('/navigator_battery_monitor/battery_critical_voltage')
            rospy.Subscriber("/battery_monitor", Float32, self.update_battery_voltage_status)
            self.is_battery_voltage_available = True
        except:
            print "Failed to connect to the thrusters - has the motor controller been launched?"

        # Connect to services that can be controlled from the GUI
        self.kb = KillBroadcaster(id='station_hold', description='Resets Pose')
        self.wrench_changer = rospy.ServiceProxy('/change_wrench', WrenchSelect)

        # Sets up the dashboard controlled alarms
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

        # System time
        self.system_time_frame = self._widget.findChild(QtGui.QFrame, 'system_time_frame')
        self.system_time_status = self._widget.findChild(QtGui.QLabel, 'system_time_status')

        # Control panel buttons
        toggle_kill_button = self._widget.findChild(QtGui.QPushButton, 'toggle_kill_button')
        toggle_kill_button.clicked.connect(self.toggle_kill)
        station_hold_button = self._widget.findChild(QtGui.QPushButton, 'station_hold_button')
        station_hold_button.clicked.connect(self.station_hold)
        autonomous_control_button = self._widget.findChild(QtGui.QPushButton, 'autonomous_control_button')
        autonomous_control_button.clicked.connect(self.select_autonomous_control)
        rc_control_button = self._widget.findChild(QtGui.QPushButton, 'rc_control_button')
        rc_control_button.clicked.connect(self.select_rc_control)

    def update_kill_status(self, alarm):
        '''
        Updates the kill status display when there is an update on the kill
        alarm.
        '''
        if (not alarm.clear):
            self.is_killed = True
            self.kill_status_status.setText("Killed")
            self.kill_status_frame.setStyleSheet("QWidget {background-color:#FF432E;}")
        else:
            self.is_killed = False
            self.kill_status_status.setText("Alive")
            self.kill_status_frame.setStyleSheet("QWidget {background-color:#B1EB00;}")

    def update_operating_mode_status(self):
        '''
        Updates the operating mode display based on the value returned by the
        wrench arbiter. Caches the current battery warning level to reduce
        system resource consumption.
        '''
        current_operating_mode = self.wrench_changer("").str
        if (current_operating_mode != self.cached_operating_mode):

            if (current_operating_mode == "rc"):
                self.operating_mode_status.setText("Remote")
                self.operating_mode_frame.setStyleSheet("QWidget {background-color:#4AA8DB;}")
                self.cached_operating_mode = "rc"

            elif (current_operating_mode == "autonomous"):
                self.operating_mode_status.setText("Autonomous")
                self.operating_mode_frame.setStyleSheet("QWidget {background-color:#B1EB00;}")
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
            if (msg.data <= self.battery_critical_voltage + 3):
                if (self.battery_voltage_frame_color != "red"):
                    self.battery_voltage_frame.setStyleSheet("QWidget {background-color:#FF432E;}")
                    self.battery_voltage_frame_color = "red"

            # Set the frame background color to yellow if the battery is at or below the low voltage
            elif (msg.data <= self.battery_low_voltage + 3):
                if (self.battery_voltage_frame_color != "yellow"):
                    self.battery_voltage_frame.setStyleSheet("QWidget {background-color:#FDEF14;}")
                    self.battery_voltage_frame_color = "yellow"

            # Set the frame background color to green if the battery is above the warning voltages
            elif (self.battery_voltage_frame_color != "green"):
                self.battery_voltage_frame.setStyleSheet("QWidget {background-color:#B1EB00;}")
                self.battery_voltage_frame_color = "green"

            # Takes advantage of the periodic nature of the /battery_monitor node
            self.update_operating_mode_status()

    def update_system_time_status(self, msg):
        '''
        Updates the system time display when there is an update on the /clock
        node.
        '''
        msg_string = str(msg.clock)
        if ((msg.clock is not None) and (len(msg_string) > 9)):
            self.system_time_status.setText(msg_string[:-9] + "." + msg_string[-9:-7] + " s")

    def toggle_kill(self):
        '''
        Toggles the kill status when the toggle_kill_button is pressed.
        '''
        rospy.loginfo("Toggling Kill")

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
