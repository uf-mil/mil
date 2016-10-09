#!/usr/bin/env python

'''
Dashboard: A graphical interface to view the status of various systems on
Navigator and to select one of the control inputs to use.
'''


import os

from kill_handling.broadcaster import KillBroadcaster
from navigator_alarm import AlarmBroadcaster
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

        # Create the widget
        self._widget = QtGui.QWidget()
        self.setObjectName('Dashboard')

        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('navigator_gui'), 'resource', 'dashboard.ui')

        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)

        # Give QObjects reasonable names
        self._widget.setObjectName('Navigator_gui')

        # Link objects in the GUI to their handler functions
        self.battery_voltage_status = self._widget.findChild(QtGui.QLabel, 'battery_voltage_status')
        self.system_time_status = self._widget.findChild(QtGui.QLabel, 'system_time_status')
        toggle_kill_button = self._widget.findChild(QtGui.QPushButton, 'toggle_kill_button')
        toggle_kill_button.clicked.connect(self.toggle_kill)
        station_hold_button = self._widget.findChild(QtGui.QPushButton, 'station_hold_button')
        station_hold_button.clicked.connect(self.station_hold)
        autonomous_control_button = self._widget.findChild(QtGui.QPushButton, 'autonomous_control_button')
        autonomous_control_button.clicked.connect(self.select_autonomous_control)
        rc_control_button = self._widget.findChild(QtGui.QPushButton, 'rc_control_button')
        rc_control_button.clicked.connect(self.select_rc_control)

        # Subscribes to topics that are monitored on the GUI
        rospy.Subscriber("/battery_monitor", Float32, self.update_battery_voltage)
        rospy.Subscriber("/clock", Clock, self.update_system_time)

        # Connect to services that can be controlled from the GUI
        self.wrench_changer = rospy.ServiceProxy('/change_wrench', WrenchSelect)
        self.kill_broadcaster = KillBroadcaster(id='station_hold', description='Resets Pose')

        # Sets up the dashboard controlled alarms
        alarm_broadcaster = AlarmBroadcaster()
        self.kill_alarm = alarm_broadcaster.add_alarm(
            name='kill',
            action_required=True,
            severity=0
        )

        # Dashboard state parameters
        self.is_killed = False

        # Show _widget.windowTitle on left-top of each plugin (when it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

    def update_battery_voltage(self, msg):
        '''
        Updates the battery voltage display when there is an update on the
        /battery_monitor node.
        '''
        self.battery_voltage_status.setText(str(msg.data))

    def update_system_time(self, msg):
        '''
        Updates the system time display when there is an update on the /clock
        node.
        '''
        if ((msg.clock is not None) and (len(str(msg.clock)) > 9)):
            self.system_time_status.setText(str(msg.clock)[:-9] + "." + str(msg.clock)[-9:-7] + " s")

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
        self.kill_broadcaster.send(active=True)
        self.kill_broadcaster.send(active=False)
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
