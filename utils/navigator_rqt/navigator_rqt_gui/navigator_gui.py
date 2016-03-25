import os
import rospy
import rospkg
from sensor_msgs.msg import Joy
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
import python_qt_binding.QtGui as qtg
import time


class Navigator_gui(Plugin):

    def __init__(self, context):
        super(Navigator_gui, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Navigator_gui')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            pass
            #print 'arguments: ', args
            #print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('navigator_rqt'), 'resource', 'navigator_gui.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('Navigator_gui')

        kill_button = self._widget.findChild(qtg.QPushButton, 'kill')
        kill_button.clicked.connect(self.toggle_kill)

        revive_button = self._widget.findChild(qtg.QPushButton, 'revive')
        revive_button.clicked.connect(self.toggle_kill)

        station_hold_button = self._widget.findChild(qtg.QPushButton, 'station_hold')
        station_hold_button.clicked.connect(self.station_hold)


        station_hold_button = self._widget.findChild(qtg.QPushButton, 'docking_mode')
        station_hold_button.clicked.connect(self.toggle_docking)


        rc_mode_button = self._widget.findChild(qtg.QPushButton, 'rc_mode')
        rc_mode_button.clicked.connect(self.toggle_mode)
        au_mode_button = self._widget.findChild(qtg.QPushButton, 'au_mode')
        au_mode_button.clicked.connect(self.toggle_mode)

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to

        self.joy_pub = rospy.Publisher("/joy", Joy, queue_size = 1)
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

    def toggle_mode(self):
        msg = Joy()
        for x in xrange(0,7):
            msg.buttons.append(0)
        for x in xrange(0,4):
            msg.axes.append(0)
        msg.buttons.append(1)
        msg.buttons.append(0)
        self.joy_pub.publish(msg)
        time.sleep(.1)

        clr_msg = Joy()

        for x in xrange(0,7):
            clr_msg.buttons.append(0)
        for x in xrange(0,4):
            clr_msg.axes.append(0)
        clr_msg.buttons.append(0)
        clr_msg.buttons.append(0)
        self.joy_pub.publish(clr_msg)

    def station_hold(self):
        msg = Joy()
        msg.buttons.append(1)
        for x in xrange(0,8):
            msg.buttons.append(0)
        for x in xrange(0,4):
            msg.axes.append(0)
        self.joy_pub.publish(msg)
        time.sleep(.1)

        clr_msg = Joy()

        clr_msg.buttons.append(0)
        for x in xrange(0,8):
            clr_msg.buttons.append(0)
        for x in xrange(0,4):
            clr_msg.axes.append(0)
        self.joy_pub.publish(clr_msg)

    def toggle_kill(self):
        msg = Joy()
        for x in xrange(0,8):
            msg.buttons.append(0)
        msg.buttons.append(1)
        for x in xrange(0,4):
            msg.axes.append(0)
        self.joy_pub.publish(msg)
        time.sleep(.1)

        clr_msg = Joy()

        for x in xrange(0,8):
            clr_msg.buttons.append(0)
        clr_msg.buttons.append(0)
        for x in xrange(0,4):
            clr_msg.axes.append(0)
        self.joy_pub.publish(clr_msg)

    def toggle_docking(self):
        msg = Joy()
        for x in xrange(0,6):
            msg.buttons.append(0)
        msg.buttons.append(1)
        msg.buttons.append(0)
        msg.buttons.append(0)
        for x in xrange(0,4):
            msg.axes.append(0)
        self.joy_pub.publish(msg)
        time.sleep(.1)

        clr_msg = Joy()

        for x in xrange(0,6):
            clr_msg.buttons.append(0)
        clr_msg.buttons.append(0)
        clr_msg.buttons.append(0)
        clr_msg.buttons.append(0)
        for x in xrange(0,4):
            clr_msg.axes.append(0)
        self.joy_pub.publish(clr_msg)


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog