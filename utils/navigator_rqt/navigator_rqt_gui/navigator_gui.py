import os
import rospy
import rospkg
from sensor_msgs.msg import Joy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import WrenchStamped
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
import python_qt_binding.QtGui as qtg
from navigator_msg_multiplexer.srv import wrench_arbiter
import time
from geometry_msgs.msg import Point


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

        self.wrench_out = WrenchStamped()

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
        revive_button.clicked.connect(self.revive)

        revive_button = self._widget.findChild(qtg.QPushButton, 'move_send')
        revive_button.clicked.connect(self.relative_move)

        station_hold_button = self._widget.findChild(qtg.QPushButton, 'station_hold')
        station_hold_button.clicked.connect(self.station_hold)
        station_hold_button = self._widget.findChild(qtg.QPushButton, 'docking_mode')
        station_hold_button.clicked.connect(self.toggle_docking)
        rc_mode_button = self._widget.findChild(qtg.QPushButton, 'rc_mode')
        rc_mode_button.clicked.connect(self.rc_mode)
        au_mode_button = self._widget.findChild(qtg.QPushButton, 'au_mode')
        au_mode_button.clicked.connect(self.au_mode)
        gui_mode_button = self._widget.findChild(qtg.QPushButton, 'gui_mode')
        gui_mode_button.clicked.connect(self.gui_mode)
        forward_slider = self._widget.findChild(qtg.QSlider, 'forward_slider')
        forward_slider.valueChanged.connect(self.forward_slider)
        backward_slider = self._widget.findChild(qtg.QSlider, 'backward_slider')
        backward_slider.valueChanged.connect(self.backward_slider)
        right_slider = self._widget.findChild(qtg.QSlider, 'right_slider')
        right_slider.valueChanged.connect(self.right_slider)
        left_slider = self._widget.findChild(qtg.QSlider, 'left_slider')
        left_slider.valueChanged.connect(self.left_slider)
        yaw_right_slider = self._widget.findChild(qtg.QSlider, 'yaw_right_slider')
        yaw_right_slider.valueChanged.connect(self.yaw_right_slider)
        yaw_left_slider = self._widget.findChild(qtg.QSlider, 'yaw_left_slider')
        yaw_left_slider.valueChanged.connect(self.yaw_left_slider)

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

        self.wrench_pub = rospy.Publisher("/wrench/gui", WrenchStamped, queue_size=1)
        self.move_pub = rospy.Publisher('/move_helper', Point, queue_size=1)
        self.wrench_changer = rospy.ServiceProxy('/change_wrench', wrench_arbiter)
        rospy.Subscriber("/tf", TFMessage, self.wrench_pub_cb)


    def wrench_pub_cb(self, msg):
        self.wrench_pub.publish(self.wrench_out)

    def rc_mode(self):
        self.wrench_changer("rc")

    def au_mode(self):
        self.wrench_changer("autonomous")

    def gui_mode(self):
        self.wrench_changer("gui")


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

    def revive(self):
        msg = Joy()

        msg.buttons.append(0)
        msg.buttons.append(1)
        for x in xrange(0,7):
            msg.buttons.append(0)
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

    def relative_move(self):
        x = self._widget.findChild(qtg.QPlainTextEdit, 'x_send').toPlainText()
        y = self._widget.findChild(qtg.QPlainTextEdit, 'y_send').toPlainText()
        z = self._widget.findChild(qtg.QPlainTextEdit, 'z_send').toPlainText()
        x,y,z = float(x), float(y), float(z)
        to_send = Point(x,y,z)
        self.move_pub.publish(to_send)



    def forward_slider(self, value):
        if self.wrench_out.wrench.force.x >= 0:
            self.wrench_out.wrench.force.x = 5 * value

    def backward_slider(self, value):
        if self.wrench_out.wrench.force.x <= 0:
            self.wrench_out.wrench.force.x = -5 * value

    def right_slider(self, value):
        if self.wrench_out.wrench.force.y <= 0:
            self.wrench_out.wrench.force.y = -5 * value

    def left_slider(self, value):
        if self.wrench_out.wrench.force.y >= 0:
            self.wrench_out.wrench.force.y = 5 * value

    def yaw_right_slider(self, value):
        if self.wrench_out.wrench.torque.z <= 0:
            self.wrench_out.wrench.torque.z = -5 * value

    def yaw_left_slider(self, value):
        if self.wrench_out.wrench.torque.z >= 0:
            self.wrench_out.wrench.torque.z = 5 * value

    def forward_slider(self, value):
        if self.wrench_out.wrench.force.x >= 0:
            self.wrench_out.wrench.force.x = 5 * value