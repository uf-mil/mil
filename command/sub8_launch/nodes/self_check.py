#!/usr/bin/python
import txros
from sub8_tools import text_effects
from twisted.internet import defer, reactor
import sys


MESSAGE_TIMEOUT = 1  # s

class TemplateChecker(object):
    '''Template for how each checker class should look.
    This provides interface functions for the main function to use. You don't have to
        implement them all in each checker.
    '''
    def __init__(self, nh, name):
        self.nh = nh
        self.name = name

        self.p = text_effects.Printer()

    @txros.util.cancellableInlineCallbacks
    def tx_init(self):
        # Do txros initiation here
        yield
        pass

    def do_check(self):
        # Do acutal checking here, either pass or fail
        pass

    def pass_check(self, status="", name=None):
        # Message that prints when the check passes
        if name is None:
            name = self.name
        pass_text = self.p.text("[ ").set_green.bold("PASS").text(" ]")
        pass_text += self.p.space().bold(name)
        if status is not "":
            pass_text += self.p.text(": {}".format(status))

        print pass_text

    def warn_check(self, reason="", name=None):
        # Message that prints when the check has a warning
        if name is None:
            name = self.name
        warn_text = self.p.text("[ ").set_yellow.bold("WARN").text(" ]")
        warn_text += self.p.space().bold(name)
        if reason is not "":
            warn_text += self.p.text(": {}".format(reason))

        print warn_text

    def fail_check(self, reason="", name=None):
        # Message that prints when the check fails
        if name is None:
            name = self.name
        fail_text = self.p.text("[ ").set_red.bold("FAIL").text(" ]")
        fail_text += self.p.space().bold(name)
        if reason is not "":
            fail_text += self.p.text(": {}".format(reason))

        print fail_text

    def err_msg(self, error, line_number):
        # If there is an error on runtime, this runs
        fail_text = self.p.text("[ ").set_red.bold(" ERR").text(" ]")
        fail_text += self.p.space().bold(self.name).text(": ({})".format(line_number))
        fail_text += self.p.space().text("{}".format(error))

        print fail_text

from sub8_msgs.msg import ThrusterStatus
class ThrusterChecker(TemplateChecker):
    @txros.util.cancellableInlineCallbacks
    def tx_init(self):
        self.thruster_topic = self.nh.subscribe("thrusters/thruster_status", ThrusterStatus)

        busses = yield self.nh.get_param("/busses")

        self.found_thrusters = {}
        for bus in busses:
            for thruster in bus.get("thrusters"):
                self.found_thrusters[thruster] = False
        print self.p.bold("  >>>>   ").set_blue.bold("Thruster Check")

    @txros.util.cancellableInlineCallbacks
    def do_check(self):
        try: 
            passed = yield txros.util.wrap_timeout(self.get_all_thrusters(), MESSAGE_TIMEOUT)
        except txros.util.TimeoutError:
            if not any(self.found_thrusters.values()):
                self.fail_check("no messages found.")
            elif self.found_thrusters.values().count(False) > 1:
                self.fail_check("more than one failed thruster.")
            elif self.found_thrusters.values().count(False) == 1:
                self.warn_check("one thruster is out, things should still work.")
            else:
                self.warn_check("unknown timeout reason.")

            defer.returnValue(False)

        if passed:
            self.pass_check("all thrusters up.")
        else:
            self.warn_check("unkown failure.")

    @txros.util.cancellableInlineCallbacks
    def get_all_thrusters(self):
        while True:
            msg = yield self.thruster_topic.get_next_message()
            self.found_thrusters[msg.name] = True

            if all(self.found_thrusters.values()):
                defer.returnValue(True)

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
class CameraChecker(TemplateChecker):
    @txros.util.cancellableInlineCallbacks
    def tx_init(self):
        self.right = self.nh.subscribe("/stereo/right/image_rect_color", Image)
        self.right_info = self.nh.subscribe("/stereo/right/camera_info", CameraInfo)
        self.left = self.nh.subscribe("/stereo/left/image_rect_color", Image)
        self.left_info = self.nh.subscribe("/stereo/left/camera_info", CameraInfo)
        self.down = self.nh.subscribe("/down/left/image_rect_color", Image)  # TODO
        self.down_info = self.nh.subscribe("/down/left/camera_info", CameraInfo)  # TODO
        
        self.subs = [("Right Image", self.right.get_next_message()), ("Right Info", self.right_info.get_next_message()),
                     ("Left Image", self.left.get_next_message()), ("Left Info", self.left_info.get_next_message()),
                     ("Down Image", self.down.get_next_message()), ("Down Info", self.down_info.get_next_message())]

        print self.p.bold("\n  >>>>   ").set_blue.bold("Camera Check")
        yield self.nh.sleep(0.1)  # Try to get all the images
        

    @txros.util.cancellableInlineCallbacks
    def do_check(self):
        passed = True
        for name, df in self.subs:
            try: 
                yield txros.util.wrap_timeout(df, MESSAGE_TIMEOUT)
            except txros.util.TimeoutError:
                self.fail_check("no messages found.", name)
                passed = False
                continue

            self.pass_check("message found.", name)

from nav_msgs.msg import Odometry
from tf.msg import tfMessage
from uf_common.msg import VelocityMeasurements, Float64Stamped
from sensor_msgs.msg import Imu, MagneticField
class StateEstChecker(TemplateChecker):
    @txros.util.cancellableInlineCallbacks
    def tx_init(self):
        self.odom = self.nh.subscribe("/odom", Odometry)
        self.tf = self.nh.subscribe("/tf", tfMessage)
        self.dvl = self.nh.subscribe("/dvl", VelocityMeasurements)
        self.depth = self.nh.subscribe("/depth", Float64Stamped)
        self.height = self.nh.subscribe("/dvl/range", Float64Stamped)
        self.imu = self.nh.subscribe("/imu/data_raw", Imu)
        self.mag = self.nh.subscribe("/imu/mag", MagneticField)

        self.subs = [("Odom", self.odom.get_next_message()), ("TF", self.tf.get_next_message()),
                     ("DVL", self.dvl.get_next_message()), ("Height", self.height.get_next_message()),
                     ("Depth", self.depth.get_next_message()), ("IMU", self.imu.get_next_message()), 
                     ("Mag", self.mag.get_next_message())]

        print self.p.bold("\n  >>>>   ").set_blue.bold("State Estimation Check")
        yield self.nh.sleep(0.1)  # Try to get all the subs

    @txros.util.cancellableInlineCallbacks
    def do_check(self):
        passed = True
        for name, df in self.subs:
            try: 
                res = yield txros.util.wrap_timeout(df, MESSAGE_TIMEOUT)
            except txros.util.TimeoutError:
                self.fail_check("no messages found.", name)
                passed = False
                continue

            if name == "DVL":
                len_measurements = len(res.velocity_measurements)
                if len_measurements == 4:
                    self.pass_check("all four beams being published.", name)
                elif len_measurements == 0:
                    self.fail_check("no beams being published", name)
                else:
                    self.warn_check("not all beams are being published", name)
            else:
                self.pass_check("message found.", name)

from std_msgs.msg import Header
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
class ShoreControlChecker(TemplateChecker):
    @txros.util.cancellableInlineCallbacks
    def tx_init(self):
        p = self.p.bold("\n  >>>>").text("   Press return when ").negative("shore_control.launch").text(" is running.\n")
        yield txros.util.nonblocking_raw_input(str(p))

        self.keepalive = self.nh.subscribe("/keep_alive", Header)
        self.spacenav = self.nh.subscribe("/spacenav/joy", Joy)
        self.posegoal = self.nh.subscribe("/posegoal", PoseStamped)

        self.subs = [("Keep Alive", self.keepalive.get_next_message()), ("Spacenav", self.spacenav.get_next_message()),
                     ("Pose Goal", self.posegoal.get_next_message())]

        print self.p.bold("  >>>>   ").set_blue.bold("Shore Control Check")
        yield self.nh.sleep(0.1)  # Try to get all the subs

    @txros.util.cancellableInlineCallbacks
    def do_check(self):
        passed = True
        for name, df in self.subs:
            try: 
                res = yield txros.util.wrap_timeout(df, MESSAGE_TIMEOUT)
            except txros.util.TimeoutError:
                self.fail_check("no messages found.", name)
                passed = False
                continue

            self.pass_check("message found.", name)

@txros.util.cancellableInlineCallbacks
def main():
    nh = yield txros.NodeHandle.from_argv("startup_checker")
    check_order = [ThrusterChecker(nh, "Thrusters"), CameraChecker(nh, "Cameras"), 
                   StateEstChecker(nh, "State Estimation"), ShoreControlChecker(nh, "Shore Control")]
    
    p = text_effects.Printer()
    yield txros.util.nonblocking_raw_input(str(p.bold("\n  >>>>").text("   Press return when ").negative("sub8.launch").text(" is running.")))
    print p.newline().set_blue.bold("-------- Running self checks...").newline()

    for check in check_order:
        try:
            yield check.tx_init()
            yield nh.sleep(0.1)
            yield check.do_check()
        except Exception as e:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            check.err_msg(e, exc_tb.tb_lineno)

        del check

    print p.newline().set_blue.bold("-------- Finished!").newline()
    reactor.stop()

if __name__ == '__main__':
    reactor.callWhenRunning(main)
    reactor.run()
