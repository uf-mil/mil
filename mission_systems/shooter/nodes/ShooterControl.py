#!/usr/bin/env python
from Sabertooth2x12 import Sabertooth2x12
import rospy
import actionlib
import time
from navigator_msgs.msg import ShooterDoAction, ShooterDoActionFeedback, ShooterDoActionResult
from navigator_msgs.srv import ShooterManual
from std_srvs.srv import Trigger

class ShooterControl:
    def __init__(self):
        self.controller_file = rospy.get_param('~controller/controller_serial', '/dev/ttyUSB0')
        rospy.loginfo("Shooter connecting to motor controller at: %s", self.controller_file)
        self.load_retract_time = rospy.Duration(0, rospy.get_param('~controller/load/retract_time_millis', 950)*1000000)
        self.load_extend_time = rospy.Duration(0, rospy.get_param('~controller/load/extend_time_millis', 500)*1000000)
        self.load_pause_time = rospy.Duration(0, rospy.get_param('~controller/load/pause_time_millis', 100)*1000000)
        self.load_total_time = self.load_retract_time + self.load_pause_time + self.load_extend_time
        self.fire_extend_time = rospy.Duration(0, rospy.get_param('~controller/fire/extend_time_millis', 400)*1000000)
        self.fire_shoot_time = rospy.Duration(0, rospy.get_param('~controller/fire/shoot_time_millis', 1000)*1000000)
        self.total_fire_time = max(self.fire_shoot_time,self.fire_extend_time)
        self.loaded = False #Assume linear actuator starts forward
        self.motor_controller = Sabertooth2x12(self.controller_file)
        self.fire_server = actionlib.SimpleActionServer('/shooter/fire', ShooterDoAction, self.fire_execute_callback, False)
        self.fire_server.start()
        self.load_server = actionlib.SimpleActionServer('/shooter/load', ShooterDoAction, self.load_execute_callback, False)
        self.load_server.start()
        self.cancel_service = rospy.Service('/shooter/cancel', Trigger, self.cancel_callback)
        self.manual_service = rospy.Service('/shooter/manual', ShooterManual, self.manual_callback)


    def load_execute_callback(self, goal):
        result = ShooterDoActionResult()
        if self.fire_server.is_active():
            rospy.loginfo("Something already running, aborting")
            result.result.success = False
            result.result.error = result.result.ALREADY_RUNNING
            self.load_server.set_aborted(result.result)
            return
        if self.loaded:
            rospy.loginfo("Already loaded, aborting")
            result.result.success = False
            result.result.error = result.result.ALREADY_LOADED
            self.load_server.set_aborted(result.result)
            return
        start_time = rospy.get_rostime()
        dur_from_start = rospy.Duration(0, 0)
        rospy.loginfo("starting load")
        rate = rospy.Rate(50) # 50hz
        feedback = ShooterDoActionFeedback()
        while dur_from_start < self.load_total_time and self.load_server.is_active():
            dur_from_start = rospy.get_rostime() - start_time
            feedback.feedback.time_remaining = self.load_total_time - dur_from_start
            self.load_server.publish_feedback(feedback.feedback)
            if dur_from_start < self.load_retract_time:
                self.motor_controller.setMotor1(1.0)
            elif dur_from_start < self.load_retract_time + self.load_pause_time:
                self.motor_controller.setMotor1(0)
            elif dur_from_start < self.load_total_time:
                self.motor_controller.setMotor1(-1.0)
            rate.sleep()
        if not self.load_server.is_active():
            return
        self.motor_controller.setMotor1(0)
        self.motor_controller.setMotor2(1)
        result.result.success = True
        self.loaded = True
        self.load_server.set_succeeded(result.result)
        rospy.loginfo("Finished loaded")


    def fire_execute_callback(self, goal):
        result = ShooterDoActionResult()
        if self.load_server.is_active():
            rospy.loginfo("Something already running, aborting fire")
            result.result.success = False
            result.result.error = result.result.ALREADY_RUNNING
            self.fire_server.set_aborted(result.result)
            return
        if not self.loaded:
            rospy.loginfo("Not loaded, aborting fire")
            result.result.success = False
            result.result.error = result.result.NOT_LOADED
            self.fire_server.set_aborted(result.result)
            return
        start_time = rospy.get_rostime()
        dur_from_start = rospy.Duration(0, 0)
        rospy.loginfo("starting fire")
        rate = rospy.Rate(50) # 50hz
        feedback = ShooterDoActionFeedback()
        self.motor_controller.setMotor1(-1)
        self.motor_controller.setMotor2(1)
        while dur_from_start < self.total_fire_time and self.fire_server.is_active():
            dur_from_start = rospy.get_rostime() - start_time
            feedback.feedback.time_remaining = self.total_fire_time - dur_from_start
            self.fire_server.publish_feedback(feedback.feedback)
            if dur_from_start > self.fire_extend_time:
                self.motor_controller.setMotor1(0)
            if dur_from_start > self.fire_shoot_time:
                self.motor_controller.setMotor2(0)
        if not self.fire_server.is_active():
            return
        self.motor_controller.setMotor1(0)
        self.motor_controller.setMotor2(0)
        result.result.success = True
        self.loaded = False
        self.fire_server.set_succeeded(result.result)
        rospy.loginfo("Finished fire")


    def stop_actions(self):
        result = ShooterDoActionResult()
        result.result.success = False
        result.result.error = result.result.MANUAL_CONTROL_USED
        if self.load_server.is_active():
            self.load_server.set_aborted(result.result)
        if self.fire_server.is_active():
            self.fire_server.set_aborted(result.result)
        d = rospy.Duration(0, 100000000)
        rospy.sleep(d)
        time.sleep(0.1)      


    def cancel_callback(self, req):
        self.stop_actions()
        self.motor_controller.setMotor1(0)
        self.motor_controller.setMotor2(0)
        rospy.loginfo("canceled")
        return True
    def manual_callback(self, req):
        self.stop_actions()
        self.motor_controller.setMotor1(-req.feeder)
        self.motor_controller.setMotor2(req.shooter)
        rospy.loginfo("manual set")
        return True
        

if __name__ == '__main__':
    rospy.init_node('shooter_control')
    control = ShooterControl()
    #rate = rospy.Rate(10) # 10hz
    rospy.spin()
