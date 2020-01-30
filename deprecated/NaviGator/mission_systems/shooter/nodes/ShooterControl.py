#!/usr/bin/env python
from Sabertooth2x12 import Sabertooth2x12
import rospy
import actionlib
from ros_alarms import AlarmListener
from navigator_msgs.msg import ShooterDoAction, ShooterDoActionFeedback, ShooterDoActionResult
from navigator_msgs.srv import ShooterManual, ShooterManualResponse
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from std_msgs.msg import String


class ShooterControl:
    def __init__(self):
        self.controller_file = rospy.get_param('~controller/controller_serial', '/dev/ttyUSB0')
        use_sim = rospy.get_param('/is_simulation', False)
        if use_sim:
            rospy.loginfo("Shooter controller running in simulation mode")
            print "Running in sim"
            self.motor_controller = Sabertooth2x12(self.controller_file, sim=True)
        else:
            self.motor_controller = Sabertooth2x12(self.controller_file)
        rospy.loginfo("Shooter connecting to motor controller at: %s", self.controller_file)
        self.load_retract_time = rospy.Duration(0, rospy.get_param(
            '~controller/load/retract_time_millis', 950) * 1000000)
        self.load_extend_time = rospy.Duration(0, rospy.get_param('~controller/load/extend_time_millis', 500) * 1000000)
        self.load_pause_time = rospy.Duration(0, rospy.get_param('~controller/load/pause_time_millis', 100) * 1000000)
        self.load_total_time = self.load_retract_time + self.load_pause_time + self.load_extend_time
        self.fire_retract_time = rospy.Duration(0, rospy.get_param(
            '~controller/fire/retract_time_millis', 400) * 1000000)
        self.fire_shoot_time = rospy.Duration(0, rospy.get_param('~controller/fire/shoot_time_millis', 1000) * 1000000)
        self.total_fire_time = max(self.fire_shoot_time, self.fire_retract_time)
        self.loaded = False  # Assume linear actuator starts forward
        self.stop = False
        self.motor1_stop = 0
        self.motor2_stop = 0
        self.fire_server = actionlib.SimpleActionServer(
            '/shooter/fire', ShooterDoAction, self.fire_execute_callback, False)
        self.fire_server.start()
        self.load_server = actionlib.SimpleActionServer(
            '/shooter/load', ShooterDoAction, self.load_execute_callback, False)
        self.load_server.start()
        self.cancel_service = rospy.Service('/shooter/cancel', Trigger, self.cancel_callback)
        self.manual_service = rospy.Service('/shooter/manual', ShooterManual, self.manual_callback)
        self.reset_service = rospy.Service('/shooter/reset', Trigger, self.reset_callback)
        self.status = "Standby"
        self.status_pub = rospy.Publisher('/shooter/status', String, queue_size=5)
        self.manual_used = False
        self.killed = False
        self.kill_listener = AlarmListener("kill", callback_funct=self.update_kill_status)

    def update_kill_status(self, alarm):
        '''
        Updates the kill status display when there is an update on the kill
        alarm.
        '''
        if (not alarm.raised and self.killed):
            self.killed = False
        elif (not self.killed) and (alarm.raised):
            self.cancel_callback(TriggerRequest())
            self.killed = True

    def load_execute_callback(self, goal):
        result = ShooterDoActionResult()
        if self.killed:
            rospy.loginfo("Load action called when killed, aborting")
            result.result.success = False
            result.result.error = result.result.KILLED
            self.load_server.set_aborted(result.result)
            return
        if self.manual_used:
            rospy.loginfo("Load action called after manual used, aborting")
            result.result.success = False
            result.result.error = result.result.MANUAL_CONTROL_USED
            self.load_server.set_aborted(result.result)
            return
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
        rate = rospy.Rate(50)  # 50hz
        feedback = ShooterDoActionFeedback()
        self.status = "Loading"
        # self.motor_controller.setMotor2(-1)
        while dur_from_start < self.load_total_time and not self.stop:
            dur_from_start = rospy.get_rostime() - start_time
            feedback.feedback.time_remaining = self.load_total_time - dur_from_start
            self.load_server.publish_feedback(feedback.feedback)
            self.load_extend_time
            # Extend to allow ball to drop in
            if dur_from_start < self.load_extend_time:
                self.motor_controller.setMotor1(-1.0)
            # Pause a bit for testing / let ball settle
            elif dur_from_start < self.load_extend_time + self.load_pause_time:
                self.motor_controller.setMotor1(0)
            # Retract for remainder of time
            elif dur_from_start < self.load_total_time:
                self.motor_controller.setMotor1(1.0)
            rate.sleep()
        if self.stop:
            result = ShooterDoActionResult()
            result.result.success = False
            result.result.error = result.result.MANUAL_CONTROL_USED
            self.motor_controller.setMotor1(self.motor1_stop)
            self.motor_controller.setMotor2(self.motor2_stop)
            self.stop = False
            self.load_server.set_preempted(result=result.result)
            return
        self.status = "Loaded"
        self.motor_controller.setMotor1(0)
        self.motor_controller.setMotor2(-1)
        result.result.success = True
        self.loaded = True
        self.load_server.set_succeeded(result.result)
        rospy.loginfo("Finished loaded")

    def fire_execute_callback(self, goal):
        result = ShooterDoActionResult()
        if self.killed:
            rospy.loginfo("Fire action called when killed, aborting")
            result.result.success = False
            result.result.error = result.result.KILLED
            self.fire_server.set_aborted(result.result)
            return
        if self.manual_used:
            rospy.loginfo("Fire action called after manual used, aborting")
            result.result.success = False
            result.result.error = result.result.MANUAL_CONTROL_USED
            self.fire_server.set_aborted(result.result)
            return
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
        feedback = ShooterDoActionFeedback()
        self.motor_controller.setMotor1(1.0)
        self.motor_controller.setMotor2(-1.0)
        self.status = "Firing"
        while dur_from_start < self.total_fire_time and not self.stop:
            dur_from_start = rospy.get_rostime() - start_time
            feedback.feedback.time_remaining = self.total_fire_time - dur_from_start
            self.fire_server.publish_feedback(feedback.feedback)
            if dur_from_start > self.fire_retract_time:
                self.motor_controller.setMotor1(0)
            if dur_from_start > self.fire_shoot_time:
                self.motor_controller.setMotor2(0)
        if self.stop:
            result.result.success = False
            result.result.error = result.result.MANUAL_CONTROL_USED
            self.motor_controller.setMotor1(self.motor1_stop)
            self.motor_controller.setMotor2(self.motor2_stop)
            self.stop = False
            self.fire_server.set_preempted(result=result.result)
            return
        self.status = "Standby"
        self.motor_controller.setMotor1(0)
        self.motor_controller.setMotor2(0)
        result.result.success = True
        self.loaded = False
        self.fire_server.set_succeeded(result.result)
        rospy.loginfo("Finished fire")

    def stop_actions(self):
        if self.load_server.is_active() or self.fire_server.is_active():
            self.stop = True
            return True
        else:
            self.stop = False
            self.motor_controller.setMotor1(self.motor1_stop)
            self.motor_controller.setMotor2(self.motor2_stop)
            self.motor1_stop = 0
            self.motor2_stop = 0
            return False

    def cancel_callback(self, req):
        self.motor1_stop = 0
        self.motor2_stop = 0
        if self.stop_actions():
            self.status = "Canceled"
            self.manual_used = True
        rospy.loginfo("canceled")
        return TriggerResponse(success=True)

    def manual_callback(self, req):
        res = ShooterManualResponse()
        # if self.killed:
        #    rospy.loginfo("Manual control called when killed, aborting")
        #    res.success = False
        #    return res
        self.status = "Manual"
        self.motor1_stop = -req.feeder
        self.motor2_stop = req.shooter
        self.manual_used = True
        self.stop_actions()
        res = ShooterManualResponse()
        res.success = True
        return res

    def reset_callback(self, data):
        self.status = "Standby"
        self.stop_actions()
        self.manual_used = False
        self.loaded = False
        return TriggerResponse(success=True)

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.status_pub.publish(self.status)
            r.sleep()


if __name__ == '__main__':
    rospy.init_node('shooter_control')
    control = ShooterControl()
    control.run()
