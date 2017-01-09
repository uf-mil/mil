#!/usr/bin/env python
import rospy
from ros_alarms import AlarmListener, HeartbeatMonitor
from std_msgs.msg import String

def printit(alarm):
    rospy.loginfo("Alarm {} raised".format(alarm.alarm_name))

if __name__ == "__main__":
    rospy.init_node("heartbeat_test")

    pub = rospy.Publisher("/heartbeat", String, queue_size=1)
    al = AlarmListener("kill")
    al.add_callback(printit, call_when_cleared=False)
    hm = HeartbeatMonitor("kill", "/heartbeat", 1)
    hm.clear_alarm()

    rospy.loginfo("Timeout test")
    for i in range(20):
        pub.publish("testing 1, 2, 3")
        rospy.sleep(0.1)
    rospy.sleep(2)
    assert al.is_raised()
        
    rospy.loginfo("Revive test")
    pub.publish("testing 1, 2, 3")
    rospy.sleep(0.1)
    assert al.is_cleared()
    rospy.sleep(2)
    assert al.is_raised()

    del hm
    def parse(msg):
        return "test" in msg.data

    hm = HeartbeatMonitor("kill", "/heartbeat", 1, parse)
    hm.clear_alarm()

    rospy.loginfo("Timeout with Predicate test")
    for i in range(20):
        pub.publish("teting 1, 2, 3")
        rospy.sleep(0.1)
    assert al.is_cleared()
        
    rospy.loginfo("Timeout with Predicate test 2")
    pub.publish("testing 1, 2, 3")
    rospy.sleep(0.1)
    assert al.is_cleared()
    rospy.sleep(2)
    assert al.is_raised()
