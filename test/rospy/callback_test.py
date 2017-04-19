#!/usr/bin/env python
import rospy
from ros_alarms import AlarmListener, AlarmBroadcaster

cb1_ran = False
cb2_ran = False

def cb1(alarm):
    global cb1_ran
    print alarm.alarm_name, alarm.raised
    cb1_ran = True
    print alarm
    print "Callback1!"

def cb2(alarm):
    global cb2_ran
    cb2_ran = True
    print "Callback2!"

if __name__ == "__main__":
    rospy.init_node("callback_test")

    ab = AlarmBroadcaster("test_alarm")
    al = AlarmListener("test_alarm")
    ab.clear_alarm()
    rospy.sleep(0.1)

    al.add_callback(cb1)

    print "Inited"
    assert al.is_cleared()
    
    rospy.loginfo("Raise Test")
    ab.raise_alarm()
    rospy.sleep(0.1)
    assert al.is_raised()
    assert cb1_ran
    cb1_ran = False

    al.clear_callbacks()

    al.add_callback(cb1, severity_required=2)
    al.add_callback(cb2, call_when_raised=False)
    
    rospy.loginfo("Severity Fail Test")
    ab.raise_alarm(severity=3)
    rospy.sleep(0.1)
    assert not cb1_ran
    assert not cb2_ran

    rospy.loginfo("Severity Pass Test")
    ab.raise_alarm(severity=2)
    rospy.sleep(0.1)
    assert cb1_ran
    assert not cb2_ran
    cb1_ran = False

    rospy.loginfo("Clear Callback Test")
    ab.clear_alarm()
    rospy.sleep(0.1)
    assert cb1_ran
    assert cb2_ran
    cb1_ran = False
    
    al.clear_callbacks()

    al.add_callback(cb1, severity_required=(0, 3))
    al.add_callback(cb2, severity_required=(2, 4))
    rospy.sleep(0.1)
    
    rospy.loginfo("Severity Range Test 1")
    ab.raise_alarm(severity=4)
    rospy.sleep(0.1)
    assert not cb1_ran
    assert cb2_ran
    cb2_ran = False

    rospy.loginfo("Severity Range Test 2")
    ab.raise_alarm(severity=1)
    rospy.sleep(0.1)
    assert cb1_ran
    assert not cb2_ran
    cb1_ran = False

    print "All checks passed"
