#!/usr/bin/env python
import rospy
from ros_alarms import AlarmListener, AlarmBroadcaster


if __name__ == "__main__":
    rospy.init_node("broadcaster_listener_test")

    ab = AlarmBroadcaster("test_alarm")
    al = AlarmListener("test_alarm")
    
    print "Inited"
    assert al.is_cleared()

    ab.raise_alarm()
    assert al.is_raised()

    rospy.sleep(0.5)
    ab.clear_alarm()
    assert al.is_cleared()

    rospy.sleep(0.5)
    ab.raise_alarm(parameters={"int": 4, "list": [1, 2, 3], "str": "stringing"})

    print "All checks passed"
