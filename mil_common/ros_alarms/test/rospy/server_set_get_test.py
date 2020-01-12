#!/usr/bin/env python
import rospy
import numpy as np
import random
import json
from ros_alarms.srv import AlarmGet, AlarmGetRequest, AlarmSet, AlarmSetRequest

# Dummy params to pass in
parameters = {"testing": True, "not_real": 3, "a_list": [1, 2, 3]}
# Names to pick from for setting and getting
names = list(np.random.randint(50, size=20).astype('str'))
raised = []
cleared = []


def raise_some(setter, count=20):
    for i in range(count):
        a = AlarmSetRequest()
        a.alarm.alarm_name = random.choice(names)
        a.alarm.node_name = rospy.get_name()
        a.alarm.parameters = json.dumps(parameters)
        a.alarm.severity = 3
        a.alarm.raised = random.choice([True, False])

        if a.alarm.alarm_name in raised or a.alarm.alarm_name in cleared:
            continue

        setter(a)
        rospy.loginfo("{} '{}' alarm".format('Raised' if a.alarm.raised else 'Cleared', a.alarm.alarm_name))
        if a.alarm.raised:
            raised.append(a.alarm.alarm_name)
        else:
            cleared.append(a.alarm.alarm_name)


def check_some(getter, count=50):
    for i in range(count):
        a = AlarmGetRequest()
        a.alarm_name = random.choice(names)

        rospy.loginfo("Checking '{}' alarm".format(a.alarm_name))

        resp = getter(a)
        print "raised", resp.alarm.raised
        print "in_raised: ", a.alarm_name in raised
        print "in_cleared: ", a.alarm_name in cleared
        found = False
        if a.alarm_name in raised:
            found = True
            assert resp.alarm.raised
        elif a.alarm_name in cleared:
            found = True
            assert not resp.alarm.raised

        if found:
            assert json.loads(resp.alarm.parameters) == parameters


if __name__ == "__main__":
    rospy.init_node("alarm_set_get_test")

    setter = rospy.ServiceProxy("/alarm/set", AlarmSet)
    getter = rospy.ServiceProxy("/alarm/get", AlarmGet)

    print "Waiting for service"
    rospy.wait_for_service("/alarm/set")
    rospy.wait_for_service("/alarm/get")

    print "Running tests"

    raise_some(setter)
    print "raised ", raised
    print "cleared ", cleared
    check_some(getter)
