#! /usr/bin/env python3
from __future__ import annotations

import json
import unittest

import rospy
import rostest
from ros_alarms import AlarmListener
from ros_alarms_msgs.msg import Alarm
from std_msgs.msg import String


class TemperatureAlarmTest(unittest.TestCase):
    def setUp(self):
        self.listener = AlarmListener("temp")
        self.alive = False
        self.set_values_pub = rospy.Publisher(
            "/subjugator_alarm_temp_test_values",
            String,
            queue_size=5,
        )
        self.alarm_updates_sub = rospy.Subscriber(
            "/alarm/updates",
            Alarm,
            self._updates_cb,
        )

    def _make_data(
        self,
        current: float,
        high: float,
        critical: float,
    ) -> dict[str, list[tuple[str, float, float, float]]]:
        # label: current, high, critical
        return {"test_temp_device": [("label", current, high, critical)]}

    def _updates_cb(self, alarm: Alarm):
        if alarm.alarm_name == "temp":
            self.alive = True

    def _send_and_test(self, current: float, high: float, critical: float):
        d = self._make_data(current, high, critical)
        for _ in range(15):
            self.set_values_pub.publish(String(json.dumps(d)))
            rospy.sleep(0.1)

    def test_sequence(self):
        self.listener.wait_for_server()
        # current > critical
        self._send_and_test(2, 1, 5)
        self.assertTrue(self.listener.is_raised())
        # normal
        self._send_and_test(2, 500, 500)
        self.assertTrue(self.listener.is_cleared())
        # current > high
        self._send_and_test(2, 5, 1)
        self.assertTrue(self.listener.is_raised())
        # normal
        self._send_and_test(0, 0, 0)
        self.assertTrue(self.listener.is_cleared())
        # current within 90% of high
        self._send_and_test(91, 200, 100)
        self.assertTrue(self.listener.is_raised())


if __name__ == "__main__":
    rospy.init_node("temp_test", anonymous=True)
    rostest.rosrun("subjugator_alarm", "temp_test", TemperatureAlarmTest)
    unittest.main()
