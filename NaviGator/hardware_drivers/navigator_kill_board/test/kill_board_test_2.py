#!/usr/bin/env python3
import unittest
from copy import copy, deepcopy
from threading import Lock

import rospy
from mil_tools import thread_lock
from navigator_alarm_handlers import NetworkLoss
from ros_alarms import AlarmBroadcaster, AlarmListener
from std_msgs.msg import Header

lock = Lock()


class killtest(unittest.TestCase):
    def __init__(self, *args):
        self.hw_kill_alarm = None
        self.network_kill_alarm = None
        self.hardware_updated = False
        self.network_updated = False
        self.AlarmListener = AlarmListener("hw-kill", self._hw_kill_cb)
        self.NetworkListener = AlarmListener("network-loss", self._network_kill_cb)
        self.AlarmBroadcaster = AlarmBroadcaster("kill")
        self.network = NetworkLoss()
        super().__init__(*args)
        # self.heartbeat = HeartbeatServer("/network", 1.0)

    @thread_lock(lock)
    def reset_update(self):
        """
        Reset update state to False so we can notice changes to hw-kill
        """
        self.hardware_updated = False
        self.network_updated = False

    @thread_lock(lock)
    def _hw_kill_cb(self, alarm):
        """
        If the raised status changed, set update flag to true so test an notice change
        Called on change in hw-kill alarm.
        """
        if self.hw_kill_alarm is None or alarm.raised != self.hw_kill_alarm.raised:
            self.hardware_updated = True
        self.hw_kill_alarm = alarm

    @thread_lock(lock)
    def _network_kill_cb(self, alarm):
        """
        Called on change in network-kill alarm.
        If the raised status changed, set update flag to true so test an notice change
        """
        if (
            self.network_kill_alarm is None
            or alarm.raised != self.network_kill_alarm.raised
        ):
            self.network_updated = True
        self.network_kill_alarm = alarm

    def wait_for_kill_update(self, timeout=rospy.Duration(0.5), ver=None):
        """
        Wait up to timeout time to an hw-kill alarm change. Returns a copy of the new alarm or throws if times out
        """
        timeout = rospy.Time.now() + timeout
        while rospy.Time.now() < timeout:
            lock.acquire()
            hardware_updated = copy(self.hardware_updated)
            network_updated = copy(self.network_updated)
            alarm = (deepcopy(self.hw_kill_alarm), deepcopy(self.network_kill_alarm))
            lock.release()
            if hardware_updated or network_updated:
                return alarm
            rospy.sleep(0.01)
        raise Exception("timeout")

    def assert_raised(self, timeout=rospy.Duration(1)):
        """
        Waits for update and ensures it is now raised
        """
        alarm = self.wait_for_kill_update(timeout)
        hardware_pass = alarm[0].raised if alarm[0] is not None else True
        network_pass = alarm[1].raised if alarm[1] is not None else True
        self.assertEqual(hardware_pass and network_pass, True)

    def assert_cleared(self, timeout=rospy.Duration(1)):
        """
        Wait for update and ensures is now cleared
        """
        alarm = self.wait_for_kill_update(timeout)
        hardware_pass = alarm[0].raised if alarm[0] is not None else True
        network_pass = alarm[1].raised if alarm[1] is not None else True
        self.assertEqual(hardware_pass and network_pass, False)

    def test_2_computer_kill(self):
        """
        Test raising/clearing kill alarm (user kill) will cause same change in hw-kill
        """
        pub = rospy.Publisher("/network", Header, queue_size=10)
        rospy.sleep(4)
        self.reset_update()
        pub.publish(Header())
        self.AlarmBroadcaster.clear_alarm()
        pub.publish(Header())
        rospy.sleep(2)
        pub.publish(Header())
        rospy.sleep(0.5)
        pub.publish(Header())
        rospy.sleep(0.5)
        pub.publish(Header())
        self.assert_cleared()

        self.reset_update()
        self.AlarmBroadcaster.raise_alarm()
        rospy.sleep(2)
        self.assert_raised()

        self.reset_update()
        pub.publish(Header())
        self.AlarmBroadcaster.clear_alarm()
        self.assert_cleared()


if __name__ == "__main__":
    rospy.init_node("lll", anonymous=True)
    import rostest

    rostest.rosrun("navigator_kill_board", "kill_board_test_2", killtest)
    unittest.main()
