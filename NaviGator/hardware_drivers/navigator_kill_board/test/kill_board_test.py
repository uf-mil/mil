#!/usr/bin/env python3
import unittest
from copy import copy, deepcopy
from threading import Lock

import rospy
from mil_tools import thread_lock
from navigator_alarm_handlers import NetworkLoss
from navigator_kill_board import HeartbeatServer
from ros_alarms import AlarmBroadcaster, AlarmListener
from std_msgs.msg import Header
from std_srvs.srv import SetBool

lock = Lock()


class killtest(unittest.TestCase):
    def __init__(self, *args):
        self.hw_kill_alarm = None
        self.network_kill_alarm = None
        self.updated = False
        self.AlarmListener = AlarmListener("hw-kill", self._hw_kill_cb)
        self.NetworkListener = AlarmListener("network-loss", self._network_kill_cb)
        self.AlarmBroadcaster = AlarmBroadcaster("kill")
        super().__init__(*args)
        # self.heartbeat = HeartbeatServer("/network", 1.0)

    @thread_lock(lock)
    def reset_update(self):
        """
        Reset update state to False so we can notice changes to hw-kill
        """
        self.updated = False

    @thread_lock(lock)
    def _hw_kill_cb(self, alarm):
        """
        Called on change in hw-kill alarm.
        If the raised status changed, set update flag to true so test an notice change
        """
        if self.hw_kill_alarm is None or alarm.raised != self.hw_kill_alarm.raised:
            self.updated = True
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
            self.updated = True
        self.network_kill_alarm = alarm
        if alarm.raised:
            rospy.logerr("network kill raised")

    def wait_for_kill_update(self, timeout=rospy.Duration(0.5)):
        """
        Wait up to timeout time to an hw-kill alarm change. Returns a copy of the new alarm or throws if times out
        """
        timeout = rospy.Time.now() + timeout
        while rospy.Time.now() < timeout:
            lock.acquire()
            updated = copy(self.updated)
            alarm = (deepcopy(self.hw_kill_alarm), deepcopy(self.network_kill_alarm))
            lock.release()
            if updated:
                return alarm
            rospy.sleep(0.01)
        raise Exception("timeout")

    def assert_raised(self, timeout=rospy.Duration(0.5)):
        """
        Waits for update and ensures it is now raised
        """
        alarm = self.wait_for_kill_update(timeout)
        self.assertEqual(alarm[0].raised and alarm[1].raised, True)

    def assert_cleared(self, timeout=rospy.Duration(0.5)):
        """
        Wait for update and ensures is now cleared
        """
        alarm = self.wait_for_kill_update(timeout)
        self.assertEqual(alarm[0].raised and alarm[1].raised, False)

    def test_1_initial_state(self):  # test the initial state of kill signal
        """
        Tests initial state of system, which should have hw-kill raised because kill is raised at startup.

        Because hw-kill will be initialized to cleared then later raised when alarm server is fully started,
        so we need to allow for pottentialy two updates before it is raised.
        """
        alarm = self.wait_for_kill_update(
            timeout=rospy.Duration(10.0)
        )  # Allow lots of time for initial alarm setup
        if alarm[0].raised or alarm[1].raised:
            self.assertTrue(True)
            return
        self.reset_update()
        self.assert_raised(timeout=rospy.Duration(10.0))

    def test_2_computer_kill(self):
        """
        Test raising/clearing kill alarm (user kill) will cause same change in hw-kill
        """
        self.reset_update()
        self.AlarmBroadcaster.clear_alarm()
        self.assert_cleared()

        self.reset_update()
        self.AlarmBroadcaster.raise_alarm()
        self.assert_raised()

        self.reset_update()
        self.AlarmBroadcaster.clear_alarm()
        self.assert_cleared()

    def _test_button(self, button):
        """
        Tests that button kills work through simulated service.
        """
        bfp = rospy.ServiceProxy(f"/kill_board_interface/BUTTON_{button}", SetBool)
        bfp.wait_for_service(timeout=5.0)

        self.reset_update()
        bfp(True)  # set the button value to true
        self.assert_raised()

        self.reset_update()
        self.AlarmBroadcaster.clear_alarm()
        bfp(False)
        self.assert_cleared()

    def test_3_buttons(self):
        """
        Tests each of the four buttons
        """
        for button in ["FRONT_PORT", "AFT_PORT", "FRONT_STARBOARD", "AFT_STARBOARD"]:
            self._test_button("FRONT_PORT")

    def test_4_remote(self):
        """
        Tests remote kill by publishing hearbeat, stopping and checking alarm is raised, then
        publishing hearbeat again to ensure alarm gets cleared.
        """
        # self.heartbeat = HeartbeatServer("/network", 0.5)
        # publishing msg to network
        pub = rospy.Publisher("/network", Header, queue_size=10)
        rate = rospy.Rate(10)
        t_end = rospy.Time.now() + rospy.Duration(1)
        while rospy.Time.now() < t_end:
            h = Header()
            h.stamp = rospy.Time.now()
            pub.publish(h)
            rate.sleep()

        self.reset_update()
        rospy.sleep(8.5)  # Wait slightly longer then the timeout on killboard
        self.assert_raised()

        self.reset_update()
        t_end = rospy.Time.now() + rospy.Duration(1)
        while rospy.Time.now() < t_end:
            h = Header()
            h.stamp = rospy.Time.now()
            pub.publish(h)
            rate.sleep()
        self.AlarmBroadcaster.clear_alarm()
        self.assert_cleared()


if __name__ == "__main__":
    rospy.init_node("lll", anonymous=True)
    import rostest

    rostest.rosrun("navigator_kill_board", "kill_board_test", killtest)
    unittest.main()
