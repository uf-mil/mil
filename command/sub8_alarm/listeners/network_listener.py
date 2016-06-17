#!/usr/bin/env python
import rospy
from std_msgs.msg import String 
from std_srvs.srv import Empty, EmptyRequest
from sub8_alarm import single_alarm
from twisted.internet import reactor
from missions import auto_mission


class NetworkCheck(object):
    def __init__(self, timeout=5.0, autonomous_msgs_req=100):
        reactor.callWhenRunning(auto_mission.main)
        self.timeout = rospy.Duration(timeout)
        self.last_time = rospy.Time.now()
        self.last_msg = ''
        
        # Make sure we don't accidentally let the sub loose.
        # We need auto_msgs_req messages before we go autonomous mode.
        self.auto_msgs_req = autonomous_msgs_req
        self.auto_msg_count = 0

        self.sub = rospy.Subscriber('/keep_alive', String, self.got_network_msg, queue_size=1)
        self.auto_service = rospy.ServiceProxy('/go_auto', Empty)
        self.alarm_broadcaster, self.alarm = single_alarm('network-timeout', severity=1)
        rospy.Timer(rospy.Duration(0.01), self.check)

    def check(self, *args):
        if self.need_kill() and self.last_msg != '':
            if self.auto_msg_count >= self.auto_msgs_req:
                rospy.loginfo("AUTONOMOUS MODE STARTED")
                self.auto_service()

                # Kill the sub after the mission
                rospy.logerr("KILLING SUB")
                self.alarm.raise_alarm()
                self.last_msg = ''
            else:
                rospy.logerr("KILLING SUB")
                self.alarm.raise_alarm()
        else:
            self.alarm.clear_alarm()

    def got_network_msg(self, msg):
        self.last_msg = msg.data
        self.last_time = rospy.Time.now()

        if msg.data == 'auto':
            if self.auto_msg_count == self.auto_msgs_req:
                rospy.loginfo("AUTONOMOUS MODE ARMED")

            self.auto_msg_count += 1
        else:
            self.auto_msg_count = 0

    def need_kill(self):
        if (rospy.Time.now() - self.last_time) > self.timeout:
            return True
        else:
            return False


if __name__ == '__main__':
    rospy.init_node('network_kill')
    four_score_and_seven_years_ago = NetworkCheck(timeout=0.1)
    rospy.spin()
