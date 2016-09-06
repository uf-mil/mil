#!/usr/bin/env python
import rospy
from std_msgs.msg import String 
from std_srvs.srv import Empty, EmptyRequest
#from sub8_alarm import single_alarm
from kill_handling.broadcaster import KillBroadcaster
from twisted.internet import reactor
import time


class NetworkCheck(object):
    def __init__(self, timeout=5.0, autonomous_msgs_req=50):
        self.timeout = rospy.Duration(timeout)
        self.last_time = rospy.Time.now()
        self.last_msg = ''
        
        # Make sure we don't accidentally let the sub loose.
        # We need auto_msgs_req messages before we go autonomous mode.
        self.auto_msgs_req = autonomous_msgs_req
        self.auto_msg_count = 0

        self.sub = rospy.Subscriber('/keep_alive', String, self.got_network_msg, queue_size=1)
        self.auto_service = rospy.ServiceProxy('/go_auto', Empty)
        self.kb = KillBroadcaster(id='network', description='Network timeout')
        #self.alarm_broadcaster, self.alarm = single_alarm('network-timeout', severity=1)
        rospy.Timer(rospy.Duration(0.1), self.check)

    def check(self, *args):
        if self.need_kill() and self.last_msg != '':
            if self.auto_msg_count >= self.auto_msgs_req:
                rospy.loginfo("AUTONOMOUS MODE STARTED")
                self.auto_service()

                # Kill the sub after the mission
                self.last_msg = 'keep_alive'
                self.auto_msg_count = 0

            #self.alarm.raise_alarm()
            rospy.logerr("KILLING SUB")
            self.kb.send(active=True)
        else:
            self.kb.send(active=False)
            #self.alarm.clear_alarm()

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
        return ((rospy.Time.now() - self.last_time) > self.timeout)

if __name__ == '__main__':
    rospy.init_node('network_kill')
    all_hail_satan = NetworkCheck(timeout=1)
    rospy.spin()
