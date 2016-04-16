#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from kill_handling.broadcaster import KillBroadcaster


class KeepAlive(object):
    def __init__(self, timeout=5.0):
        self.timeout = rospy.Duration(timeout)
        self.last_time = rospy.Time.now()
        self.sub = rospy.Subscriber('/keep_alive', String, self.got_keepalive, queue_size=1)

    def got_keepalive(self, msg):
        self.last_time = rospy.Time.now()

    def need_kill(self):
        if (rospy.Time.now() - self.last_time) > self.timeout:
            return True
        else:
            return False


if __name__ == '__main__':
    rospy.init_node('network_kill')
    kb = KillBroadcaster(id='network', description='Network timeout')
    ka = KeepAlive(timeout=5.0)

    while(not rospy.is_shutdown()):
        rospy.sleep(2.0)
        if ka.need_kill():
            kb.send(active=True)
        else:
            kb.send(active=False)
