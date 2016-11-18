#!/usr/bin/env python
import rospy
import actionlib

from geometry_msgs.msg import PoseStamped
from lqrrt_ros.msg import MoveAction, MoveFeedback, MoveResult

import navigator_tools
from navigator_tools import fprint as _fprint


fprint = lambda *args, **kwargs: _fprint(title="FAKE_ACTION_SERVER", time="", *args, **kwargs)

class FakeActionServer(object):
    def __init__(self):
        self.goal_pose_pub = rospy.Publisher("/lqrrt/goal", PoseStamped, queue_size=3)

        self.move_server = actionlib.SimpleActionServer("/move_to", MoveAction, execute_cb=self.move_cb, auto_start=False)
        self.move_server.start()
        rospy.sleep(.1)

        fprint("Fake action server ready!", msg_color="green")

    def move_cb(self, msg):
        fprint("Move request received!", msg_color="blue")
        
        if msg.move_type not in ['hold', 'drive', 'skid', 'circle']:
            fprint("Move type {} not found", msg_color='red')
            self.move_server.set_aborted(MoveResult('move_type'))
        
        p = PoseStamped()
        p.header = navigator_tools.make_header(frame="enu")
        p.pose = msg.goal
        self.goal_pose_pub.publish(p)

        # Sleep before you continue
        rospy.sleep(2)

        fprint("Finished move!", newline=2)
        self.move_server.set_succeeded(MoveResult(''))

if __name__ == "__main__":
    rospy.init_node("fake_action_server")
    f = FakeActionServer()
    rospy.spin()
