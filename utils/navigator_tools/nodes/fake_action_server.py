#!/usr/bin/env python
from __future__ import division

import numpy as np

import rospy
import actionlib
import tf.transformations as trns

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from lqrrt_ros.msg import MoveAction, MoveFeedback, MoveResult

import navigator_tools
from navigator_tools import fprint as _fprint
from behaviors import params


fprint = lambda *args, **kwargs: _fprint(title="FAKE_ACTION_SERVER", time="", *args, **kwargs)

class FakeActionServer(object):
    def __init__(self):
        self.goal_pose_pub = rospy.Publisher("/lqrrt/goal", PoseStamped, queue_size=3)
        
        # Parameters to simulate lqrrt
        self.blind = False
        
        self.ogrid = None
        set_ogrid = lambda msg: setattr(self, "ogrid", msg)
        rospy.Subscriber("/ogrid_master", OccupancyGrid, set_ogrid)

        self.move_server = actionlib.SimpleActionServer("/move_to", MoveAction, execute_cb=self.move_cb, auto_start=False)
        self.move_server.start()
        rospy.sleep(.1)

        fprint("Fake action server ready!", msg_color="green")

    def move_cb(self, msg):
        fprint("Move request received!", msg_color="blue")
        
        if msg.move_type not in ['hold', 'drive', 'skid', 'circle']:
            fprint("Move type '{}' not found".format(msg.move_type), msg_color='red')
            self.move_server.set_aborted(MoveResult('move_type'))
            return
        
        self.blind = msg.blind

        p = PoseStamped()
        p.header = navigator_tools.make_header(frame="enu")
        p.pose = msg.goal
        self.goal_pose_pub.publish(p)

        # Sleep before you continue
        rospy.sleep(1)

        yaw = trns.euler_from_quaternion(navigator_tools.rosmsg_to_numpy(msg.goal.orientation))[2]        
        if not self.is_feasible(np.array([msg.goal.position.x, msg.goal.position.y, yaw]), np.zeros(3)):
            fprint("Not feasible", msg_color='red')
            self.move_server.set_aborted(MoveResult('occupied'))
            return

        fprint("Finished move!", newline=2)
        self.move_server.set_succeeded(MoveResult(''))

    def is_feasible(self, x, u):
        """
        Given a state x and effort u, returns a bool
        that is only True if that (x, u) is feasible.

        """
        # If there's no ogrid yet or it's a blind move, anywhere is valid
        if self.ogrid is None or self.blind:
            return True 

        # Body to world
        c, s = np.cos(x[2]), np.sin(x[2])
        R = np.array([[c, -s], 
        [s,  c]]) 

        # Vehicle points in world frame
        points = x[:2] + R.dot(params.vps).T

        # Check for collision
        cpm = 1 / self.ogrid.info.resolution
        origin = navigator_tools.pose_to_numpy(self.ogrid.info.origin)[0][:2]
        indicies = (cpm * (points - origin)).astype(np.int64)
        
        try:
            data = np.array(self.ogrid.data).reshape(self.ogrid.info.width, self.ogrid.info.height)
            grid_values = data[indicies[:, 1], indicies[:, 0]]
        except IndexError:
            return False

        # Greater than threshold is a hit
        return np.all(grid_values < 90)


if __name__ == "__main__":
    rospy.init_node("fake_action_server")
    f = FakeActionServer()
    rospy.spin()
