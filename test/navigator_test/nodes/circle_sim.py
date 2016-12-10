#!/usr/bin/env python
from __future__ import division

import rospy

import numpy as np
import cv2

import navigator_tools
from navigator_tools import fprint as _fprint
from navigator_msgs.srv import ObjectDBQuery
from nav_msgs.msg import OccupancyGrid, Odometry


fprint = lambda *args, **kwargs: _fprint(time='', title='SIM',*args, **kwargs)

class DoOdom(object):
    """Republish odom  for lqrrt"""
    def __init__(self):
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=2)
        self.odom = None
        self.carrot_sub = rospy.Subscriber("/lqrrt/ref", Odometry, self.set_odom)

    def set_odom(self, msg):
        self.odom = navigator_tools.pose_to_numpy(msg.pose.pose) 
        self.odom_pub.publish(msg)


class Sim(object):
    def __init__(self, bf_size=30, min_t_spacing=10, num_of_buoys=25):
        self.ogrid_pub = rospy.Publisher('/master_ogrid', OccupancyGrid, queue_size=2)
        self.odom = DoOdom()
        fprint("Shaking hands and taking names.")
        rospy.sleep(1)
        
        # Generate some buoys and totems
        buoy_positions = np.random.uniform(bf_size, size=(num_of_buoys, 2))
        self.totem_positions = np.array([[10, 10], [15, 10], [15, 20], [5, 12]])
        
        self.bf_size = bf_size
        self.buoy_size = 1.2  # radius of buoy (m)
        self.totem_size = 1.2  # radius of totem (m)


        # Let's make sure no buoys arae too close to the totems
        _buoy_positions = []
        for b in buoy_positions:
            print np.linalg.norm(self.totem_positions - b)
            if np.linalg.norm(self.totem_positions - b) > min_t_spacing:
                _buoy_positions.append(b)
        self.buoy_positions = np.array(_buoy_positions)
        fprint("Removed {} buoys that were too close to totems".format(len(buoy_positions) - len(_buoy_positions)))
        assert len(self.buoy_positions) > .5 * num_of_buoys, "Not enough buoys remain, try rerunning."
        
        # Some ogrid defs
        self.grid = None
        self.resolution = 0.3
        self.height = bf_size
        self.width = bf_size
        self.origin = navigator_tools.numpy_quat_pair_to_pose([0, 0, 0],
                                                              [0, 0, 0, 1])
        self.transform = self._make_ogrid_transform()
        self.publish_ogrid = lambda: self.ogrid_pub.publish(self.get_message())

        self.draw_buoys()
        self.draw_totems()
        self.publish_ogrid()

    def _make_ogrid_transform(self):
        self.grid = np.zeros((self.height / self.resolution, self.width / self.resolution))
        # Transforms points from ENU to ogrid frame coordinates
        self.t = np.array([[1 / self.resolution, 0, -self.origin.position.x / self.resolution],
                           [0, 1 / self.resolution, -self.origin.position.y / self.resolution],
                           [0,               0,            1]])
        
        return lambda point: self.t.dot(np.append(point[:2], 1))[:2]

    def get_message(self):
        if self.grid is None:
            fprint("Ogrid was requested but no ogrid was found. Using blank.", msg_color='yellow')
            self.grid = np.zeros((self.height / self.resolution, self.width / self.resolution))

        ogrid = OccupancyGrid()
        ogrid.header = navigator_tools.make_header(frame="enu")
        ogrid.info.resolution = self.resolution
        ogrid.info.height, ogrid.info.width = self.grid.shape
        ogrid.info.origin = self.origin
        grid = np.copy(self.grid)
        ogrid.data = np.clip(grid.flatten() - 1, -100, 100).astype(np.int8)

        return ogrid

    def draw_buoys(self):
        for b in self.buoy_positions:
            center = tuple(self.transform(b).astype(np.int32).tolist())
            cv2.circle(self.grid, center, int(self.buoy_size * self.resolution), 255, -1)

    def draw_totems(self):
        for b in self.totem_positions:
            print b
            center = tuple(self.transform(b).astype(np.int32).tolist())
            print center
            cv2.circle(self.grid, center, int(self.totem_size * self.resolution), 255, -1)
            cv2.circle(self.grid, center, int(.5 * self.totem_size * self.resolution), -5, -1)


if __name__ == "__main__":
    fprint("Starting", msg_color='blue')
    rospy.init_node("Sim")

    Sim()
    rospy.spin()
