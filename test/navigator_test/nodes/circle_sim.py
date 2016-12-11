#!/usr/bin/env python
from __future__ import division

import rospy

from random import shuffle
import numpy as np
import cv2

import navigator_tools
import tf.transformations as trns
from navigator_tools import fprint as _fprint
from navigator_msgs.srv import ObjectDBQuery
from navigator_msgs.msg import PerceptionObject
from nav_msgs.msg import OccupancyGrid, Odometry


fprint = lambda *args, **kwargs: _fprint(time='', title='SIM',*args, **kwargs)

class DoOdom(object):
    """Republish odom  for lqrrt"""
    def __init__(self, rand_size):
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=2)
        self.odom = None
        self.carrot_sub = rospy.Subscriber("/lqrrt/ref", Odometry, self.set_odom)
        
        fprint("Shaking hands and taking names.")
        rospy.sleep(1)

        # We need to publish an inital odom message for lqrrt
        start_ori = trns.quaternion_from_euler(0, 0, np.random.normal() * 3.14)
        start_pos = np.append(np.random.uniform(rand_size, size=(2)), 1)
        start_pose = navigator_tools.numpy_quat_pair_to_pose(start_pos, start_ori)
        start_odom = Odometry()
        start_odom.header = navigator_tools.make_header(frame='enu')
        start_odom.child_frame_id = 'base_link'
        start_odom.pose.pose = start_pose
        self.odom_pub.publish(start_odom)

    def set_odom(self, msg):
        self.odom = navigator_tools.pose_to_numpy(msg.pose.pose) 
        self.odom_pub.publish(msg)


class Sim(object):
    def __init__(self, bf_size=60, min_t_spacing=10, num_of_buoys=15):
        self.ogrid_pub = rospy.Publisher('/master_ogrid', OccupancyGrid, queue_size=2)
        self.odom = DoOdom(bf_size)
        
        # Generate some buoys and totems
        buoy_positions = np.random.uniform(bf_size, size=(num_of_buoys, 2))
        self.totem_positions = np.array([[47, 41], [45, 15], [25, 40], [5, 12]])
        self.colors = [[0, 1, 0], [1, 1, 0], [0, 0, 1], [1, 0, 0]]

        self.bf_size = bf_size
        self.buoy_size = 1  # radius of buoy (m)
        self.totem_size = 1  # radius of totem (m)

        # Let's make sure no buoys arae too close to the totems
        _buoy_positions = []
        for b in buoy_positions:
            if np.all(np.linalg.norm(self.totem_positions - b, axis=1) > min_t_spacing):
                _buoy_positions.append(b)
        self.buoy_positions = np.array(_buoy_positions)
        print len(self.buoy_positions)
        fprint("Removed {} buoys that were too close to totems".format(len(buoy_positions) - len(_buoy_positions)))
        assert len(self.buoy_positions) > .5 * num_of_buoys, "Not enough buoys remain, try rerunning."
        
        # Some ogrid defs
        self.grid = None
        self.resolution = 0.3
        self.height = bf_size * 3 
        self.width = bf_size * 3
        self.origin = navigator_tools.numpy_quat_pair_to_pose([-bf_size, -bf_size, 0],
                                                              [0, 0, 0, 1])
        self.transform = self._make_ogrid_transform()
        self.publish_ogrid = lambda *args: self.ogrid_pub.publish(self.get_message())

        self.draw_buoys()
        self.draw_totems()
        self.publish_ogrid()

        # Now set up the database request service
        rospy.Service("/database/requests", ObjectDBQuery, self.got_request)

        rospy.Timer(rospy.Duration(1), self.publish_ogrid)

    def _make_ogrid_transform(self):
        self.grid = np.zeros((self.height / self.resolution, self.width / self.resolution))
        # Transforms points from ENU to ogrid frame coordinates
        self.t = np.array([[1 / self.resolution, 0, -self.origin.position.x / self.resolution],
                           [0, 1 / self.resolution, -self.origin.position.y / self.resolution],
                           [0,               0,            1]])
        
        return lambda point: self.t.dot(np.append(point[:2], 1))[:2]

    def position_to_object(self, position, color, name="totem"):
        obj = PerceptionObject()
        obj.header = navigator_tools.make_header(frame="enu")
        obj.name = name
        obj.position = navigator_tools.numpy_to_point(position)
        obj.color.r = color[0]
        obj.color.g = color[1]
        obj.color.b = color[2]
        obj.color.a = 1

        return obj

    def got_request(self, req):
        fprint("Request recieved {}".format(req.name))
        if req.name == "totem":
            objects = [self.position_to_object(p, c) for p, c in zip(self.totem_positions, self.colors)]
        elif req.name == "BuoyField":
            objects = [self.position_to_object([self.bf_size / 2, self.bf_size / 2, 0 ], [0, 0, 0], "BuoyField")]
        else:
            return {'found': False}

        return {'objects': objects, 'found':True}

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
            cv2.circle(self.grid, center, int(self.buoy_size / self.resolution), 255, -1)

    def draw_totems(self):
        for b in self.totem_positions:
            center = tuple(self.transform(b).astype(np.int32).tolist())
            cv2.circle(self.grid, center, int(self.totem_size / self.resolution), 255, -1)
            cv2.circle(self.grid, center, int(.5 * self.totem_size / self.resolution), -50, -1)


if __name__ == "__main__":
    fprint("Starting", msg_color='blue')
    rospy.init_node("Sim")

    Sim()
    rospy.spin()
