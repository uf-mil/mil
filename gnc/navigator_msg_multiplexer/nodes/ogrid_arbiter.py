#!/usr/bin/python
from __future__ import division

import tf
import rospy
import roslib
import navigator_tools
from navigator_tools import fprint as _fprint

from scipy.misc import imresize
import numpy as np
from copy import deepcopy

from dynamic_reconfigure.server import Server
from navigator_msg_multiplexer.cfg import OgridConfig

from rostopic import ROSTopicHz
from nav_msgs.msg import OccupancyGrid, MapMetaData

'''
''  - NaviGator Occupancy Grid Server -
''      Server that will handle merging multiple ROS Occupancy Maps into one map which
''      will then be fed to the motion planner.
''
''      Requirements:
''      1. At launch, all nodes specified in launch file will be merged onto the global map.
''      2. New maps can be added to the server via. the Parameter Server. This is outlined below:
''          - Added via paramter server
''          - Service call to /_____/____ will trigger an update
'''

# Wow what a concept
fprint = lambda *args, **kwargs: _fprint(title="OGRID_ARB", *args, **kwargs)

def make_ogrid_transform(ogrid):
        '''
            Returns a matrix that transforms a point in ENU to this ogrid.
            Invert the result to get ogrid -> ENU.
        '''
        resolution = ogrid.info.resolution
        origin = navigator_tools.pose_to_numpy(ogrid.info.origin)[0]

        # Transforms points from ENU to ogrid frame coordinates
        t = np.array([[1 / resolution,  0, -origin[1] / resolution],
                      [0,  1 / resolution, -origin[0] / resolution],
                      [0,               0,                       1]])
        return t

def numpyify(ogrid):
    # Could be a lambda
    return np.array(ogrid.data).reshape(ogrid.info.height, ogrid.info.width)

def transform_enu_to_ogrid(enu_points, grid):
    '''
        Convert an enu point into the global ogrid's frame.
        `enu_points` should be a list of points in the ENU frame that will be converted
            into the grid frame
    '''
    t = make_ogrid_transform(grid)
    return t.dot(np.array(enu_points).T).T

def transform_ogrid_to_enu(grid_points, grid):
    '''
        Convert an ogrid cell into the enu frame.
        `grid_points` should be a list of points in the ogrid frame that will be converted
            into the ENU frame
    '''
    t = np.linalg.inv(make_ogrid_transform(grid))
    return t.dot(np.array(grid_points).T).T

def transform_between_ogrids(grid1_points, grid1, grid2):
    enu = transform_ogrid_to_enu(grid1_points, grid1)
    return transform_enu_to_ogrid(enu, grid2)

def get_enu_corners(grid):
    '''
        Given an ogrid, this returns the top left (?) and bottom right (?) points in the ENU frame
    '''
    grid_to_enu = np.linalg.inv(make_ogrid_transform(grid))
    _min = grid_to_enu.dot([0, 0, 1])
    _max = grid_to_enu.dot([grid.info.height, grid.info.width, 1])
    return (_min, _max)

class OGrid:
    def __init__(self, topic, frame_id='enu', debug=False):
        # Assert that the topic is valid
        self.topic = topic
        self.frame_id = frame_id

        self.last_message_stamp = None
        self.map_position = None        # We should default to the origin if no map location is given

        self.status = False             # If we are no  longer using this map, status is False
        self.nav_ogrid = None           # Last recieved OccupancyGrid message
        self.np_map = None              # Numpy version of last recieved OccupancyGrid message
        self.debug = debug              # Diagnostic print flag

        self.transform = None

        self.subscriber = rospy.Subscriber(self.topic, OccupancyGrid, self.cb, queue_size=1)

    def __str__(self):
        # TODO: Reformat this line
        return '{}: \nFrame ID: {}\n  Position: {}\n  Callback Delta: {}\n'.format(self.topic,
                                                                                   self.frame_id,
                                                                                   self.map_position,
                                                                                   self.callback_delta)

    @property
    def callback_delta(self):
        if self.last_message_stamp is None:
            return 0
        return (rospy.Time.now() - self.last_message_stamp).to_sec()

    def trim_to_global(self, g_xs, g_ys, global_ogrid):
        start_x, end_x = transform_between_ogrids(g_xs, global_ogrid, self.nav_ogrid)[:2]
        start_y, end_y = transform_between_ogrids(g_ys, global_ogrid, self.nav_ogrid)[:2]
        return self.np_map[start_x:end_x, start_y:end_y]

    def get_enu_corners(self):
        if self.nav_ogrid is None:
            return None
        return get_enu_corners(self.nav_ogrid)

    def convert_ogrid_np(self):
        # Converts nav_msgs/OccupancyGrid to Numpy array
        if self.nav_ogrid is None:
            return None

        # Reject 2 second old ogrids
        if self.callback_delta > 2:
            return None

        return numpyify(self.nav_ogrid)

    def cb(self, ogrid):
        # Fetches the currently available map
        self.last_message_stamp = ogrid.header.stamp
        self.nav_ogrid = ogrid
        self.np_map = self.convert_ogrid_np()

        if self.debug:
            # TODO: Why is this not printing?
            rospy.loginfo('  Topic: %s', self.topic)
            rospy.loginfo('  Last recieved stamp: %s', str(self.last_message_stamp.secs))
            rospy.loginfo('  Callback Delta: %s', str(self.callback_delta))


class OGridServer:
    def __init__(self, topics=None, frame_id='enu', map_size=800, resolution=0.3, rate=1):
        self.resolution = resolution        # Meter/Grid Space
        self.size = (map_size, map_size)    # Size of the grid on which we will place incoming grids (meters)

        self.topics = topics                # List of nav_msgs/topics that we wish to subscribe to
        self.frame_id = frame_id            # Frame that we will be operating from (ENU)
        self.ogrids = {}                    # Dict of maps that have been added to the global map (In case we want to remove maps)

        self.global_ogrid = self.create_grid(self.size, self.resolution)

        self.publisher = rospy.Publisher('/ogrid_master', OccupancyGrid, queue_size=1)

        srv = Server(OgridConfig, self.dynamic_cb)

        rospy.Timer(rospy.Duration(1.0 / rate), self.publish)

        rospy.spin()

    def dynamic_cb(self, config, level):
        topics =  config['topics'].replace(' ', '').split(',')
        new_grids = {}

        for topic in topics:
           new_grids[topic] = OGrid(topic) if topic not in self.ogrids else self.ogrids[topic]

        self.ogrids = new_grids

        # Change size or resolution of global ogrid
        return config

    def create_grid(self, map_size, resolution, origin=None):
        '''
            Creates blank ogrids for everyone for the low low price of $9.95!
            `map_size` should be in the form of (h, w)
            `resolution` should be in m/cell
        '''
        ogrid_ = OccupancyGrid()
        ogrid_.header.stamp = rospy.Time.now()
        ogrid_.header.frame_id = self.frame_id

        ogrid_.info.resolution = resolution
        ogrid_.info.width = map_size[1]
        ogrid_.info.height = map_size[0]

        if origin is None:
            position = np.array([-(map_size[1] * resolution) / 2, -(map_size[0] * resolution) / 2, 0])
            quaternion = np.array([0, 0, 0, 1])
            origin = navigator_tools.numpy_quat_pair_to_pose(position, quaternion)

        ogrid_.info.origin = origin

        # TODO: Make sure this produces the correct sized ogrid
        ogrid_.data = np.full((map_size[1], map_size[0]), -1).flatten()

        fprint('Created Occupancy Map')
        return ogrid_

    def merge_grids(self):
        #rospy.loginfo('Merging Maps')
        global_ogrid = deepcopy(self.global_ogrid)

        np_grid = numpyify(global_ogrid)

        for ogrid in self.ogrids.itervalues():
            if ogrid.nav_ogrid is None or ogrid.callback_delta > 2:
                continue

            # Global ogrid
            corners = get_enu_corners(global_ogrid)
            index_limits = transform_enu_to_ogrid(corners, global_ogrid)[:,:2]

            g_x_min = index_limits[0][0]
            g_x_max = index_limits[1][0]

            g_y_min = index_limits[0][1]
            g_y_max = index_limits[1][1]

            # Local Ogrid (get everything in global frame though)
            corners = ogrid.get_enu_corners()
            index_limits = transform_enu_to_ogrid(corners, ogrid.nav_ogrid)
            index_limits = transform_between_ogrids(index_limits, ogrid.nav_ogrid, global_ogrid)[:,:2]

            l_x_min = index_limits[0][0]
            l_x_max = index_limits[1][0]

            l_y_min = index_limits[0][1]
            l_y_max = index_limits[1][1]

            xs = np.sort([g_x_max, g_x_min, l_x_max, l_x_min])
            ys = np.sort([g_y_max, g_y_min, l_y_max, l_y_min])

            start_x = xs[1]
            end_x = xs[2]

            start_y = ys[1]
            end_y = ys[2]

            fprint("{},{} {},{}".format(start_x, start_y, end_x, end_y))

            # Assuming resolution has been normalized.
            # Getting the indices to slice the local ogrid.
            l_ogrid_start = transform_between_ogrids([start_x, start_y, 1], global_ogrid, ogrid.nav_ogrid)
            index_width = round(l_ogrid_start[0] + end_x - start_x)  # I suspect rounding will be the source of error
            index_height = round(l_ogrid_start[1] + end_y - start_y)

            try:
                to_add = ogrid.np_map[l_ogrid_start[0]:index_width, l_ogrid_start[1]:index_height]
                fprint("to_add shape: {}".format(to_add.shape))
                np_grid[start_x:end_x, start_y:end_y] += to_add
                fprint("np_grid shape: {}".format(np_grid[start_x:end_x, start_y:end_y].shape))
            except Exception as e:
                fprint("Exception caught, probably a dimension mismatch:", msg_color='red')
                print e

            print "=" * 25



        np_grid = np.clip(np_grid, -1, 100)         # Clip values for OccupancyGrid bounds
        global_ogrid.data = np_grid.flatten().astype(np.int8)

        print "=" * 50
        return global_ogrid

    def check_ogrid_topics():
        pass

    def publish(self, *args):
        self.publisher.publish(self.merge_grids())

    # TODO: Make these service calls
    def add_grid(self, ogrid, interpolate=False):
        pass

    def pause_grid(self):
        pass

    def remove_grid(self):
        pass

    def update_global_map(self):
        pass

    def reset_global_grid(self):
        pass

if __name__ == '__main__':
    #if args.topics is None:
    #    rospy.logwarn('No input topics provided')
        # Trigger search for nav_msgs/OccupancyGrid topics currently being published. If there are
        # none, then exit with logerr.
    #    pass
    #elif args.topics is not None:

    rospy.init_node('ogrid_server', anonymous=False)
    og_server = OGridServer()