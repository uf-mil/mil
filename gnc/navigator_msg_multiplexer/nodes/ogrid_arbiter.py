#!/usr/bin/python
import tf
import rospy
import roslib
import argparse
import numpy as np
import navigator_tools
import message_filters
import matplotlib.image as mpimg

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

        self.subscriber = rospy.Subscriber(self.topic, OccupancyGrid, self.cb, queue_size=1)

    def __repr__(self):
        #return '<%s instance at %s>' % (self.__class__.__name__, if self)
        pass

    def __str__(self):
        # TODO: Reformat this line
        return '{}: \nFrame ID: {}\n  Position: {}\n  Last Message Recieved: {}\n  Status: {}\n'.format(self.topic,
                                                                                                   self.map_position,
                                                                                                   self.last_message_stamp,
                                                                                                   self.status)

    def convert_ogrid_np(self):
        # Converts nav_msgs/OccupancyGrid to Numpy array
        if self.nav_ogrid is None:
            # raise ValueError('No OccupancyMap stored by %i', id(self))
            pass
        else:
            grid_ = np.asarray(self.nav_ogrid.data).reshape(self.nav_ogrid.info.width, self.nav_ogrid.info.height)
        return grid_

    def interpolate_ogrid(self, ogrid_, scale):
        # Handling maps with different scales (???)
        pass

    def callback_delta(self):
        return rospy.get_rostime() - self.last_message_stamp

    def cb(self, ogrid):
        # Fetches the currently available map
        self.last_message_stamp = ogrid.header.stamp
        self.nav_ogrid = ogrid
        self.np_map = self.convert_ogrid_np(ogird)

        if self.debug is False:
            # TODO: Why is this not printing?
            rospy.loginfo('  Topic: %s', self.topic)
            rospy.loginfo('  Last recieved stamp: %s', str(self.last_message_stamp.secs))
            rospy.loginfo('  Callback Delta: %s', str(self.callback_delta))


class OGridServer:
    def __init__(self, topics=None, frame_id='enu', map_size=200, resolution=0.3, rate=30):

        rospy.init_node('ogrid_server', anonymous=False)

        self.resolution = resolution        # Meter/Grid Space
        self.size = (map_size, map_size)    # Size of the grid on which we will place incoming grids

        self.topics = topics                # List of nav_msgs/topics that we wish to subscribe to
        self.rate = rospy.Rate(int(rate))   # Rate at which we will publish the merge ogrid
        self.frame_id = frame_id            # Frame that we will be operating from (ENU)
        self.ogrids = []                    # List of maps that have been added to the global map (In case we want to remove maps)

        # Spool up a Map object for every ogrid that we wish to work with
        self.ogrids = [OGrid(frame_id='enu', topic=topic) for topic in topics]

        self.publisher = rospy.Publisher('/ogrid_server/global_grid', OccupancyGrid, queue_size=1)

        # Diagnostic Information
        rospy.loginfo('Occupancy Map Server Initialized')
        rospy.loginfo('Adding %i maps to server', len(self.topics))

        rospy.Timer(rospy.Duration(0.1), self.cb())

    def __repr__(self):
        return 'Work in progress'

    def create_grid(self, map_size, resolution=self.):
        ogrid_ = OccupancyGrid()
        ogrid_.header.stamp = rospy.Time.now()
        ogrid_.header.frame_id = self.frame_id

        # TODO: Map position!!!!
        ogrid_.info.resolution = resolution
        ogrid_.info.width = map_size
        ogrid_.info.height = map_size

        rospy.loginfo('Created Occupancy Map')
        return ogrid_

    def transform_ogrid(self, ogrid):
        # If the occupancy map's frame id is not the default, transform it
        pass

    def merge_grids(self):
        rospy.loginfo('Merging Maps')
        ogrid_ = OccupancyGrid()
        ogrid_.header = navigator_tools.make_header(frame=self.frame_id)

        # TODO: Move these operations elsewhere, they seem to be slowing everything down
        global_ogrid_ = np.full((671, 671), -1)

        m_ = MapMetaData()
        m_.resolution = 0.3
        m_.width = 671
        m_.height = 671

        position = np.array([-67.5, -111.5, 1.6])
        quaternion = np.array([0, 0, 0, 1])

        m_.origin = navigator_tools.numpy_quat_pair_to_pose(position, quaternion)

        for ogrid in self.ogrids:
            if ogrid.nav_ogrid is None:
                pass
            else:
                o_np = ogrid.convert_ogrid_np()                   # Numpy representation of occupancy grid
                np.add(global_ogrid_, o_np, out=global_ogrid_)    # Add current ogrid to global one

        np.clip(global_ogrid_, -1, 99, out=global_ogrid_)         # Clip values for OccupancyGrid bounds

        ogrid_.info = m_
        ogrid_.data = global_ogrid_.flatten()

        return ogrid_

    def check_ogrid_topics():
        pass

    def cb(self):
        self.publisher.publish(self.merge_grids())
        self.rate.sleep()
        rospy.spin()

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
    parser = argparse.ArgumentParser()
    parser.add_argument('--topics', '-t', nargs = '*', help='Provide topics that the server will subscribe to')
    args = parser.parse_args()

    #if args.topics is None:
    #    rospy.logwarn('No input topics provided')
        # Trigger search for nav_msgs/OccupancyGrid topics currently being published. If there are
        # none, then exit with logerr.
    #    pass
    #elif args.topics is not None:
    og_server = OGridServer(topics=['/ogrid', '/ogrid_batcave'])