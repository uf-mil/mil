#!/usr/bin/env python
from __future__ import division

import rospy
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion, Pose2D
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sub8_msgs.srv import VisionRequest2D, VisionRequest2DResponse, SearchPose
from image_geometry import PinholeCameraModel
from sub8_ros_tools import threading_helpers, msg_helpers
from std_srvs.srv import Empty, EmptyResponse
import cv2
import numpy as np
import threading


def unit_vector(vect):
    return vect / np.linalg.norm(vect)


def make_2D_rotation(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s],
                     [s, c]], dtype=np.float32)


class OccGridUtils(object):
    '''
    Contains functions for dealing with occupancy grids as well as storing and publishing them.

    All distance measured in meters.
    '''
    def __init__(self, res, width, height, starting_pose, topic_name='/search_grid'):
        self.meta_data = MapMetaData()
        # Resolution is m/cell. Width is X, height is Y.
        self.meta_data.resolution = res  # rospy.get_param("map_resolution")
        self.meta_data.width = width  # rospy.get_param("map_width")
        self.meta_data.height = height  # rospy.get_param("map_height")

        # Starting Position
        self.mid_x = -starting_pose.x
        self.mid_y = -starting_pose.y

        self.meta_data.origin = Pose(position=Point(x=-starting_pose.x * res, y=-starting_pose.y * res, z=0),
                                     orientation=Quaternion(*tf.transformations.quaternion_from_euler(0, 0, starting_pose.theta)))

        s = rospy.Service('/reset_occ_grid', Empty, self.reset_grid)

        # Create array of -1's of the correct size
        self.occ_grid = np.zeros((self.meta_data.height, self.meta_data.width)) - 1
        self.searched = np.zeros((self.meta_data.height, self.meta_data.width))
        self.markers = np.zeros((self.meta_data.height, self.meta_data.width))

        self.occ_grid_pub = rospy.Publisher(topic_name, OccupancyGrid, queue_size=1)

        self.searcher = Searcher(self.searched, res, [self.mid_x, self.mid_y])

    def add_circle(self, center, radius):
        '''
        Adds a circle to the grid.
        Also used to project the camera's view onto the grid becuase it is rotationally intolerant.
        '''
        center_offset = np.array(center) / self.meta_data.resolution - np.array([self.mid_x, self.mid_y])

        try:
            radius = int(radius / self.meta_data.resolution)
        except:
            radius = 0

        cv2.circle(self.searched, tuple(center_offset.astype(np.int32)), radius, 1, -1)

    def found_marker(self, pose_2d):
        '''
        Used to mark found markers.
        It doesn't matter that this isn't perfect, we just need to know that something is there.
        '''
        TRENCH_LENGTH = 1.22 / self.meta_data.resolution  # cells (4 ft)
        TRENCH_WIDTH = .1524 / self.meta_data.resolution  # cells (6 inches)

        center = np.array([pose_2d.x, pose_2d.y])  # The negative all depends on how the center is returned
        rotation = -pose_2d.theta

        center_offset = center / self.meta_data.resolution - np.array([self.mid_x, self.mid_y])

        rot_top_point = np.dot(np.array([TRENCH_LENGTH, 0]) / 2, make_2D_rotation(rotation)).astype(np.int32)
        rot_bottom_point = np.dot(-np.array([TRENCH_LENGTH, 0]) / 2, make_2D_rotation(rotation)).astype(np.int32)

        pos_top_point = np.int0(rot_top_point + center_offset)
        pos_bottom_point = np.int0(rot_bottom_point + center_offset)

        cv2.line(self.markers, tuple(pos_top_point), tuple(pos_bottom_point), 101, int(TRENCH_WIDTH))

    def publish_grid(self):
        '''
        Take the occupancy grid and send it out over ros with timestamps and whatnot.
        '''
        t = rospy.Time.now()
        header = Header(stamp=t, frame_id='/map')
        # Populate occ grid msg
        occ_msg = OccupancyGrid()
        occ_msg.header = header
        occ_msg.info = self.meta_data
        # Make sure values don't go out of range
        occ_grid = self.searched + self.markers - 1
        occ_msg.data = np.clip(occ_grid.flatten(), -1, 100)
        self.occ_grid_pub.publish(occ_msg)

    def reset_grid(self, srv):
        '''
        Resets occupancy grid. I'm using a random service since I don't really care about getting or returning information here.
        '''
        # Create array of -1's of the correct size
        print "Resetting Grid."
        self.occ_grid = np.zeros((self.meta_data.height, self.meta_data.width)) - 1
        self.searched = np.zeros((self.meta_data.height, self.meta_data.width))
        self.markers = np.zeros((self.meta_data.height, self.meta_data.width))
        return EmptyResponse()


class Searcher():
    '''
    Intented to provide a service that will return a pose to go to in order to search for a missing marker.
    Not sure how this will be implemented in its entirety.
    '''
    def __init__(self, searched_area, grid_res, position_offset):
        self.searched_area = searched_area
        self.grid_res = grid_res
        self.position_offset = position_offset
        self.poly_generator = self.polygon_generator()
        self.max_searches = 12
        self.current_search = 0

        s = rospy.Service('/next_search_pose', SearchPose, self.return_pose)

    def return_pose(self, srv):
        '''
        Returns the next pose to go to in order to try and find the marker.
        Only searches in a circle around our current location (could be extended to searching farther if we need to).
        This will skip points that have already been searched.
        '''
        if srv.reset_search:
            self.current_search = 0
            self.poly_generator = self.polygon_generator()
            return [0, False, srv.intial_position]

        # We search at 1.5 * r so that there is some overlay in the search feilds.
        np_pose = msg_helpers.pose_to_numpy(srv.intial_position)
        rot_mat = make_2D_rotation(tf.transformations.euler_from_quaternion(np_pose[1])[2])
        coor = np.append(np.dot(rot_mat, next(self.poly_generator)) * srv.search_radius * 1.75, 0) \
            + np_pose[0]

        # if self.current_search > self.max_searches:
        #     rospy.logwarn("Searched {} times. Marker not found.".format(self.max_searches))
        #     return [0, False, srv.intial_position]

        self.current_search += 1
        rospy.loginfo("Search number: {}".format(self.current_search))
        # Should be variable based on height maybe?
        pose = msg_helpers.numpy_quat_pair_to_pose(coor, np.array([0, 0, 0, 1]))
        return [self.check_searched(coor[:2], srv.search_radius), True, pose]

    def check_searched(self, search_center, search_radius):
        '''
        Mask out a circle of the searched area around the point, then find the area of the grid in that mask. If
        the area is above some threshold assume we have searched this area and do not need to serach it again.
        '''
        center_offset = np.array(search_center) / self.grid_res - self.position_offset

        # Create mask
        circle_mask = np.zeros(self.searched_area.shape)
        radius = int(search_radius / self.grid_res)
        cv2.circle(circle_mask, tuple(center_offset.astype(np.int32)), radius, 1, -1)
        masked_search = self.searched_area * circle_mask

        # If there has already been a marker found on the grid in our search area go there.
        if np.max(masked_search) > 5:
            return 0

        # If the center is off of the grid, skip that spot.
        if len(circle_mask[circle_mask > .5]) / (np.pi * radius ** 2) < .3:
            return 1

        return len(masked_search[masked_search > .5]) / len(circle_mask[circle_mask > .5])

    def polygon_generator(self, n=12):
        '''
        Using a generator here to allow for easy expansion in the future.

        TODO: Add difference search patterns.
        '''
        theta = np.radians(360 / n)
        rot_mat = make_2D_rotation(theta)

        # Start by going directly right
        point = np.array([0, -1])

        yield point

        count = 0
        while True:
            point = np.dot(rot_mat, point)

            if count == n - 1:
                point[1] -= 1
                count = 0
            else:
                count += 1

            yield point


class MarkerOccGrid(OccGridUtils):
    '''
    Handles updating occupancy grid when new data comes in.
    TODO: Upon call can return some path to go to in order to find them.
    '''
    def __init__(self, image_sub, grid_res, grid_width, grid_height, grid_starting_pose):
        super(self.__class__, self).__init__(res=grid_res, width=grid_width, height=grid_height, starting_pose=grid_starting_pose)

        self.tf_listener = tf.TransformListener()

        self.cam = PinholeCameraModel()
        self.camera_info = image_sub.wait_for_camera_info()
        if self.camera_info is None:
            # Maybe raise an alarm here.
            rospy.logerr("I don't know what to do without my camera info.")

        self.cam.fromCameraInfo(self.camera_info)

    def update_grid(self, timestamp):
        '''
        Takes marker information to update occupacy grid.
        '''
        x_y_position, height = self.get_tf(timestamp)

        self.add_circle(x_y_position, self.calculate_visual_radius(height))
        self.publish_grid()

    def add_marker(self, marker, timestamp):
        '''
        Find the actual 3d pose of the marker and fill in the occupancy grid for that pose.
        This works by:
            1. Calculate unit vector between marker point and the image center in the image frame.
            2. Use height measurement to find real life distance (m) between center point and marker center.
            3. Use unit vec and magnitude to find dx and dy in meters.
            3. Pass info to OccGridUtils.
        '''
        if marker[0] is None:
            return

        x_y_position, height = self.get_tf(timestamp)

        if marker[2] < self.calculate_marker_area(height) * .6:
            # If the detected region isn't big enough dont add it.
            rospy.logwarn("Marker found but it is too small, not adding marker.")
            return

        # Calculate position of marker accounting for camera rotation.
        dir_vector = unit_vector(np.array([self.cam.cx(), self.cam.cy()] - marker[0]))
        trans, rot = self.tf_listener.lookupTransform("/map", "/downward", timestamp)
        cam_rotation = tf.transformations.euler_from_quaternion(rot)[2] + np.pi / 2
        dir_vector = np.dot(dir_vector, make_2D_rotation(cam_rotation))
        marker_rotation = cam_rotation + marker[1]

        trans, rot = self.tf_listener.lookupTransform("/map", "/base_link", timestamp)
        if (np.abs(np.array(rot)[:2]) > .005).any():
            rospy.logwarn("We're at a weird angle, not adding marker.")

        magnitude = self.calculate_visual_radius(height, second_point=marker[0])
        local_position = dir_vector[::-1] * magnitude
        position = local_position + x_y_position

        #print local_position

        # Pose on ground plane from center
        pose = Pose2D(x=position[0], y=position[1], theta=marker_rotation)
        self.found_marker(pose)

        # The image coordinate pose and real life pose are different.
        pose = Pose2D(x=position[0], y=position[1], theta=marker_rotation)
        # In meters with initial point at (0,0)
        return pose

    def get_tf(self, timestamp=None):
        '''
        x_y position and height in meters
        '''
        if timestamp is None:
            timestamp = rospy.Time()

        self.tf_listener.waitForTransform("/map", "/downward", timestamp, rospy.Duration(5.0))
        trans, rot = self.tf_listener.lookupTransform("/map", "/downward", timestamp)
        x_y_position = trans[:2]
        self.tf_listener.waitForTransform("/ground", "/downward", timestamp, rospy.Duration(5.0))
        trans, _ = self.tf_listener.lookupTransform("/ground", "/downward", timestamp)
        height = np.nan_to_num(trans[2])

        return x_y_position, height

    def correct_height(self, measured_height, timestamp):
        '''
        Adjust the measured height from the seafloor using our orientation relative to it.
        We assume the floor is flat (should be valid for transdec but maybe not for pool).
        All the math is just solving triangles.

        Note: If the roll or pitch is too far off, the range could be to a point that is not
            planar to the floor directly under us - in which case this will fail.

        TODO: See if this actually can produce better results.
        '''
        trans, rot = self.tf_listener.lookupTransform("/map", "/base_link", timestamp)
        euler_rotation = tf.transformations.euler_from_quaternion(rot)

        roll_offset = np.abs(np.sin(euler_rotation[0]) * measured_height)
        pitch_offset = np.abs(np.sin(euler_rotation[1]) * measured_height)

        height = np.sqrt(measured_height ** 2 - (roll_offset ** 2 + pitch_offset ** 2))
        return height

    def calculate_marker_area(self, height):
        '''
        Esitmate what the area of the marker should be so we don't add incorrect markers to the occupancy grid.
        What we really don't want is to add markers to the grid that are on the edge since the direction and center of
            the marker are off.
        '''
        MARKER_LENGTH = 1.22  # m
        MARKER_WIDTH = .1524  # m

        # Get m/px on the ground floor.
        m = self.calculate_visual_radius(height)
        if self.cam.cy() < self.cam.cx():
            px = self.cam.cy()
        else:
            px = self.cam.cx()

        m_px = m / px
        marker_area_m = MARKER_WIDTH * MARKER_LENGTH
        marker_area_px = marker_area_m / (m_px ** 2)

        return marker_area_px

    def calculate_visual_radius(self, height, second_point=None):
        '''
        Draws rays to find the radius of the FOV of the camera in meters.
        It also can work to find the distance between two planar points some distance from the camera.
        '''

        mid_ray = np.array([0, 0, 1])

        if second_point is None:
            if self.cam.cy() < self.cam.cx():
                second_point = np.array([self.cam.cx(), 0])
            else:
                second_point = np.array([0, self.cam.cy()])

        edge_ray = unit_vector(self.cam.projectPixelTo3dRay(second_point))

        # Calculate angle between vectors and use that to find r
        theta = np.arccos(np.dot(mid_ray, edge_ray))
        return np.tan(theta) * height

if __name__ == "__main__":
    rospy.init_node('searcher')
    tr = MarkerOccGrid(res=.1, width=100, height=500, starting_pose=Pose2D(x=50, y=50, theta=0))
    rospy.spin()
