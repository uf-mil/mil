#! /usr/bin/env python

import rospy
import actionlib
import robotx_mission_planner.msg
from robotx_msgs.msg import Waypoint
from robotx_tools import geometry_helper as gh
from robotx_tools import msg_helper as mh


class goal_proxy(object):

    '''
        This class is used to send waypoint goals to the motion planner
        It contains several shortcuts to send movements as well as the
        ability to specific location information

    '''

    def __init__(self, testing):

        self.current_pos = Waypoint()
        self.client = actionlib.SimpleActionClient('goal', robotx_mission_planner.msg.current_goalAction)
        self.client.wait_for_server()
        self.testing = testing

    def odom_cb(self, msg):
        ''' Capture lamove_result position '''
        self.current_pos = msg

    def send_goal(self, goal_pos, timeout=10):
        ''' Send a move_result move to the server to complete '''

        goal = robotx_mission_planner.msg.current_goalGoal(waypoint=goal_pos)
        self.client.send_goal(goal)

        # Give the move the specified amount of time to complete
        finished_in_time = False
        finished_in_time = self.client.wait_for_result(rospy.Duration(timeout))

        if finished_in_time:
            rospy.loginfo("Move complete")
            if self.testing:
                return goal_pos
            return True
        else:
            rospy.loginfo("Move not complete")
            if self.testing:
                return goal_pos
            return False

    def move_forward(self, move, timeout=10):
        ''' Move forward in the given amount of time '''
        to_send = Waypoint()
        to_send.pose.position.x = self.current_pos.pose.position.x + move

        move_result = self.send_goal(to_send, timeout)
        return move_result

    def move_back(self, move, timeout=10):
        ''' Move backwards in the given amount of time '''
        to_send = Waypoint()
        to_send.pose.position.x = self.current_pos.pose.position.x - move
        # send goal to server to complete within timeout
        move_result = self.send_goal(to_send, timeout)
        return move_result

    def move_left(self, move, timeout=10):
        ''' Move left in the given amount of time '''
        to_send = Waypoint()
        to_send.pose.position.y = self.current_pos.pose.position.y - move
        # send goal to server to complete within timeout
        move_result = self.send_goal(to_send, timeout)
        return move_result

    def move_right(self, move, timeout=10):
        ''' Move right in the given amount of time '''
        to_send = Waypoint()
        to_send.pose.position.y = self.current_pos.pose.position.y + move
        # send goal to server to complete within timeout
        move_result = self.send_goal(to_send, timeout)
        return move_result

    def move_up(self, move, timeout=10):
        ''' Move up in the given amount of time '''
        to_send = Waypoint()
        to_send.pose.position.z = self.current_pos.pose.position.z + move
        # send goal to server to complete within timeout
        move_result = self.send_goal(to_send, timeout)
        return move_result

    def move_down(self, move, timeout=10):
        ''' Move down in the given amount of time '''
        to_send = Waypoint()
        to_send.pose.position.z = self.current_pos.pose.position.z - move
        # send goal to server to complete within timeout
        move_result = self.send_goal(to_send, timeout)
        return move_result

    def absolute_move(self, x=None, y=None, z=None, qx=None, qy=None, qz=None, qw=None, timeout=10):
        '''
            Move to an absolute /map position
            The user should specify position with
            class_instance.absolute_goal and then call this function
        '''

        if x is None:
            x = self.current_pos.pose.x
        if y is None:
            y = self.current_pos.pose.y
        if z is None:
            z = self.current_pos.pose.z
        if qx is None:
            qx = self.current_pos.pose.orientation.x
        if qy is None:
            qy = self.current_pos.pose.orientation.y
        if qz is None:
            qz = self.current_pos.pose.orientation.z
        if qw is None:
            qw = self.current_pos.pose.orientation.w

        to_send = Waypoint()
        # Set variables from built position
        to_send.pose.position.x = x
        to_send.pose.position.y = y
        to_send.pose.position.z = z
        to_send.pose.orientation.x = qx
        to_send.pose.orientation.y = qy
        to_send.pose.orientation.z = qz
        to_send.pose.orientation.w = qw
        # send goal to server to complete within timeout
        move_result = self.send_goal(to_send, timeout)
        return move_result

    def relative_move(self, x=0, y=0, z=0, tx=0, ty=0, tz=0, timeout=10):
        '''
            Move to a position relative to the current position
            The user should specify position with
            class_instance.relative_goal and then call this function
        '''

        to_send = Waypoint()

        # Add move onto current position
        to_send.pose.position.x = self.current_pos.pose.position.x + x
        to_send.pose.position.y = self.current_pos.pose.position.y + y
        to_send.pose.position.z = self.current_pos.pose.position.z + z

        # This section adds the desired axis rotation to current position
        # convert from quaternion to numpy array
        cur_rotation = gh.quat_to_euler(gh.quat_to_np(self.current_pos.pose.orientation))
        new_rotation = cur_rotation
        new_rotation[0] = cur_rotation[0] + tx
        new_rotation[1] = cur_rotation[1] + ty
        new_rotation[2] = cur_rotation[2] + tz
        # convert from rotvec back into ROS quaternion message
        quat = gh.euler_to_quat(new_rotation)
        # set quaternion to send as the new quaternion
        to_send.pose.orientation = mh.numpy_to_quat(quat)
        # send goal to server to complete within timeout
        move_result = self.send_goal(to_send, timeout)
        return move_result
