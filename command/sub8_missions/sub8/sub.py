#!/usr/bin/env python
import rospy
import numpy as np
import sub8_ros_tools as sub8_utils
from sub8_msgs.msg import Trajectory, Waypoint
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations as tr


class SingletonCheck(object):
    '''TODO: Better way to do this'''
    singleton_used = False


class Sub(object):
    '''Sub singleton object
    Inspired by Forrest's approach

    Absolutely *all* reference frames are FLU (forward-left-up) -> XYZ
        - Where +X, or forward, points out of the "eyes" of the sub
        - and +Y, or left, is the left of the sub if you were standing behind it

    TODO:
        - Request trajectories
        - Be able to block while working on a trajectory
            - (This is why we should track target points in here)
            - Against: it's less clean to track q_d' in the controller if we handle target points here
        - Wait until we have state
        - Don't use truth odom
    '''
    def __init__(self):
        rospy.init_node('sub_mission_manager')
        assert not SingletonCheck.singleton_used, "You can only instantiate one sub!"
        SingletonCheck.singleton_used = True
        self._pose = self._twist = self._pose_covariance = self._twist_covariance = None
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback, queue_size=1)

        # TODO: Handle this better - we should perhaps be publishing one pt at a time
        # TODO: Request trajectory instead of publishing one directly
        self.trajectory_pub = rospy.Publisher('trajectory', Trajectory, queue_size=1)
        rospy.sleep(1)  # Wait a second to gather state

    def goto_position(self, position):
        '''Go to a position
        TODO:
            - Request a trajectory from the trajectory generator
        '''
        assert len(position) == 3, "Must give a world-position in normal cartesian coordinates"
        self.goto_pose(np.eye(3), position)

    def goto_pose(self, R, t):
        '''TODO:
            - Request a trajectory
        '''
        assert R.shape == (3, 3)
        assert len(t) == 3
        pose_msg = sub8_utils.numpy_pair_to_pose(R, t)
        print pose_msg.orientation
        single_pt_traj = [
            Waypoint(pose=pose_msg, twist=Twist())
        ]
        self.trajectory_pub.publish(
            Trajectory(
                header=sub8_utils.make_header('/world'),
                trajectory=single_pt_traj
            )
        )
        print single_pt_traj[0]
        self.block_until_near(t, timeout=30)

    def point_at(self, target_position):
        '''Point at a position in the world'''
        cur_position = np.copy(self.position)
        relative_position = target_position - cur_position

        world_Z = np.array([0.0, 0.0, 1.0])
        pt_along_gnd_plane = sub8_utils.project_pt_to_plane(relative_position, world_Z)

        # Compute the forward-looking direction
        x = sub8_utils.normalize(relative_position)
        # Compute the y, z vectors s.t. the sub does not roll
        y = sub8_utils.normalize(np.cross(relative_position, pt_along_gnd_plane))
        z = sub8_utils.normalize(np.cross(x, y))
        rotation = np.array([
            x,
            y,
            z
        ])
        self.goto_pose(rotation, cur_position)

    def point_at_yaw_only(self, target_position):
        '''Point at a target _only_ yaw, do not pitch'''
        cur_position = np.copy(self.position)
        relative_position = target_position - cur_position
        world_Z = np.array([0.0, 0.0, 1.0])

        x = sub8_utils.normalize(relative_position)
        y = sub8_utils.normalize(np.cross(world_Z, x))

        rotation = np.array([
            x,
            y,
            world_Z
        ]).transpose()

        self.goto_pose(rotation, cur_position)

    def point_at_relative(self, relative_position):
        '''Point at a position given in the current frame of the sub'''
        raise(NotImplementedError)

    def can_i_goto(self, position):
        '''Check if it is possible to go to $position
        TODO:
            - What would this look like?
                - Is this even a capability we should expose?
        '''
        raise(NotImplementedError)

    def do_task(self, task_name):
        '''Hmmm...'''
        raise(NotImplementedError)

    def block_until_pointing(self, orientation_target, angle_error=0.2, timeout=10):
        '''Block until the vehicle publishing odometry is less than $angle_error away from $orientation_target
        TODO: Implement
        '''
        raise(NotImplementedError)
        end_time = rospy.Time.now() + rospy.Duration(timeout)
        while(not rospy.is_shutdown() and (rospy.Time.now() < end_time)):

            # distance = np.linalg.norm(self.position - position)
            # if (distance < min_distance):
                # return True
            rospy.sleep(0.1)
        else:
            return False

    def block_until_near(self, position, velocity=None, min_distance=0.2, timeout=10):
        '''Block until the vehicle publishing odometry is less than $min_distance away from $position
        TODO: Velocity arrival
        '''
        if velocity is None:
            velocity = np.zeros(3)
        end_time = rospy.Time.now() + rospy.Duration(timeout)
        while(not rospy.is_shutdown() and (rospy.Time.now() < end_time)):
            distance = np.linalg.norm(self.position - position)
            if (distance < min_distance):
                return True
            rospy.sleep(0.1)
        else:
            return False

    def odom_callback(self, msg):
        '''Get an odometry message and deserialize it to numpy arrays'''
        odom_unpacked = sub8_utils.odometry_to_numpy(msg)
        self._pose, self._twist, self._pose_covariance, self._twist_covariance = odom_unpacked

    @property
    def position(self):
        assert self._pose is not None, "Attempted to get pose before the vehicle knew its state"
        return self._pose[0]

    @property
    def quaternion(self):
        assert self.pose is not None, "Attempted to get pose before the vehicle knew its state"
        return self.pose[1]

    @property
    def velocity(self):
        '''Twist is in body frame'''
        assert self._twist is not None, "Attempted to get twist before the vehicle knew its state"
        return self._twist[0]

    @property
    def angular_velocity(self):
        '''Avoid doing things that rely on angular velocity in a mission'''
        assert self._twist is not None, "Attempted to get twist before the vehicle knew its state"
        return self._twist[1]


if __name__ == '__main__':
    ss = Sub()
    ss.goto_position(np.array([0.0, 0.0, -5.0]))
    ss.point_at_yaw_only(np.array([-5.0, 0.0, 0.0]))
    rospy.spin()
