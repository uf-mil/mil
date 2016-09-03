import numpy as np
from tf import transformations
import geometry_msgs.msg as geometry_msgs
import std_msgs.msg as std_msgs
import nav_msgs.msg as nav_msgs
import rospy

def rosmsg_to_numpy(rosmsg, keys=None):
    '''Convert an arbitrary ROS msg to a numpy array
    With no additional arguments, it will by default handle:
        Point2D, Point3D, Vector3D, and quaternions

    Ex:
        quat = Quaternion(1.0, 0.0, 0.0, 0.0)
        quat is not a vector, you have quat.x, quat.y,... and you can't do math on that

        But wait, there's hope!
            rosmsg_to_numpy(quat) -> array([1.0, 0.0, 0.0, 0.0])

        Yielding a vector, which you can do math on!

        Further, imagine a bounding box message, BB, with properties BB.x, BB.h, BB.y, and BB.w

            rosmsg_to_numpy(BB, ['x', 'h', 'y', 'w']) -> array([BB.x, BB.h, BB.y, BB.w])
            or...
            rosmsg_to_numpy(some_Pose2D, ['x', 'y', 'yaw']) = array([x, y, yaw])

    Note:
        - This function is designed to handle the most common use cases (vectors, points and quaternions)
            without requiring any additional arguments.
    '''
    if keys is None:
        keys = ['x', 'y', 'z', 'w']
        output_array = []
        for key in keys:
            # This is not necessarily the fastest way to do this
            if hasattr(rosmsg, key):
                output_array.append(getattr(rosmsg, key))
            else:
                break

        return np.array(output_array).astype(np.float32)

    else:
        output_array = np.zeros(len(keys), np.float32)
        for n, key in enumerate(keys):
            output_array[n] = getattr(rosmsg, key)

        return output_array

point_to_numpy = rosmsg_to_numpy
quaternion_to_numpy = rosmsg_to_numpy

def pose_to_numpy(pose):
    '''TODO: Unit-test
    returns (position, orientation)
    '''
    position = rosmsg_to_numpy(pose.position)
    orientation = rosmsg_to_numpy(pose.orientation)
    return position, orientation


def twist_to_numpy(twist):
    '''TODO: Unit-test
    Convert a twist message into a tuple of numpy arrays
    returns (linear, angular)
    '''
    linear = rosmsg_to_numpy(twist.linear)
    angular = rosmsg_to_numpy(twist.angular)
    return linear, angular


def posetwist_to_numpy(posetwist):
    pose = pose_to_numpy(posetwist.pose)
    twist = twist_to_numpy(posetwist.twist)
    return pose, twist


def odometry_to_numpy(odom):
    '''TODO: Unit-test
    Convert an odometry message into a tuple of numpy arrays
    returns (pose, twist, pose_covariance, twist_covariance)
    '''
    pose = pose_to_numpy(odom.pose.pose)
    pose_covariance = np.array(odom.pose.covariance).reshape(6, 6)

    twist = twist_to_numpy(odom.twist.twist)
    twist_covariance = np.array(odom.twist.covariance).reshape(6, 6)

    return pose, twist, pose_covariance, twist_covariance


def wrench_to_numpy(wrench):
    force = rosmsg_to_numpy(wrench.force)
    torque = rosmsg_to_numpy(wrench.torque)
    return force, torque


def numpy_to_point(np_vector):
    return geometry_msgs.Point(*np_vector)


def numpy_to_quaternion(np_quaternion):
    return geometry_msgs.Quaternion(*np_quaternion)


def numpy_to_twist(linear_vel, angular_vel):
    return geometry_msgs.Twist(linear=geometry_msgs.Vector3(*linear_vel), angular=geometry_msgs.Vector3(*angular_vel))


def numpy_to_wrench(forcetorque):
    return geometry_msgs.Wrench(
        force=geometry_msgs.Vector3(*forcetorque[:3]),
        torque=geometry_msgs.Vector3(*forcetorque[3:])
    )


def numpy_matrix_to_quaternion(np_matrix):
    '''Given a 3x3 rotation matrix, return its geometry_msgs Quaternion'''
    assert np_matrix.shape == (3, 3), "Must submit 3x3 rotation matrix"
    pose_mat = np.eye(4)
    pose_mat[:3, :3] = np_matrix
    np_quaternion = transformations.quaternion_from_matrix(pose_mat)
    return geometry_msgs.Quaternion(*np_quaternion)


def numpy_pair_to_pose(np_translation, np_rotation_matrix):
    '''Convert a R, t pair to a geometry_msgs Pose'''
    orientation = numpy_matrix_to_quaternion(np_rotation_matrix)
    position = numpy_to_point(np_translation)
    return geometry_msgs.Pose(position=position, orientation=orientation)


def numpy_quat_pair_to_pose(np_translation, np_quaternion):
    orientation = numpy_to_quaternion(np_quaternion)
    position = numpy_to_point(np_translation)
    return geometry_msgs.Pose(position=position, orientation=orientation)


def make_header(frame='/body', stamp=None):
    if stamp is None:
        try:
            stamp = rospy.Time.now()
        except rospy.ROSInitException:
            stamp = rospy.Time(0)

    header = std_msgs.Header(
        stamp=stamp,
        frame_id=frame
    )
    return header


def make_wrench_stamped(force, torque, frame='/body'):
    '''Make a WrenchStamped message without all the fuss
        Frame defaults to body
    '''
    wrench = geometry_msgs.WrenchStamped(
        header=make_header(frame),
        wrench=geometry_msgs.Wrench(
            force=geometry_msgs.Vector3(*force),
            torque=geometry_msgs.Vector3(*torque)
        )
    )
    return wrench


def make_pose_stamped(position, orientation, frame='/body'):
    wrench = geometry_msgs.WrenchStamped(
        header=make_header(frame),
        pose=geometry_msgs.Pose(
            force=geometry_msgs.Vector3(*position),
            orientation=geometry_msgs.Quaternion(*orientation)
        )
    )
    return wrench


def odom_sub(topic, callback):
    def wrapped_callback(*args):
        msg = args[-1]
        callback(odometry_to_numpy(msg))

    return rospy.Subscriber(topic, nav_msgs.Odometry, wrapped_callback, queue_size=1)


def ros_to_np_3D(msg):
    xyz_array = np.array(([msg.x, msg.y, msg.z]))
    return xyz_array
