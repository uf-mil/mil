import numpy as np


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