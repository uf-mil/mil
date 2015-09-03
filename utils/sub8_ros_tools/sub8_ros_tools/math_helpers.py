import numpy as np


def rosmsg_to_numpy(rosmsg, keys=None):
    '''Convert a ROS Vector or Quaternion to a numpy vector
    Ex:
    quat = Quaternion(1.0, 0.0, 0.0, 0.0)
    quat is not a vector, you have quat.x, quat.y,... and you can't do math on that

    But wait, there's hope!
        rosmsg_to_numpy(quat) -> array([1.0, 0.0, 0.0, 0.0])
    Yielding a vector, which you can do math on!

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

    return np.array(output_array)
