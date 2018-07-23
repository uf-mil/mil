#!/usr/bin/env python

import rospy
from hydrophones.msg import ProcessedPing
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa
import numpy as np

vectors_3d = None

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


def callback(ping):
    global vectors_3d

    vec = np.array([ping.position.x, ping.position.y, ping.position.z])
    vec = vec / np.linalg.norm(vec)

    if vectors_3d is None:
        vectors_3d = vec
    else:
        vectors_3d = np.vstack((vectors_3d, vec.T))

    print(vectors_3d)

    for x, y, z in vectors_3d:
        ax.plot([0, x], [0, y], zs=[0, z])


rospy.init_node('hydrophones_point_cloud')
sub = rospy.Subscriber('/hydrophones/processed', ProcessedPing, callback)
plt.ion()
plt.show(block=True)
rospy.spin()
