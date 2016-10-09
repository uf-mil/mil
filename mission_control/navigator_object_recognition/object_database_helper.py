import rospy
from navigator_msgs.msg import PerceptionObjects
from nav_msgs.msg import Odometry
import navigator_tools as nt
import numpy as np


class ObjectDBHelper(object):

    def __init__(self):
        self.position = None
        self.rot = None
        self.unknowns = []
        self.objects = []
        self._objects_sub = rospy.Subscriber('/unclassified/objects', PerceptionObjects, self.object_cb)
        self._odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb)

    def odom_cb(self, odom):
        self.position, self.rot = nt.odometry_to_numpy(odom)[0]

    def object_cb(self, perception_objects):
        for i in perception_objects:
            if i.name == 'unknown':
                self.unknowns.append(i)
            else:
                self.objects.append(i)

    def get_closest_uknown(self):
        """Return a PerceptionObject that is the closest unknown to the current position of the boat."""
        if self.position is None or len(self.unknowns) == 0:
            return None
        return min(self.unknowns, key=lambda x: np.linalg.norm(nt.rosmsg_to_numpy(x) - self.position))
