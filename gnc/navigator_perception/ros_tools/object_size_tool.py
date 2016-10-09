# !/usr/bin/python
import rospy
from navigator_msgs.msg import BuoyArray
from nav_msgs.msg import Odometry
import navigator_tools as nt
import numpy as np


class SizeToolObject():

    def __init__(self):
        self.count = 0
        self.sum_vol = 0
        self.min_vol = 10000
        self.max_vol = 0
        self.sum_height = 0
        self.min_height = 10000
        self.max_height = 0
        self.sum_dist = 0


class ObjectSizeTool(object):

    def __init__(self):
        self.sub_unclassified = rospy.Subscriber('/unclassified/objects', BuoyArray, self.new_objects)
        self._odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.ids = []
        self.objects = {}
        self.position = None
        self.rot = None

    def odom_cb(self, odom):
        self.position, self.rot = nt.odometry_to_numpy(odom)[0]

    def new_objects(self, buoy_array):
        for b in buoy_array.buoys:
            if b.id in self.objects.keys():
                o = self.objects[b.id]
                v = b.height * b.width * b.depth
                o.sum_vol += v
                o.sum_height += b.height
                if self.position is not None:
                    o.sum_dist += np.linalg.norm(nt.rosmsg_to_numpy(b.position) - self.position)

                if b.height < o.min_height:
                    o.min_height = b.height

                if b.height > o.max_height:
                    o.max_height = b.height

                if v < o.min_vol:
                    o.min_vol = v

                if v > o.max_vol:
                    o.max_vol = v

                o.count += 1

            elif b.id in self.ids:
                o = SizeToolObject()
                o.count += 1
                self.objects[b.id] = o


def f(x):
    result = []
    for part in x.split(','):
        if '-' in part:
            a, b = part.split('-')
            a, b = int(a), int(b)
            result.extend(range(a, b + 1))
        else:
            a = int(part)
            result.append(a)
    return result

if __name__ == '__main__':
    rospy.init_node('object_size_tools')
    o = ObjectSizeTool()
    while True:
        x = raw_input("Enter ids you want: ")
        if x == 'q':
            break
        ids = f(x)
        for i in ids:
            o.ids.append(i)

    for key, v in o.objects.iteritems():
        print key, ": "
        print "Average Volume, Min, Max:", v.sum_vol / v.count, v.min_vol, v.max_vol
        print "Average Height, Min, Max:", v.sum_height / v.count, v.min_height, v.max_height
        print "Average Distance", v.sum_dist / v.count
