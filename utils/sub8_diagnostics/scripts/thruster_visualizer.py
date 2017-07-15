#!/usr/bin/env python
from __future__ import division
import rospy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from sub8_msgs.msg import Thrust
from mil_ros_tools import numpy_to_point

__author__ = 'Kevin Allen'


class ThrusterVisualizer(object):
    '''
    ROS node to monitor SubjuGator's thrusters and publish RVIZ marker arrows
    representing the current requested effort of each thruster.

    Useful for diagnosing thruster mapper issues.
    '''
    MAX_LENGTH = 0.3  # Length of arrow when at max effort in current direction

    def __init__(self):
        self.layout = rospy.get_param('/thruster_layout/thrusters')  # Add thruster layout from ROS param set by mapper
        assert self.layout is not None, 'Could not load thruster layout from ROS param'

        '''
        Create MarkerArray with an arrow marker for each thruster at index node_id.
        The position of the marker is as specified in the layout, but the length of the arrow
        will be set at each thrust callback.
        '''
        self.markers = MarkerArray()
        for i in xrange(len(self.layout)):
            # Initialize each marker (color, scale, namespace, frame)
            m = Marker()
            m.header.frame_id = '/base_link'
            m.type = Marker.ARROW
            m.ns = 'thrusters'
            m.color.a = 0.8
            m.scale.x = 0.01  # Shaft diameter
            m.scale.y = 0.05  # Head diameter
            m.action = Marker.DELETE
            m.lifetime = rospy.Duration(0.1)
            self.markers.markers.append(m)
        for key, layout in self.layout.iteritems():
            # Set position and id of marker from thruster layout
            idx = layout['node_id']
            pt = numpy_to_point(layout['position'])
            self.markers.markers[idx].points.append(pt)
            self.markers.markers[idx].points.append(pt)
            self.markers.markers[idx].id = idx

        # Create publisher for marker and subscribe to thrust
        self.pub = rospy.Publisher('/thrusters/markers', MarkerArray, queue_size=5)
        self.thrust_sub = rospy.Subscriber('/thrusters/thrust', Thrust, self.thrust_cb, queue_size=5)

    def thrust_cb(self, thrust):
        '''
        Each thrust callback, update the length of the arrow
        based on the commanded thrust (in newtons).

        Also update the color of the arrow based on thrust, from green to yellow, with
        red being reserved for bounds.
        '''
        for cmd in thrust.thruster_commands:
            if cmd.name not in self.layout:  # Don't draw marker if thruster is not in layout
                continue
            layout = self.layout[cmd.name]
            idx = layout['node_id']
            bounds = layout['thrust_bounds']

            # Select an arrow length based on commanded thrust, max thrust (from layout), and the MAX_LENGTH constant
            if cmd.thrust < 0:
                scale = -cmd.thrust / bounds[0]
            else:
                scale = cmd.thrust / bounds[1]

            if np.isclose(scale, 0.0):  # Avoid sending 0 length disk-like markers
                self.markers.markers[idx].action = Marker.DELETE
                continue
            else:
                self.markers.markers[idx].action = Marker.ADD

            # Set color of marker based on thrust
            if (cmd.thrust < 0 and cmd.thrust == bounds[0]) or cmd.thrust == bounds[1]:
                self.markers.markers[idx].color.r = 1.0
                self.markers.markers[idx].color.g = 0.0
            else:
                self.markers.markers[idx].color.r = abs(scale)
                self.markers.markers[idx].color.g = 1.0

            # Select endpoint for arrow based on length and direction vector from layout
            direction = np.array(layout['direction'])
            direction = direction / np.linalg.norm(direction)
            pt2 = np.array(layout['position']) + self.MAX_LENGTH * scale * direction

            self.markers.markers[idx].points[1] = numpy_to_point(pt2)
            self.markers.markers[idx].header.stamp = rospy.Time.now()

        self.pub.publish(self.markers)


if __name__ == '__main__':
    rospy.init_node('thruster_visualizer')
    ThrusterVisualizer()
    rospy.spin()
