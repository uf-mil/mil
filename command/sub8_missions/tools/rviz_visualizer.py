#!/usr/bin/env python
from __future__ import division

import numpy as np
import rospy
import visualization_msgs.msg as visualization_msgs
from visualization_msgs.msg import Marker, InteractiveMarkerControl, InteractiveMarker
from interactive_markers.interactive_marker_server import *

from geometry_msgs.msg import Pose, Vector3
from std_msgs.msg import ColorRGBA, Float64
from uf_common.msg import Float64Stamped  # This needs to be deprecated
from sub8_alarm import AlarmListener, AlarmBroadcaster
import sub8_ros_tools as sub8_utils


class RvizVisualizer(object):
    '''Cute tool for drawing both depth and height-from-bottom in RVIZ
    '''

    def __init__(self):
        rospy.init_node('revisualizer')
        self.rviz_pub = rospy.Publisher("visualization/state", visualization_msgs.Marker, queue_size=2)
        self.rviz_pub_t = rospy.Publisher("visualization/state_t", visualization_msgs.Marker, queue_size=2)
        self.rviz_pub_utils = rospy.Publisher("visualization/bus_voltage", visualization_msgs.Marker, queue_size=2)
        self.kill_server = InteractiveMarkerServer("interactive_kill")

        # text marker
        # TODO: Clean this up, there should be a way to set all of this inline
        self.surface_marker = visualization_msgs.Marker()
        self.surface_marker.type = self.surface_marker.TEXT_VIEW_FACING
        self.surface_marker.color = ColorRGBA(1, 1, 1, 1)
        self.surface_marker.scale.z = 0.1

        self.depth_marker = visualization_msgs.Marker()
        self.depth_marker.type = self.depth_marker.TEXT_VIEW_FACING
        self.depth_marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        self.depth_marker.scale.z = 0.1

        # create marker for displaying current battery voltage
        self.low_battery_threshold = 43.0
        self.voltage_marker = visualization_msgs.Marker()
        self.voltage_marker.header.frame_id = "base_link"
        self.voltage_marker.lifetime = rospy.Duration(5)
        self.voltage_marker.ns = 'sub'
        self.voltage_marker.id = 22
        self.voltage_marker.pose.position.x = -2.0
        self.voltage_marker.scale.z = 0.2
        self.voltage_marker.color.a = 1
        self.voltage_marker.type = visualization_msgs.Marker.TEXT_VIEW_FACING


        # create an interactive marker to display kill status and change it
        self.need_kill_update = True
        self.kill_marker = InteractiveMarker()
        self.kill_marker.header.frame_id = "base_link"
        self.kill_marker.pose.position.x = -2.3
        self.kill_marker.name = "kill button"
        kill_status_marker = Marker()
        kill_status_marker.type = Marker.TEXT_VIEW_FACING
        kill_status_marker.text = "UNKILLED"
        kill_status_marker.id = 55
        kill_status_marker.scale.z = 0.2
        kill_status_marker.color.a = 1.0
        kill_button_control = InteractiveMarkerControl()
        kill_button_control.name = "kill button"
        kill_button_control.interaction_mode = InteractiveMarkerControl.BUTTON
        kill_button_control.markers.append(kill_status_marker)
        self.kill_server.insert(self.kill_marker, self.kill_buttton_callback)
        self.kill_marker.controls.append(kill_button_control)
        self.killed = False

        # connect kill marker to kill alarm
        self.kill_listener = AlarmListener(alarm_name="kill", callback_funct=self.kill_alarm_callback)
        self.kill_alarm = AlarmBroadcaster().add_alarm("kill")

        # distance to bottom
        self.range_sub = rospy.Subscriber("dvl/range", Float64Stamped, self.range_callback)
        # distance to surface
        self.depth_sub = rospy.Subscriber("depth", Float64Stamped, self.depth_callback)
        # battery voltage
        self.voltage_sub = rospy.Subscriber("/bus_voltage", Float64, self.voltage_callback)


    def update_kill_button(self):
        if self.killed:
            self.kill_marker.controls[0].markers[0].text = "KILLED"
            self.kill_marker.controls[0].markers[0].color.r = 1
            self.kill_marker.controls[0].markers[0].color.g = 0
        else:
            self.kill_marker.controls[0].markers[0].text = "UNKILLED"
            self.kill_marker.controls[0].markers[0].color.r = 0
            self.kill_marker.controls[0].markers[0].color.g = 1
        self.kill_server.insert(self.kill_marker)
        self.kill_server.applyChanges()

    def kill_alarm_callback(self, alarm):
        self.need_kill_update = False
        self.killed = not alarm.clear
        self.update_kill_button()

    def kill_buttton_callback(self, feedback):
        if not feedback.event_type == 3:
            return
        if self.need_kill_update:
            return
        self.need_kill_update = True
        if self.killed:
            self.kill_alarm.clear_alarm()
        else:
            self.kill_alarm.raise_alarm()

    def voltage_callback(self, voltage):
        self.voltage_marker.text = str(round(voltage.data, 3)) + ' volts'
        self.voltage_marker.header.stamp = rospy.Time()
        if voltage.data < self.low_battery_threshold:
            self.voltage_marker.color.r = 1
            self.voltage_marker.color.g = 0
        else:
            self.voltage_marker.color.r = 0
            self.voltage_marker.color.g = 1
        self.rviz_pub_utils.publish(self.voltage_marker)

    def depth_callback(self, msg):
        '''Handle depth data sent from depth sensor'''
        frame = '/depth'
        distance = msg.data
        marker = self.make_cylinder_marker(
            np.array([0.0, 0.0, 0.0]),  # place at origin
            length=distance,
            color=(0.0, 1.0, 0.2, 0.7),  # green,
            frame=frame,
            id=0  # Keep these guys from overwriting eachother
        )
        self.surface_marker.ns='sub'
        self.surface_marker.header = sub8_utils.make_header(frame='/depth')
        self.surface_marker.pose = marker.pose
        self.surface_marker.text = str(round(distance, 3)) + 'm'
        self.surface_marker.id = 0

        self.rviz_pub.publish(marker)
        self.rviz_pub_t.publish(self.depth_marker)

    def range_callback(self, msg):
        '''Handle range data grabbed from dvl'''
        # future: should be /base_link/dvl, no?
        frame = '/dvl'
        distance = msg.data

        # Color a sharper red if we're in danger
        if distance < 1.0:
            color = (1.0, 0.1, 0.0, 0.9)
        else:
            color = (0.2, 0.8, 0.0, 0.7)

        marker = self.make_cylinder_marker(
            np.array([0.0, 0.0, 0.0]),  # place at origin
            length=distance,
            color=color,  # red,
            frame=frame,
            up_vector=np.array([0.0, 0.0, -1.0]),  # up is down in range world
            id=1  # Keep these guys from overwriting eachother
        )
        self.depth_marker.ns='sub'
        self.depth_marker.header = sub8_utils.make_header(frame='/dvl')
        self.depth_marker.pose = marker.pose
        self.depth_marker.text = str(round(distance, 3)) + 'm'
        self.depth_marker.id = 1

        self.rviz_pub_t.publish(self.depth_marker)
        self.rviz_pub.publish(marker)

    def make_cylinder_marker(self, base, length, color, frame='/base_link', up_vector=np.array([0.0, 0.0, 1.0]), **kwargs):
        '''Handle the frustration that Rviz cylinders are designated by their center, not base'''

        center = base + (up_vector * (length / 2))

        pose = Pose(
            position=sub8_utils.numpy_to_point(center),
            orientation=sub8_utils.numpy_to_quaternion([0.0, 0.0, 0.0, 1.0])
        )

        marker = visualization_msgs.Marker(
            ns='sub',
            header=sub8_utils.make_header(frame=frame),
            type=visualization_msgs.Marker.CYLINDER,
            action=visualization_msgs.Marker.ADD,
            pose=pose,
            color=ColorRGBA(*color),
            scale=Vector3(0.2, 0.2, length),
            lifetime=rospy.Duration(),
            **kwargs
        )
        return marker


if __name__ == '__main__':
    vizualizer = RvizVisualizer()
    rospy.spin()
