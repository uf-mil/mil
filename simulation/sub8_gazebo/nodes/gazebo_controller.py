#!/usr/bin/env python
import rospy
import math
import mil_ros_tools
import numpy as np
from tf import transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Pose, Point, Quaternion, WrenchStamped
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import ApplyBodyWrench
from gazebo_msgs.msg import LinkStates, ModelState
from sub8_gazebo.srv import ResetGazebo, ResetGazeboResponse
from mil_msgs.msg import RangeStamped
from mil_blueview_driver.msg import BlueViewPing


class GazeboInterface(object):

    def __init__(self, target='sub8::base_link'):
        self.target = target
        self.physics = rospy.get_param('/physics_flag', False)
        if self.physics:
            rospy.wait_for_service('/gazebo/apply_body_wrench')
            self.wrench_srv = rospy.ServiceProxy(
                '/gazebo/apply_body_wrench', ApplyBodyWrench)
            # For now, let's skip the wrench
            self.wrench_sub = rospy.Subscriber(
                'wrench', WrenchStamped, self.wrench_cb)

        self.last_odom = None
        self.position_offset = None
        self.state_sub = rospy.Subscriber(
            '/gazebo/link_states', LinkStates, self.state_cb)
        self.state_set_pub = rospy.Publisher(
            '/gazebo/set_model_state', ModelState, queue_size=1)

        self.raw_dvl_sub = rospy.Subscriber(
            'dvl/range_raw', Range, self.publish_height)
        self.dvl_pub = rospy.Publisher('dvl/range', RangeStamped, queue_size=1)

        self.raw_blueview_sub = rospy.Subscriber(
            '/blueview_driver/gazebo', LaserScan, self.publish_blueview_ping)
        self.blueview_ranges_pub = rospy.Publisher(
            '/blueview_driver/ranges', BlueViewPing, queue_size=1)

        self.reset_srv = rospy.Service(
            'gazebo/reset_gazebo', ResetGazebo, self.reset)
        self.state_pub = rospy.Publisher('model_odom', Odometry, queue_size=1)
        self.world_state_pub = rospy.Publisher(
            'world_odom', Odometry, queue_size=1)

        self.odom_freq = 0.03
        rospy.Timer(rospy.Duration(self.odom_freq), self.publish_odom)

    def send_wrench(self, wrench):
        self.wrench_srv(
            body_name=self.target,
            reference_frame='world',
            wrench=wrench,
            duration=rospy.Duration(0.001)
        )

    def reset(self, srv):
        model = self.target.split('::')[0]
        self.state_set_pub.publish(ModelState(
            model_name=model,
            pose=Pose(
                position=Point(*self.position_offset),
                orientation=Quaternion(
                    *transformations.quaternion_from_euler(0, 0, 0))
            )
        ))
        return ResetGazeboResponse()

    def wrench_cb(self, msg):
        wrench = msg.wrench
        self.send_wrench(wrench)

    def publish_height(self, msg):
        '''Sim DVL uses laserscan message to relay height'''
        self.dvl_pub.publish(
            RangeStamped(
                header=mil_ros_tools.make_header(),
                range=msg.range
            )
        )

    def publish_blueview_ping(self, msg):
        self.blueview_ranges_pub.publish(
            BlueViewPing(
                header=msg.header,
                bearings=[(msg.angle_min + i * msg.angle_increment)
                          for i in xrange(len(msg.ranges))],
                ranges=msg.ranges,
                intensities=[np.uint16(x) if not math.isinf(
                    x) else 0 for x in msg.ranges]
            )
        )

    def publish_odom(self, *args):
        if self.last_odom is None or self.position_offset is None:
            return

        msg = self.last_odom
        if self.target in msg.name:
            header = mil_ros_tools.make_header(frame='/map')

            target_index = msg.name.index(self.target)
            twist = msg.twist[target_index]

            # Add position offset to make the start position (0, 0, -depth)
            position_np, orientation_np = mil_ros_tools.pose_to_numpy(msg.pose[
                                                                      target_index])
            pose = mil_ros_tools.numpy_quat_pair_to_pose(
                position_np - self.position_offset, orientation_np)

            self.state_pub.publish(
                header=header,
                child_frame_id='/base_link',
                pose=PoseWithCovariance(
                    pose=pose
                ),
                twist=TwistWithCovariance(
                    twist=twist
                )
            )

            header = mil_ros_tools.make_header(frame='/world')
            twist = msg.twist[target_index]
            pose = msg.pose[target_index]
            self.world_state_pub.publish(
                header=header,
                child_frame_id='/base_link',
                pose=PoseWithCovariance(
                    pose=pose
                ),
                twist=TwistWithCovariance(
                    twist=twist
                )
            )
        else:
            # fail
            return

    def state_cb(self, msg):
        '''
        Position is offset so first message is taken as zero point. (More reflective of actual sub).
        Z position is absolute and so is rotation.

        TODO: Add noise
        '''
        if self.target not in msg.name:
            return
        if (self.last_odom is None or self.position_offset is None):
            pose = msg.pose[msg.name.index(self.target)]
            position, orientation = mil_ros_tools.pose_to_numpy(pose)
            position[2] = 0.0
            self.position_offset = position

        self.last_odom = msg


if __name__ == '__main__':
    rospy.init_node('gazebo_interface')
    GI = GazeboInterface()
    rospy.spin()
