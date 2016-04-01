#!/usr/bin/env python
import rospy
import sub8_ros_tools as sub8_utils
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped, PoseWithCovariance, TwistWithCovariance
from gazebo_msgs.srv import ApplyBodyWrench
from gazebo_msgs.msg import LinkStates


class GazeboInterface(object):
    def __init__(self, target='sub8::base_link'):
        self.target = target
        rospy.wait_for_service('/gazebo/apply_body_wrench')
        self.wrench_srv = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        self.wrench_sub = rospy.Subscriber('wrench', WrenchStamped, self.wrench_cb)
        self.state_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.state_cb)
        self.state_pub = rospy.Publisher('odom', Odometry, queue_size=1)

    def send_wrench(self, wrench):
        self.wrench_srv(
            body_name=self.target,
            reference_frame=self.target,
            wrench=wrench,
            duration=rospy.Duration(0.001)
        )

    def wrench_cb(self, msg):
        wrench = msg.wrench
        self.send_wrench(wrench)

    def state_cb(self, msg):
        '''TODO: add noise'''
        if self.target in msg.name:
            header = sub8_utils.make_header(frame='/map')

            target_index = msg.name.index(self.target)
            pose = msg.pose[target_index]
            twist = msg.twist[target_index]
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

        else:
            # fail
            return


if __name__ == '__main__':
    rospy.init_node('gazebo_interface')
    GI = GazeboInterface()
    rospy.spin()
