#!/usr/bin/env python
import rospy
import sub8_ros_tools as sub8_utils
from tf import transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped, PoseWithCovariance, TwistWithCovariance, Pose, Point, Quaternion
from gazebo_msgs.srv import ApplyBodyWrench
from gazebo_msgs.msg import LinkStates, ModelState
from sub8_msgs.srv import ResetGazebo, ResetGazeboResponse
from uf_common.msg import Float64Stamped


class GazeboInterface(object):
    def __init__(self, target='sub8::base_link'):
        self.target = target
        # rospy.wait_for_service('/gazebo/apply_body_wrench')
        # self.wrench_srv = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        # For now, let's skip the wrench
        # self.wrench_sub = rospy.Subscriber('wrench', WrenchStamped, self.wrench_cb)
        self.last_odom = None
        self.state_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.state_cb)
        self.state_set_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)

        self.dvl_pub = rospy.Publisher('dvl/range', Float64Stamped, queue_size=1)

        self.reset_srv = rospy.Service('gazebo/reset_gazebo', ResetGazebo, self.reset)
        self.state_pub = rospy.Publisher('odom', Odometry, queue_size=1)
        rospy.Timer(rospy.Duration(0.03), self.publish_odom)

    def send_wrench(self, wrench):
        self.wrench_srv(
            body_name=self.target,
            reference_frame=self.target,
            wrench=wrench,
            duration=rospy.Duration(0.001)
        )

    def reset(self, srv):
        model = self.target.split('::')[0]
        self.state_set_pub.publish(ModelState(
            model_name=model,
            pose=Pose(
                position=Point(0, 0, -15),
                orientation=Quaternion(*transformations.quaternion_from_euler(0, 0, 0.3))
            )
        ))
        return ResetGazeboResponse()

    def wrench_cb(self, msg):
        wrench = msg.wrench
        self.send_wrench(wrench)

    def publish_height(self, z):
        '''HACK. Lanford to fix'''
        self.dvl_pub.publish(
            Float64Stamped(
                header=sub8_utils.make_header(),
                data=(20.0 + z)
            )
        )

    def publish_odom(self, *args):
        if self.last_odom is None:
            return

        msg = self.last_odom
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
            self.publish_height(pose.position.z)

        else:
            # fail
            return

    def state_cb(self, msg):
        '''TODO: add noise'''
        self.last_odom = msg


if __name__ == '__main__':
    rospy.init_node('gazebo_interface')
    GI = GazeboInterface()
    rospy.spin()
