#!/usr/bin/env python
from txros import util
from navigator import Navigator
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from genpy import Duration
import yaml


class ConstantVelocity(Navigator):
    '''
    Mission to command a constant velocity to NaviGator's controller.
    '''
    ZERO_TIME = 5.0     # Time to publish zero velocity to stabilize before commanding constant
    CLEANUP_TIME = 0.5  # Time to publish zero velocity when command canceled

    @classmethod
    def init(cls):
        cls.msg = Odometry()
        cls.msg.header.frame_id = 'enu'
        cls.msg.child_frame_id = 'base_link'
        cls.ref_pub = cls.nh.advertise('/trajectory/constant', Odometry)

    @classmethod
    @util.cancellableInlineCallbacks
    def cleanup(cls):
        '''
        When command is canceled, publish zero velocity for a while
        so controller doesn't "run of the tracks" and keep going
        '''
        odom = yield cls._odom_sub.get_next_message()
        cls.msg.pose = odom.pose
        cls.msg.header.stamp = odom.header.stamp
        cls.msg.twist.twist = Twist()
        now = yield cls.nh.get_time()
        done = now + Duration(cls.CLEANUP_TIME)
        while now < done:
            cls.msg.header.stamp = now
            cls.ref_pub.publish(cls.msg)
            now = yield cls.nh.get_time()
            yield cls.nh.sleep(0.1)

    @classmethod
    def decode_parameters(cls, parameters):
        '''
        Ensure arguments are in the form "[x, y, z]" velocity
        where x, y and the desired linear velocity in base_link frame in m/s
        and z is desired angular velocity in base_link frame in rad/s
        '''
        err = Exception('parameters must be in form [x,y,yaw] in m/s,m/s,rad/s')
        if len(parameters) == 0:
            raise err
        parsed = yaml.load(parameters)
        if not isinstance(parsed, list) or len(parsed) != 3:
            raise err
        for i in range(3):
            if not (isinstance(parsed[i], int) or isinstance(parsed[i], float)):
                raise err
        return parsed

    @util.cancellableInlineCallbacks
    def run(self, args):
        # Publish a velocity of zero for a while to stabalize navigator
        self.send_feedback('Switching trajectory to constant')
        yield self.change_trajectory('constant')
        yield self.nh.sleep(0.1)
        done_zero = yield self.nh.get_time() + Duration(self.ZERO_TIME)
        odom = yield self._odom_sub.get_next_message()
        self.msg.header.stamp = odom.header.stamp
        self.msg.pose = odom.pose
        self.msg.twist.twist = Twist()
        self.send_feedback('Sending Zero Velocity for {} seconds'.format(self.ZERO_TIME))
        while (yield self.nh.get_time()) < done_zero:
            self.ref_pub.publish(self.msg)
            yield self.nh.sleep(0.1)

        # Publish user selected velocity until task is canceled or a new task is run
        self.send_feedback('Publishing constant velocity {}. Cancel task to stop.'.format(args))
        self.msg.twist.twist.linear.x = args[0]
        self.msg.twist.twist.linear.y = args[1]
        self.msg.twist.twist.angular.z = args[2]
        original_odom = yield self._odom_sub.get_last_message()
        while True:
            odom = yield self._odom_sub.get_next_message()
            self.msg.header.stamp = odom.header.stamp
            self.msg.pose = odom.pose
            if args[0] == 0 and args[1] == 0:  # If only rotation, keep position same
                self.msg.pose.pose.position = original_odom.pose.pose.position
            elif args[2] == 0:  # If only linear, keep orientation same
                self.msg.pose.pose.orientation = original_odom.pose.pose.orientation
            self.ref_pub.publish(self.msg)
