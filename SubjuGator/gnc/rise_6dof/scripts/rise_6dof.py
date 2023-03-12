#!/usr/bin/env python3

import contextlib
import traceback

import numpy
import rospy
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from geometry_msgs.msg import Vector3, Wrench, WrenchStamped
from mil_msgs.msg import PoseTwist, PoseTwistStamped
from mil_ros_tools import pose_to_numpy, twist_to_numpy
from nav_msgs.msg import Odometry
from rise_6dof import controller
from rise_6dof.cfg import controllerConfig
from rise_6dof.srv import SendConstantWrench
from ros_alarms import AlarmListener
from std_msgs.msg import Header


class Node:
    def reconfigure(self, config, level):
        try:
            processed_config = {}
            for name in "k ks alpha beta accel_feedforward vel_feedforward".split(" "):
                x = numpy.array(map(float, config[name].split(",")))
                assert len(x) == 6
                processed_config[name] = x
            processed_config["use_rise"] = config["use_rise"]
            processed_config["two_d_mode"] = config["two_d_mode"]

            self.config = config
            self.processed_config = processed_config

            return config
        except Exception:
            traceback.print_exc()
            # go back to old config if new is invalid
            return self.config

    def __init__(self):
        self.kill_listener = AlarmListener("kill")

        self.reconfigure(controllerConfig.defaults, 0)
        self.server = DynamicReconfigureServer(controllerConfig, self.reconfigure)

        self.controller = controller.Controller(self.processed_config)

        self.desired = None
        rospy.Subscriber(
            "/desired",
            PoseTwistStamped,
            lambda desired: setattr(self, "desired", desired),
        )

        self.current = None
        rospy.Subscriber(
            "/current",
            Odometry,
            lambda current: setattr(self, "current", current),
        )
        # rospy.Subscriber('/odom', Odometry, lambda current: setattr(self, 'current', current))

        self.constant_wrench = None
        self.constant_wrench_time = None
        rospy.Service(
            "send_constant_wrench",
            SendConstantWrench,
            self.send_constant_wrench,
        )

        pub = rospy.Publisher("/output", WrenchStamped, queue_size=2)
        pd_pub = rospy.Publisher("/pd_output", WrenchStamped, queue_size=2)

        dt = 1 / 50

        while not rospy.is_shutdown():
            rospy.sleep(dt)

            if rospy.Time.now() < rospy.Time(0.5):
                self.kill()
                continue

            if self.kill_listener.is_raised():
                self.kill()
                # Published zeroed wrench when killed so thruster feedback is
                # still produced
                pub.publish(
                    WrenchStamped(
                        header=Header(stamp=rospy.Time.now(), frame_id="/base_link"),
                        wrench=Wrench(force=Vector3(0, 0, 0), torque=Vector3(0, 0, 0)),
                    ),
                )
                continue

            if self.current is None:
                rospy.logwarn("RISE has not gotten a current waypoint")
                rospy.sleep(1 / 10)
                continue

            if self.desired is None:
                # default to original pose
                self.desired = PoseTwistStamped(
                    header=Header(
                        frame_id=self.current.header.frame_id,
                    ),
                    posetwist=PoseTwist(
                        pose=self.current.pose.pose,
                    ),
                )

            assert self.current.header.frame_id == self.desired.header.frame_id
            assert self.current.header.frame_id in ["map", "/odom", "/enu"]

            if not self.is_constant_wrench_valid():
                self.controller.config = self.processed_config
                pd_wrench, wrench = self.controller.update(
                    dt,
                    (
                        pose_to_numpy(self.desired.posetwist.pose),
                        twist_to_numpy(self.desired.posetwist.twist),
                        twist_to_numpy(self.desired.posetwist.acceleration),
                    ),
                    (
                        pose_to_numpy(self.current.pose.pose),
                        twist_to_numpy(self.current.twist.twist),
                    ),
                )

                pd_pub.publish(
                    WrenchStamped(
                        header=Header(
                            stamp=rospy.Time.now(),
                            frame_id="/base_link",
                        ),
                        wrench=Wrench(
                            force=Vector3(*pd_wrench[0]),
                            torque=Vector3(*pd_wrench[1]),
                        ),
                    ),
                )
                wrench = Wrench(force=Vector3(*wrench[0]), torque=Vector3(*wrench[1]))
            else:
                wrench = self.constant_wrench.wrench

            pub.publish(
                WrenchStamped(
                    header=Header(
                        stamp=rospy.Time.now(),
                        frame_id="/base_link",
                    ),
                    wrench=wrench,
                ),
            )

    def is_constant_wrench_valid(self):
        if self.constant_wrench is None:
            return False
        return (
            rospy.Duration(0)
            <= rospy.Time.now() - self.constant_wrench_time
            < self.constant_wrench.lifetime
        )

    def send_constant_wrench(self, constant_wrench):
        self.constant_wrench_time = rospy.Time.now()
        self.constant_wrench = constant_wrench
        while self.is_constant_wrench_valid():
            rospy.sleep(0.1)
        return ()

    def kill(self):
        self.current = None
        self.desired = None
        self.controller.reset()


if __name__ == "__main__":
    rospy.init_node("rise_6dof")
    with contextlib.suppress(rospy.ROSInterruptException):
        Node()
