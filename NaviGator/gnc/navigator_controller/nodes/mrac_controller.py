#!/usr/bin/python3

import traceback

import numpy as np
import numpy.linalg as npl
import rospy
import tf.transformations as trns
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, WrenchStamped
from nav_msgs.msg import Odometry
from navigator_msgs.msg import PoseTwistStamped
from std_msgs.msg import Float64MultiArray, Header, String
from vrx_gazebo.msg import Task


class MRAC_Controller:
    LEARN_WRENCHES = ["/wrench/autonomous", "autonomous"]

    def __init__(self):
        # TUNABLES
        # Proportional gains, body frame
        self.kp_body = np.diag(rospy.get_param("~kp", [1000.0, 1000.0, 5600.0]))
        # Derivative gains, body frame
        self.kd_body = np.diag(rospy.get_param("~kd", [1200.0, 1200.0, 6000.0]))
        # Disturbance adaptation rates, world frame
        self.ki = np.array(rospy.get_param("~ki", [0.1, 0.1, 0.1]))
        # Drag adaptation rates, world frame
        self.kg = np.array(rospy.get_param("~kg", [5.0, 5.0, 5.0, 5.0, 5.0]))
        # X error threshold for long trajectory to be satisfied
        self.x_thresh = rospy.get_param("x_thresh", 0.5)
        # Y error threshold for long trajectory to be satisfied
        self.y_thresh = rospy.get_param("y_thresh", 0.5)
        # Yaw error threshold for long trajectory to be satisfied
        self.yaw_thresh = rospy.get_param("yaw_thresh", 0.02)
        self.task_info_sub = rospy.Subscriber(
            "/vrx/task/info", Task, self.taskinfoSubscriber
        )
        self.thresholds_changed = False
        # Initial disturbance estimate
        self.dist_est = np.array([0, 0, 0])
        # Initial drag estimate
        self.drag_est = np.array([0, 0, 0, 0, 0])  # [d1 d2 Lc1 Lc2 Lr]
        # Limits on disturbance estimate
        self.dist_limit = np.array([200, 200, 200])
        # Limits on drag estimate effort
        self.drag_limit = np.array([1000, 1000, 1000])
        # User-imposed speed limit
        self.vel_max_body_positive = np.array([1.1, 0.45, 0.19])  # [m/s, m/s, rad/s]
        self.vel_max_body_negative = np.array([0.68, 0.45, 0.19])  # [m/s, m/s, rad/s]
        # "Smart heading" threshold
        self.heading_threshold = 500  # m
        # Only use PD controller
        self.only_PD = False
        self.learning_radius = 10  # m
        # Use external trajectory generator instead of internal reference generator
        # Subscribes to /trajectory if using external, else just takes /waypoint for the end goal
        self.use_external_tgen = True
        # experimental
        self.kp_body_orig = self.kp_body
        self.kd_body_orig = self.kd_body
        self.heavy_pid = False
        self.old_p_ref = None

        # REFERENCE MODEL (note that this is not the adaptively estimated TRUE model; rather,
        #                     these parameters will govern the trajectory we want to achieve).
        self.mass_ref = rospy.get_param(
            "~mass", 500
        )  # kg, determined to be larger than boat in practice due to water mass  ## might change to lowe number (350kg)
        self.inertia_ref = rospy.get_param(
            "~izz", 400
        )  # kg*m^2, determined to be larger than boat in practice due to water mass
        self.thrust_max = 220  # N

        # back-left, back-right, front-left front-right, thruster positions in meters
        self.thruster_positions = np.array(
            [
                [-1.9000, 1.0000, -0.0123],
                [-1.9000, -1.0000, -0.0123],
                [1.6000, 0.6000, -0.0123],
                [1.6000, -0.6000, -0.0123],
            ]
        )

        # back-left, back-right, front-left, front-right thruster directions in radians
        self.thruster_directions = np.array(
            [
                [0.7071, 0.7071, 0.0000],
                [0.7071, -0.7071, 0.0000],
                [0.7071, -0.7071, 0.0000],
                [0.7071, 0.7071, 0.0000],
            ]
        )
        self.lever_arms = np.cross(self.thruster_positions, self.thruster_directions)
        self.B_body = np.concatenate((self.thruster_directions.T, self.lever_arms.T))
        self.Fx_max_body = self.B_body.dot(self.thrust_max * np.array([1, 1, 1, 1]))
        self.Fy_max_body = self.B_body.dot(self.thrust_max * np.array([1, -1, -1, 1]))
        self.Mz_max_body = self.B_body.dot(self.thrust_max * np.array([-1, 1, -1, 1]))
        self.D_body_positive = (
            abs(
                np.array(
                    [self.Fx_max_body[0], self.Fy_max_body[1], self.Mz_max_body[5]]
                )
            )
            / self.vel_max_body_positive**2
        )
        self.D_body_negative = (
            abs(
                np.array(
                    [self.Fx_max_body[0], self.Fy_max_body[1], self.Mz_max_body[5]]
                )
            )
            / self.vel_max_body_negative**2
        )

        # BASIC INITIALIZATIONS
        # Position waypoint
        self.p_des = np.array([0, 0])
        # Orientation waypoint
        self.q_des = np.array([1, 0, 0, 0])
        # Total distance that will be traversed for this waypoint
        self.traversal = 0
        # Reference states
        self.p_ref = None  # np.array([0, 0])
        self.v_ref = np.array([0, 0])
        self.q_ref = np.array([0, 0, 0, 0])
        self.w_ref = 0
        self.a_ref = np.array([0, 0])
        self.aa_ref = 0
        # Messages
        self.last_odom = None
        self.learn = False

        # ROS
        # Time since last controller call = 1/controller_call_frequency
        self.timestep = 0.02
        # For unpacking ROS messages later on
        self.position = np.zeros(2)
        self.orientation = np.array([1, 0, 0, 0])
        self.lin_vel = np.zeros(2)
        self.ang_vel = 0
        self.state = Odometry()

        self.use_lqrrt = True

        # Subscribers
        if self.use_lqrrt:
            rospy.Subscriber("/trajectory/cmd", Odometry, self.set_traj_from_odom_msg)
            rospy.Subscriber("/trajectory_long/cmd", Odometry, self.set_waypoint)
        else:
            rospy.Subscriber("/trajectory_long/cmd", Odometry, self.set_waypoint)

        rospy.Subscriber("/odom", Odometry, self.get_command)
        rospy.Subscriber("/wrench/selected", String, self.set_learning)
        # Publishers
        self.wrench_pub = rospy.Publisher(
            "/wrench/autonomous", WrenchStamped, queue_size=0
        )
        self.pose_ref_pub = rospy.Publisher("pose_ref", PoseStamped, queue_size=0)
        self.adaptation_pub = rospy.Publisher("adaptation", WrenchStamped, queue_size=0)
        self.theta_pub = rospy.Publisher("~theta", Float64MultiArray, queue_size=0)

        rospy.spin()

    def taskinfoSubscriber(self, msg):
        if not self.thresholds_changed and msg.name == "wayfinding":
            self.x_thresh = 0.15
            self.y_thresh = 0.15
            self.yaw_thresh = 0.005
            self.thresholds_changed = True

    def set_traj(self, msg: PoseTwistStamped) -> None:
        """
        Sets instantaneous reference state.
        Convert twist to world frame for controller math.

        Serves as the callback function for the subscriber to the /trajectory
        topic.

        Args:
            msg: PoseTwistStamped - The message to the callback by
              the subscriber.
        """
        self.p_ref = np.array(
            [msg.posetwist.pose.position.x, msg.posetwist.pose.position.y]
        )
        self.q_ref = np.array(
            [
                msg.posetwist.pose.orientation.x,
                msg.posetwist.pose.orientation.y,
                msg.posetwist.pose.orientation.z,
                msg.posetwist.pose.orientation.w,
            ]
        )

        R = trns.quaternion_matrix(self.q_ref)[:3, :3]

        self.v_ref = R.dot(
            np.array(
                [
                    msg.posetwist.twist.linear.x,
                    msg.posetwist.twist.linear.y,
                    msg.posetwist.twist.linear.z,
                ]
            )
        )[:2]
        self.w_ref = R.dot(
            np.array(
                [
                    msg.posetwist.twist.angular.x,
                    msg.posetwist.twist.angular.y,
                    msg.posetwist.twist.angular.z,
                ]
            )
        )[2]

        self.a_ref = R.dot(
            np.array(
                [
                    msg.posetwist.acceleration.linear.x,
                    msg.posetwist.acceleration.linear.y,
                    msg.posetwist.acceleration.linear.z,
                ]
            )
        )[:2]
        self.aa_ref = R.dot(
            np.array(
                [
                    msg.posetwist.acceleration.angular.x,
                    msg.posetwist.acceleration.angular.y,
                    msg.posetwist.acceleration.angular.z,
                ]
            )
        )[2]

    def set_traj_from_odom_msg(self, msg: Odometry) -> None:
        """
        Sets instantaneous reference state.
        Convert twist to world frame for controller math.

        Serves as the callback for the subscriber to the /trajectory/cmd
        topic.

        Args:
            msg: Odometry - The Odometry message from the topic.
        """
        if (
            self.p_ref is not None
            and self.old_p_ref is not None
            and self.old_p_ref[0] == msg.pose.pose.position.x
            and self.old_q_ref[0] == msg.pose.pose.orientation.x
        ):
            return

        self.p_ref = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.q_ref = np.array(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ]
        )

        self.old_p_ref = self.p_ref
        self.old_q_ref = self.q_ref

        R = trns.quaternion_matrix(self.q_ref)[:3, :3]

        self.v_ref = R.dot(
            np.array(
                [
                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y,
                    msg.twist.twist.linear.z,
                ]
            )
        )[:2]
        self.w_ref = R.dot(
            np.array(
                [
                    msg.twist.twist.angular.x,
                    msg.twist.twist.angular.y,
                    msg.twist.twist.angular.z,
                ]
            )
        )[2]

        self.a_ref = np.array([0, 0])
        self.aa_ref = 0

    def set_waypoint(self, msg) -> None:
        """
        Sets desired waypoint ("GO HERE AND STAY").
        Resets reference model to current state (i.e. resets trajectory generation).
        """
        self.p_ref = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.q_ref = np.array(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ]
        )

        R = trns.quaternion_matrix(self.q_ref)[:3, :3]

        self.v_ref = R.dot(
            np.array(
                [
                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y,
                    msg.twist.twist.linear.z,
                ]
            )
        )[:2]
        self.w_ref = R.dot(
            np.array(
                [
                    msg.twist.twist.angular.x,
                    msg.twist.twist.angular.y,
                    msg.twist.twist.angular.z,
                ]
            )
        )[2]

        self.a_ref = np.array([0, 0])
        self.aa_ref = 0
        self.heavy_pid = True
        self.kp_body = np.diag([1500.0, 1500.0, 5600.0])
        self.kd_body = np.diag([2000.0, 2000.0, 5000.0])

    def get_command(self, msg: Odometry) -> None:
        """
        Publishes the wrench for this instant.
        (Note: this is called get_command because it used to be used for
        getting the actual thruster values, but now it is only being
        used for getting the wrench which is then later mapped elsewhere).

        Args:
            msg: Odometry - The message passed to the callback by the
              subscriber.
        """
        if self.p_ref is None:
            return  # C3 is killed

        # Compute timestep from interval between this message's stamp and last's
        if self.last_odom is None:
            self.last_odom = msg
        else:
            self.timestep = (msg.header.stamp - self.last_odom.header.stamp).to_sec()
            self.last_odom = msg

        # ROS read-in
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        lin_vel_body = msg.twist.twist.linear
        ang_vel_body = msg.twist.twist.angular

        # ROS unpack
        self.position = np.array([position.x, position.y])
        self.orientation = np.array(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )

        # Frame management quantities
        R = np.eye(3)
        R[:2, :2] = trns.quaternion_matrix(self.orientation)[:2, :2]
        y = trns.euler_from_quaternion(self.orientation)[2]

        # More ROS unpacking, converting body frame twist to world frame lin_vel and ang_vel
        self.lin_vel = R.dot(
            np.array([lin_vel_body.x, lin_vel_body.y, lin_vel_body.z])
        )[:2]
        self.ang_vel = R.dot(
            np.array([ang_vel_body.x, ang_vel_body.y, ang_vel_body.z])
        )[2]

        # Convert body PD gains to world frame
        kp = R.dot(self.kp_body).dot(R.T)
        kd = R.dot(self.kd_body).dot(R.T)

        # Compute error components (reference - true)
        p_err = self.p_ref - self.position
        y_err = trns.euler_from_quaternion(
            trns.quaternion_multiply(
                self.q_ref, trns.quaternion_inverse(self.orientation)
            )
        )[2]
        v_err = self.v_ref - self.lin_vel
        w_err = self.w_ref - self.ang_vel

        # Adjust kp_body and kd_body based on p_err if heavy_pid was set
        if (
            (self.heavy_pid == True)
            and (abs(p_err[0]) < self.x_thresh)
            and (abs(p_err[1]) < self.y_thresh)
            and (abs(y_err) < self.yaw_thresh)
        ):
            print("threshold hit")
            self.kp_body = self.kp_body_orig
            self.kd_body = self.kd_body_orig
            self.heavy_pid = False
            self.movement_finished = True

        # Combine error components into error vectors
        err = np.concatenate((p_err, [y_err]))
        errdot = np.concatenate((v_err, [w_err]))

        # Compute "anticipation" feedforward based on the boat's inertia
        inertial_feedforward = np.concatenate((self.a_ref, [self.aa_ref])) * [
            self.mass_ref,
            self.mass_ref,
            self.inertia_ref,
        ]

        # Compute the "learning" matrix
        drag_regressor = np.array(
            [
                [
                    self.lin_vel[0] * np.cos(y) ** 2
                    + self.lin_vel[1] * np.sin(y) * np.cos(y),
                    self.lin_vel[0] / 2
                    - (self.lin_vel[0] * np.cos(2 * y)) / 2
                    - (self.lin_vel[1] * np.sin(2 * y)) / 2,
                    -self.ang_vel * np.sin(y),
                    -self.ang_vel * np.cos(y),
                    0,
                ],
                [
                    self.lin_vel[1] / 2
                    - (self.lin_vel[1] * np.cos(2 * y)) / 2
                    + (self.lin_vel[0] * np.sin(2 * y)) / 2,
                    self.lin_vel[1] * np.cos(y) ** 2
                    - self.lin_vel[0] * np.cos(y) * np.sin(y),
                    self.ang_vel * np.cos(y),
                    -self.ang_vel * np.sin(y),
                    0,
                ],
                [
                    0,
                    0,
                    self.lin_vel[1] * np.cos(y) - self.lin_vel[0] * np.sin(y),
                    -self.lin_vel[0] * np.cos(y) - self.lin_vel[1] * np.sin(y),
                    self.ang_vel,
                ],
            ]
        )

        # wrench = PD + feedforward + I + adaptation
        if self.only_PD:
            wrench = (kp.dot(err)) + (kd.dot(errdot))
            self.drag_effort = [0, 0, 0]
            self.dist_est = [0, 0, 0]
        else:
            self.drag_effort = np.clip(
                drag_regressor.dot(self.drag_est), -self.drag_limit, self.drag_limit
            )
            wrench = (
                (kp.dot(err))
                + (kd.dot(errdot))
                + inertial_feedforward
                + self.dist_est
                + self.drag_effort
            )
            # Update disturbance estimate, drag estimates
            if self.learn and (npl.norm(p_err) < self.learning_radius):
                self.dist_est = np.clip(
                    self.dist_est + (self.ki * err * self.timestep),
                    -self.dist_limit,
                    self.dist_limit,
                )
                self.drag_est = self.drag_est + (
                    self.kg * (drag_regressor.T.dot(err + errdot)) * self.timestep
                )

        # Update model reference for the next call
        if not self.use_external_tgen:
            self.increment_reference()

        # convert wrench to body frame
        wrench_body = R.T.dot(wrench)

        # NOT NEEDED SINCE WE ARE USING A DIFFERENT NODE FOR ACTUAL THRUSTER MAPPING
        # # Compute world frame thruster matrix (B) from thruster geometry, and then map wrench to thrusts
        # B = np.concatenate((R.dot(self.thruster_directions.T), R.dot(self.lever_arms.T)))
        # B_3dof = np.concatenate((B[:2, :], [B[5, :]]))
        # command = self.thruster_mapper(wrench, B_3dof)

        # Give wrench to ROS
        wrench_msg = WrenchStamped()
        wrench_msg.header.frame_id = "wamv/base_link"
        wrench_msg.wrench.force.x = wrench_body[0]
        wrench_msg.wrench.force.y = wrench_body[1]
        wrench_msg.wrench.torque.z = wrench_body[2]
        self.wrench_pub.publish(wrench_msg)

        # Publish reference pose for examination
        self.pose_ref_pub.publish(
            PoseStamped(
                header=Header(
                    frame_id="/enu",
                    stamp=msg.header.stamp,
                ),
                pose=Pose(
                    position=Point(self.p_ref[0], self.p_ref[1], 0),
                    orientation=Quaternion(*self.q_ref),
                ),
            )
        )

        # Publish adaptation (Y*theta) for plotting
        adaptation_msg = WrenchStamped()
        adaptation_msg.header.frame_id = "wamv/base_link"
        adaptation_msg.wrench.force.x = (self.dist_est + self.drag_effort)[0]
        adaptation_msg.wrench.force.y = (self.dist_est + self.drag_effort)[1]
        adaptation_msg.wrench.torque.z = (self.dist_est + self.drag_effort)[2]
        self.adaptation_pub.publish(adaptation_msg)

        try:
            self.theta_pub.publish(Float64MultiArray(data=self.drag_est))
        except BaseException:
            traceback.print_exc()

    def increment_reference(self) -> None:
        """
        Steps the model reference (trajectory to track) by one self.timestep.
        """
        # Frame management quantities
        R_ref = trns.quaternion_matrix(self.q_ref)[:3, :3]
        y_ref = trns.euler_from_quaternion(self.q_ref)[2]

        # Convert body PD gains to world frame
        kp = R_ref.dot(self.kp_body).dot(R_ref.T)
        kd = R_ref.dot(self.kd_body).dot(R_ref.T)

        # Compute error components (desired - reference), using "smartyaw"
        p_err = self.p_des - self.p_ref
        v_err = -self.v_ref
        w_err = -self.w_ref
        if npl.norm(p_err) <= self.heading_threshold:
            q_err = trns.quaternion_multiply(
                self.q_des, trns.quaternion_inverse(self.q_ref)
            )
        else:
            q_direct = trns.quaternion_from_euler(
                0, 0, np.angle(p_err[0] + (1j * p_err[1]))
            )
            q_err = trns.quaternion_multiply(
                q_direct, trns.quaternion_inverse(self.q_ref)
            )
        y_err = trns.euler_from_quaternion(q_err)[2]

        # Combine error components into error vectors
        err = np.concatenate((p_err, [y_err]))
        errdot = np.concatenate((v_err, [w_err]))
        wrench = (kp.dot(err)) + (kd.dot(errdot))

        # Compute world frame thruster matrix (B) from thruster geometry, and then map wrench to thrusts
        B = np.concatenate(
            (R_ref.dot(self.thruster_directions.T), R_ref.dot(self.lever_arms.T))
        )
        B_3dof = np.concatenate((B[:2, :], [B[5, :]]))
        command = self.thruster_mapper(wrench, B_3dof)
        wrench_saturated = B.dot(command)

        # Use model drag to find drag force on virtual boat
        twist_body = R_ref.T.dot(np.concatenate((self.v_ref, [self.w_ref])))
        D_body = np.zeros_like(twist_body)
        for i, v in enumerate(twist_body):
            if v >= 0:
                D_body[i] = self.D_body_positive[i]
            else:
                D_body[i] = self.D_body_negative[i]
        drag_ref = R_ref.dot(D_body * twist_body * abs(twist_body))

        # Step forward the dynamics of the virtual boat
        self.a_ref = (wrench_saturated[:2] - drag_ref[:2]) / self.mass_ref
        self.aa_ref = (wrench_saturated[5] - drag_ref[2]) / self.inertia_ref
        self.p_ref = self.p_ref + (self.v_ref * self.timestep)
        self.q_ref = trns.quaternion_from_euler(
            0, 0, y_ref + (self.w_ref * self.timestep)
        )
        self.v_ref = self.v_ref + (self.a_ref * self.timestep)
        self.w_ref = self.w_ref + (self.aa_ref * self.timestep)

    def set_learning(self, str_msg: String) -> None:
        """
        Sets learning status based on current wrench.

        Serves as the callback for the subscriber to the
        /wrench/selected node.

        Args:
            str_msg: String - The message passed to the callback
              by the subscriber.
        """
        learn = str_msg.data in self.LEARN_WRENCHES
        if learn == self.learn:
            return
        self.learn = learn
        if self.learn:
            rospy.loginfo("MRAC: learning")
        else:
            rospy.loginfo("MRAC: reset and not learning")
            self.dist_est = np.zeros(3)
            self.drag_est = np.zeros(5)

    def thruster_mapper(self, wrench, B):
        """
        Virtual thruster mapper used by the model reference.
        Math-wise, it is the same as the thruster mapper used by
        the actual boat.
        """
        # Get minimum energy mapping using pseudoinverse
        command = npl.pinv(B).dot(wrench)

        # Scale back commands to maximums
        command_max = np.max(np.abs(command))
        if command_max > self.thrust_max:
            command = (self.thrust_max / command_max) * command

        return command


# ROS
if __name__ == "__main__":
    rospy.init_node("controller")
    controller = MRAC_Controller()
