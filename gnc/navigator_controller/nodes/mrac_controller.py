#!/usr/bin/python
from __future__ import division
import numpy as np
import numpy.linalg as npl
import tf.transformations as trns

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, WrenchStamped, PoseStamped, Quaternion, Pose
from std_msgs.msg import Header
from uf_common.msg import PoseTwistStamped
from std_msgs.msg import Bool


class MRAC_Controller:

    def __init__(self):
        '''
        Set-up.

        '''
        #### TUNABLES
        # Proportional gains, body frame
        self.kp_body = np.diag([1000, 1000, 5600])
        # Derivative gains, body frame
        self.kd_body = np.diag([1200, 1200, 6000])
        # Disturbance adaptation rates, world frame
        self.ki = np.array([0.1, 0.1, 0.1])
        # Drag adaptation rates, world frame
        self.kg = 5 * np.array([1, 1, 1, 1, 1])
        # Initial disturbance estimate
        self.dist_est = np.array([0, 0, 0])
        # Initial drag estimate
        self.drag_est = np.array([0, 0, 0, 0, 0])  # [d1 d2 Lc1 Lc2 Lr]
        # Limits on disturbance estimate
        self.dist_limit = np.array([200, 200, 200])
        # Limits on drag estimate effort
        self.drag_limit = np.array([1000, 1000, 1000])
        # User-imposed speed limit
        self.vel_max_body_positive = np.array([1.1,  0.45, 0.19])  # [m/s, m/s, rad/s]
        self.vel_max_body_negative = np.array([0.68, 0.45, 0.19])  # [m/s, m/s, rad/s]
        # "Smart heading" threshold
        self.heading_threshold = 500  # m
        # Only use PD controller
        self.only_PD = False
        self.learning_radius = 10  # m
        # Use external trajectory generator instead of internal reference generator
        # Subscribes to /trajectory if using external, else just takes /waypoint for the end goal
        self.use_external_tgen = True

        #### REFERENCE MODEL (note that this is not the adaptively estimated TRUE model; rather,
        #                     these parameters will govern the trajectory we want to achieve).
        self.mass_ref = 500  # kg, determined to be larger than boat in practice due to water mass
        self.inertia_ref = 400  # kg*m^2, determined to be larger than boat in practice due to water mass
        self.thrust_max = 220  # N
        self.thruster_positions = np.array([[-1.9000,  1.0000, -0.0123],
                                            [-1.9000, -1.0000, -0.0123],
                                            [ 1.6000,  0.6000, -0.0123],
                                            [ 1.6000, -0.6000, -0.0123]]) # back-left, back-right, front-left front-right, m
        self.thruster_directions = np.array([[ 0.7071,  0.7071,  0.0000],
                                             [ 0.7071, -0.7071,  0.0000],
                                             [ 0.7071, -0.7071,  0.0000],
                                             [ 0.7071,  0.7071,  0.0000]]) # back-left, back-right, front-left front-right
        self.lever_arms = np.cross(self.thruster_positions, self.thruster_directions)
        self.B_body = np.concatenate((self.thruster_directions.T, self.lever_arms.T))
        self.Fx_max_body = self.B_body.dot(self.thrust_max * np.array([1, 1, 1, 1]))
        self.Fy_max_body = self.B_body.dot(self.thrust_max * np.array([1, -1, -1, 1]))
        self.Mz_max_body = self.B_body.dot(self.thrust_max * np.array([-1, 1, -1, 1]))
        self.D_body_positive = abs(np.array([self.Fx_max_body[0], self.Fy_max_body[1], self.Mz_max_body[5]])) / self.vel_max_body_positive**2
        self.D_body_negative = abs(np.array([self.Fx_max_body[0], self.Fy_max_body[1], self.Mz_max_body[5]])) / self.vel_max_body_negative**2

        #### BASIC INITIALIZATIONS
        # Position waypoint
        self.p_des = np.array([0, 0])
        # Orientation waypoint
        self.q_des = np.array([1, 0, 0, 0])
        # Total distance that will be traversed for this waypoint
        self.traversal = 0
        # Reference states
        self.p_ref = None #np.array([0, 0])
        self.v_ref = np.array([0, 0])
        self.q_ref = np.array([0, 0, 0, 0])
        self.w_ref = 0
        self.a_ref = np.array([0, 0])
        self.aa_ref = 0
        # Messages
        self.last_odom = None
        self.learn = False

        #### ROS
        # Time since last controller call = 1/controller_call_frequency
        self.timestep = 0.02
        # For unpacking ROS messages later on
        self.position = np.zeros(2)
        self.orientation = np.array([1, 0, 0, 0])
        self.lin_vel = np.zeros(2)
        self.ang_vel = 0
        self.state = Odometry()
        # Subscribers
        if self.use_external_tgen:
            rospy.Subscriber("/trajectory", PoseTwistStamped, self.set_traj)
        else:
            rospy.Subscriber("/waypoint", PoseStamped, self.set_waypoint)
        rospy.Subscriber("/odom", Odometry, self.get_command)
        rospy.Subscriber('/learn', Bool, self.toggle_learning)
        # Publishers
        self.wrench_pub = rospy.Publisher("/wrench/autonomous", WrenchStamped, queue_size=0)
        self.pose_ref_pub = rospy.Publisher("pose_ref", PoseStamped, queue_size=0)
        self.adaptation_pub = rospy.Publisher("adaptation", WrenchStamped, queue_size=0)

        rospy.spin()

    def set_traj(self, msg):
        '''
        Sets instantaneous reference state.
        Convert twist to world frame for controller math.
        '''
        self.p_ref = np.array([msg.posetwist.pose.position.x, msg.posetwist.pose.position.y])
        self.q_ref = np.array([msg.posetwist.pose.orientation.x, msg.posetwist.pose.orientation.y, msg.posetwist.pose.orientation.z, msg.posetwist.pose.orientation.w])

        R = trns.quaternion_matrix(self.q_ref)[:3, :3]

        self.v_ref = R.dot(np.array([msg.posetwist.twist.linear.x, msg.posetwist.twist.linear.y, msg.posetwist.twist.linear.z]))[:2]
        self.w_ref = R.dot(np.array([msg.posetwist.twist.angular.x, msg.posetwist.twist.angular.y, msg.posetwist.twist.angular.z]))[2]

        self.a_ref = R.dot(np.array([msg.posetwist.acceleration.linear.x, msg.posetwist.acceleration.linear.y, msg.posetwist.acceleration.linear.z]))[:2]
        self.aa_ref = R.dot(np.array([msg.posetwist.acceleration.angular.x, msg.posetwist.acceleration.angular.y, msg.posetwist.acceleration.angular.z]))[2]

    def set_waypoint(self, msg):
        '''
        Sets desired waypoint ("GO HERE AND STAY").
        Resets reference model to current state (i.e. resets trajectory generation).
        '''
        self.p_des = np.array([msg.pose.position.x, msg.pose.position.y])
        self.q_des = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        self.traversal = npl.norm(self.p_des - self.position)
        self.p_ref = self.position
        self.v_ref = self.lin_vel
        self.q_ref = self.orientation
        self.w_ref = self.ang_vel
        self.a_ref = np.array([0, 0])
        self.aa_ref = 0

    def get_command(self, msg):
        '''
        Publishes the wrench for this instant.
        (Note: this is called get_command because it used to be used for
        getting the actual thruster values, but now it is only being
        used for getting the wrench which is then later mapped elsewhere).

        '''
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
        self.orientation = np.array([orientation.x, orientation.y, orientation.z, orientation.w])

        # Frame management quantities
        R = np.eye(3)
        R[:2, :2] = trns.quaternion_matrix(self.orientation)[:2, :2]
        y = trns.euler_from_quaternion(self.orientation)[2]

        # More ROS unpacking, converting body frame twist to world frame lin_vel and ang_vel
        self.lin_vel = R.dot(np.array([lin_vel_body.x, lin_vel_body.y, lin_vel_body.z]))[:2]
        self.ang_vel = R.dot(np.array([ang_vel_body.x, ang_vel_body.y, ang_vel_body.z]))[2]

        # Convert body PD gains to world frame
        kp = R.dot(self.kp_body).dot(R.T)
        kd = R.dot(self.kd_body).dot(R.T)

        # Compute error components (reference - true)
        p_err = self.p_ref - self.position
        y_err = trns.euler_from_quaternion(trns.quaternion_multiply(self.q_ref, trns.quaternion_inverse(self.orientation)))[2]
        v_err = self.v_ref - self.lin_vel
        w_err = self.w_ref - self.ang_vel

        # Combine error components into error vectors
        err = np.concatenate((p_err, [y_err]))
        errdot = np.concatenate((v_err, [w_err]))

        # Compute "anticipation" feedforward based on the boat's inertia
        inertial_feedforward = np.concatenate((self.a_ref, [self.aa_ref])) * [self.mass_ref, self.mass_ref, self.inertia_ref]

        # Compute the "learning" matrix
        drag_regressor = np.array([[               self.lin_vel[0]*np.cos(y)**2 + self.lin_vel[1]*np.sin(y)*np.cos(y),    self.lin_vel[0]/2 - (self.lin_vel[0]*np.cos(2*y))/2 - (self.lin_vel[1]*np.sin(2*y))/2,                             -self.ang_vel*np.sin(y),                               -self.ang_vel*np.cos(y),          0],
                                   [self.lin_vel[1]/2 - (self.lin_vel[1]*np.cos(2*y))/2 + (self.lin_vel[0]*np.sin(2*y))/2,                   self.lin_vel[1]*np.cos(y)**2 - self.lin_vel[0]*np.cos(y)*np.sin(y),                              self.ang_vel*np.cos(y),                               -self.ang_vel*np.sin(y),          0],
                                   [                                                                     0,                                                                         0,    self.lin_vel[1]*np.cos(y) - self.lin_vel[0]*np.sin(y),    - self.lin_vel[0]*np.cos(y) - self.lin_vel[1]*np.sin(y),    self.ang_vel]])

        # wrench = PD + feedforward + I + adaptation
        if self.only_PD:
            wrench = (kp.dot(err)) + (kd.dot(errdot))
            self.drag_effort = [0, 0, 0]
            self.dist_est = [0, 0, 0]
        else:
            self.drag_effort = np.clip(drag_regressor.dot(self.drag_est), -self.drag_limit, self.drag_limit)
            wrench = (kp.dot(err)) + (kd.dot(errdot)) + inertial_feedforward + self.dist_est + self.drag_effort
            # Update disturbance estimate, drag estimates
            if self.learn and (npl.norm(p_err) < self.learning_radius):
                self.dist_est = np.clip(self.dist_est + (self.ki * err * self.timestep), -self.dist_limit, self.dist_limit)
                self.drag_est = self.drag_est + (self.kg * (drag_regressor.T.dot(err + errdot)) * self.timestep)

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
        wrench_msg.header.frame_id = "/base_link"
        wrench_msg.wrench.force.x = wrench_body[0]
        wrench_msg.wrench.force.y = wrench_body[1]
        wrench_msg.wrench.torque.z = wrench_body[2]
        self.wrench_pub.publish(wrench_msg)

        # Publish reference pose for examination
        self.pose_ref_pub.publish(PoseStamped(
             header=Header(
                 frame_id='/enu',
                 stamp=msg.header.stamp,
             ),
             pose=Pose(
                 position=Point(self.p_ref[0], self.p_ref[1], 0),
                 orientation=Quaternion(*self.q_ref),
             ),
        ))

        # Publish adaptation (Y*theta) for plotting
        adaptation_msg = WrenchStamped()
        adaptation_msg.header.frame_id = "/base_link"
        adaptation_msg.wrench.force.x = (self.dist_est + self.drag_effort)[0]
        adaptation_msg.wrench.force.y = (self.dist_est + self.drag_effort)[1]
        adaptation_msg.wrench.torque.z = (self.dist_est + self.drag_effort)[2]
        self.adaptation_pub.publish(adaptation_msg)

    def increment_reference(self):
        '''
        Steps the model reference (trajectory to track) by one self.timestep.

        '''
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
            q_err = trns.quaternion_multiply(self.q_des, trns.quaternion_inverse(self.q_ref))
        else:
            q_direct = trns.quaternion_from_euler(0, 0, np.angle(p_err[0] + (1j * p_err[1])))
            q_err = trns.quaternion_multiply(q_direct, trns.quaternion_inverse(self.q_ref))
        y_err = trns.euler_from_quaternion(q_err)[2]

        # Combine error components into error vectors
        err = np.concatenate((p_err, [y_err]))
        errdot = np.concatenate((v_err, [w_err]))
        wrench = (kp.dot(err)) + (kd.dot(errdot))

        # Compute world frame thruster matrix (B) from thruster geometry, and then map wrench to thrusts
        B = np.concatenate((R_ref.dot(self.thruster_directions.T), R_ref.dot(self.lever_arms.T)))
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
        self.q_ref = trns.quaternion_from_euler(0, 0, y_ref + (self.w_ref * self.timestep))
        self.v_ref = self.v_ref + (self.a_ref * self.timestep)
        self.w_ref = self.w_ref + (self.aa_ref * self.timestep)

    def toggle_learning(self, bool_msg):
        '''
        Callback for the /mrac_learn topic. Sets
        self.learn to True or False depending
        on the Bool message.

        '''
        self.learn = bool_msg.data
        if self.learn:
            print("MRAC controller is learning.")
        else:
            print("MRAC controller reset and stopped learning.")
            self.dist_est = np.zeros(3)
            self.drag_est = np.zeros(5)

    def thruster_mapper(self, wrench, B):
        '''
        Virtual thruster mapper used by the model reference.
        Math-wise, it is the same as the thruster mapper used by
        the actual boat.

        '''
        # Get minimum energy mapping using pseudoinverse
        command = npl.pinv(B).dot(wrench)

        # Scale back commands to maximums
        command_max = np.max(np.abs(command))
        if command_max > self.thrust_max:
            command = (self.thrust_max / command_max) * command

        return command


# ROS
if __name__ == '__main__':

    rospy.init_node("controller")

    controller = MRAC_Controller()
