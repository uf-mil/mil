#!/usr/bin/python
'''
Model-Reference Adaptive Controller

###Purpose
###How to use
###Conventions, units, and array orders
###Explanation of controller
###References

'''
from __future__ import division
import rospy
import numpy as np
import numpy.linalg as npl
import tf.transformations as trns
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, WrenchStamped, PoseStamped, Quaternion, Pose
from std_msgs.msg import Header

class MRAC_Controller:

    def __init__(self):
        '''
        ###Explain what is assigned here

        '''
        #### TUNABLES
        # Proportional gains, body frame
        self.kp_body = np.diag([100, 100, 100])
        # Derivative gains, body frame
        self.kd_body = np.diag([100, 100, 100])
        # Disturbance adaptation rates, world frame
        self.ki = np.array([0.1, 0.1, 0.1])
        # Drag adaptation rates, world frame
        self.kg = np.array([3, 3, 3, 3, 3])
        # Initial disturbance estimate
        self.dist_est = np.array([0, 0, 0])
        # Initial drag estimate
        self.drag_est = np.array([30, 100, -2, -10, 100])  # [d1 d2 Lc1 Lc2 Lr]
        # User-imposed speed limit
        self.vel_max_body = np.array([3, 0.1, 1])
        # "Smart heading" threshold
        self.heading_threshold = 2

        #### BOAT MODEL
        self.mass = 42
        self.inertia = 18.5
        self.thrust_max = 60
        self.thruster_positions = np.array([[-0.5215,  0.3048, -0.0123],
                                            [-0.5215, -0.3048, -0.0123],
                                            [ 0.5215, -0.3048, -0.0123],
                                            [ 0.5215,  0.3048, -0.0123]]) # back-left, back-right, front-right front-left
        self.thruster_directions = np.array([[ 0.7071,  0.7071,  0.0000],
                                             [ 0.7071, -0.7071,  0.0000],
                                             [ 0.7071,  0.7071,  0.0000],
                                             [ 0.7071, -0.7071,  0.0000]]) # back-left, back-right, front-right front-left
        self.lever_arms = np.cross(self.thruster_positions, self.thruster_directions)
        self.B_body = np.concatenate((self.thruster_directions.T, self.lever_arms.T))
        self.Fx_max_body = self.B_body.dot(self.thrust_max * np.array([1, 1, 1, 1]))
        self.Fy_max_body = self.B_body.dot(self.thrust_max * np.array([1, -1, 1, -1]))
        self.Mz_max_body = self.B_body.dot(self.thrust_max * np.array([-1, 1, 1, -1]))
        self.D_body = abs(np.array([self.Fx_max_body[0], self.Fy_max_body[1], self.Mz_max_body[5]])) / self.vel_max_body**2

        #### MEMORY
        # Time since last controller call = 1/controller_frequency
        self.timestep = 0.01
        # Position waypoint
        self.p_des = np.array([0, 0])
        # Orientation waypoint
        self.q_des = np.array([1, 0, 0, 0])
        # Total distance that will be traversed for this waypoint
        self.traversal = 0
        # Reference states
        self.p_ref = np.array([0, 0])
        self.v_ref = np.array([0, 0])
        self.q_ref = np.array([0, 0, 0, 0])
        self.w_ref = 0
        self.a_ref = np.array([0, 0])
        self.aa_ref = 0

        #### ROS
        # For unpacking ROS messages later on
        self.position = np.zeros(2)
        self.orientation = np.array([1, 0, 0, 0])
        self.lin_vel = np.zeros(2)
        self.ang_vel = 0
        self.state = Odometry()
        # ayeeee subscrizzled
        rospy.Subscriber("/desired_pose", Point, self.set_waypoint)
        rospy.Subscriber("/odom", Odometry, self.get_command)
        self.last_odom = None
        self.wrench_pub = rospy.Publisher("/wrench/autonomous", WrenchStamped, queue_size=0)
        self.pose_ref_pub = rospy.Publisher("pose_ref", PoseStamped, queue_size=0)
        rospy.spin()


    def set_waypoint(self, msg):
        '''
        ###Inputs and outputs
        ###Explain what happens here

        '''
        # Set desired to user specified waypoint
        self.p_des = np.array([msg.x, msg.y])
        self.q_des = trns.quaternion_from_euler(0, 0, np.deg2rad(msg.z))
        self.traversal = npl.norm(self.p_des - self.position)
        self.p_ref = self.position
        self.v_ref = self.lin_vel
        self.q_ref = self.orientation
        self.w_ref = self.ang_vel
        self.a_ref = np.array([0, 0])
        self.aa_ref = 0


    def get_command(self, msg):
        '''
        ###Inputs and outputs
        ###Explain what happens here

        '''

        # compute timestep from interval between this message's stamp and last's
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
        R = trns.quaternion_matrix(self.orientation)[:3, :3]
        y = trns.euler_from_quaternion(self.orientation)[2]

        # More ROS unpacking, converting body frame twist to world frame lin_vel and ang_vel
        self.lin_vel = R.dot(np.array([lin_vel_body.x, lin_vel_body.y, lin_vel_body.z]))[:2]
        self.ang_vel = R.dot(np.array([ang_vel_body.x, ang_vel_body.y, ang_vel_body.z]))[2]

        # Convert body PD gains to world frame
        kp = np.diag(R.dot(self.kp_body).dot(R.T))
        kd = np.diag(R.dot(self.kd_body).dot(R.T))

        # Compute error components (reference - true)
        p_err = self.p_ref - self.position
        y_err = trns.euler_from_quaternion(trns.quaternion_multiply(self.q_ref, trns.quaternion_inverse(self.orientation)))[2]
        v_err = self.v_ref - self.lin_vel
        w_err = self.w_ref - self.ang_vel

        # Combine error components into error vectors
        err = np.concatenate((p_err, [y_err]))
        errdot = np.concatenate((v_err, [w_err]))

        # Compute "anticipation" feedforward based on the boat's inertia
        inertial_feedforward = np.concatenate((self.a_ref, [self.aa_ref])) * [self.mass, self.mass, self.inertia]

        # Compute the "learning" matrix
        drag_regressor = np.array([[               self.lin_vel[0]*np.cos(y)**2 + self.lin_vel[1]*np.sin(y)*np.cos(y),    self.lin_vel[0]/2 - (self.lin_vel[0]*np.cos(2*y))/2 - (self.lin_vel[1]*np.sin(2*y))/2,                             -self.ang_vel*np.sin(y),                               -self.ang_vel*np.cos(y),          0],
                                   [self.lin_vel[1]/2 - (self.lin_vel[1]*np.cos(2*y))/2 + (self.lin_vel[0]*np.sin(2*y))/2,                   self.lin_vel[1]*np.cos(y)**2 - self.lin_vel[0]*np.cos(y)*np.sin(y),                              self.ang_vel*np.cos(y),                               -self.ang_vel*np.sin(y),          0],
                                   [                                                                     0,                                                                         0,    self.lin_vel[1]*np.cos(y) - self.lin_vel[0]*np.sin(y),    - self.lin_vel[0]*np.cos(y) - self.lin_vel[1]*np.sin(y),    self.ang_vel]])

        # wrench = PD + feedforward + I + adaptation
        wrench = (kp * err) + (kd * errdot) + inertial_feedforward + self.dist_est + (drag_regressor.dot(self.drag_est))

        # Update disturbance estimate, drag estimates, and model reference for the next call
        self.dist_est = self.dist_est + (self.ki * err * self.timestep)
        self.drag_est = self.drag_est + (self.kg * (drag_regressor.T.dot(err + errdot)) * self.timestep)
        self.increment_reference()

        # NOT NEEDED SINCE WE ARE USING AZIDRIVE
        # # Compute world frame thruster matrix (B) from thruster geometry, and then map wrench to thrusts
        # B = np.concatenate((R.dot(self.thruster_directions.T), R.dot(self.lever_arms.T)))
        # B_3dof = np.concatenate((B[:2, :], [B[5, :]]))
        # command = self.thruster_mapper(wrench, B_3dof)

        # Give wrench to ROS
        to_send = WrenchStamped()
        to_send.wrench.force.x = wrench[0]
        to_send.wrench.force.y = wrench[1]
        to_send.wrench.torque.z = wrench[2]
        self.wrench_pub.publish(to_send)

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


    def increment_reference(self):
        '''
        ###Inputs and outputs
        ###Explain what happens here

        '''
        # Frame management quantities
        R_ref = trns.quaternion_matrix(self.q_ref)[:3, :3]
        y_ref = trns.euler_from_quaternion(self.q_ref)[2]

        # Convert body PD gains to world frame
        kp = np.diag(R_ref.dot(self.kp_body).dot(R_ref.T))
        kd = np.diag(R_ref.dot(self.kd_body).dot(R_ref.T))

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
        wrench = (kp * err) + (kd * errdot)

        # Compute world frame thruster matrix (B) from thruster geometry, and then map wrench to thrusts
        B = np.concatenate((R_ref.dot(self.thruster_directions.T), R_ref.dot(self.lever_arms.T)))
        B_3dof = np.concatenate((B[:2, :], [B[5, :]]))
        command = self.thruster_mapper(wrench, B_3dof)
        wrench_saturated = B.dot(command)

        # Use model drag to find drag force on virtual boat
        twist_body = R_ref.T.dot(np.concatenate((self.v_ref, [self.w_ref])))
        drag_ref = R_ref.dot(self.D_body * twist_body * abs(twist_body))

        # Step forward the dynamics of the virtual boat
        self.a_ref = (wrench_saturated[:2] - drag_ref[:2]) / self.mass
        self.aa_ref = (wrench_saturated[5] - drag_ref[2]) / self.inertia
        self.p_ref = self.p_ref + (self.v_ref * self.timestep)
        self.q_ref = trns.quaternion_from_euler(0, 0, y_ref + (self.w_ref * self.timestep))
        self.v_ref = self.v_ref + (self.a_ref * self.timestep)
        self.w_ref = self.w_ref + (self.aa_ref * self.timestep)


    def thruster_mapper(self, wrench, B):
        '''
        ###Inputs and outputs
        ###Explain what happens here

        '''
        # Get minimum energy mapping using pseudoinverse
        command = npl.pinv(B).dot(wrench)

        # Scale back commands to maximums
        command_max = np.max(np.abs(command))
        if command_max > self.thrust_max:
            command = (self.thrust_max / command_max) * command

        return command


# ROS ME BABY
if __name__ == '__main__':

    rospy.init_node("controller")

    controller = MRAC_Controller()
