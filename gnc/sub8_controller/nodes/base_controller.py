#! /usr/bin/env python
from tf import transformations
import threading
import geometry_msgs.msg as geometry_msgs
import rospy
import numpy as np
from sub8_msgs.msg import Trajectory, Waypoint
from collections import deque
from scipy import linalg
from sub8_simulation.srv import SimSetPose
import nav_msgs.msg as nav_msgs
import sub8_ros_tools as sub8_utils
from dynamic_reconfigure.server import Server
from sub8_controller.cfg import GainConfig  # Configuration file

lock = threading.Lock()


class Controller(object):
    def __init__(self, odom_topic='/truth/odom', sampling_period=0.1, control_period=None,
                 history_length=100, waypoint_epsilon=1):
        '''
        Note:
            Sampling period and control_period are specified in seconds
            ALL units are MKS
        Arguments:
            waypoint_epsilon: If l2 norm of state error to the current waypoint is less than epsilon,
             the controller will consider the waypoint 'achieved'
        TODO:
            - Plotting
            - Adaptive controller (Estimate m!)
        '''
        rospy.init_node('adaptive_controller_EML6350')
        self.state_variables = ['position', 'linear_vel', 'orientation_q', 'angular_vel']

        self.dynamic_reconfig_srv = Server(GainConfig, self.dynamic_reconfig)

        # Ros setup
        self.wrench_pub = rospy.Publisher('/wrench', geometry_msgs.WrenchStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber(odom_topic, nav_msgs.Odometry, self.odom_cb, queue_size=1)
        self.trajectory_sub = rospy.Subscriber('/trajectory', Trajectory, self.trajectory_cb, queue_size=1)

        # Initialize timers and history trackers
        self.last_sample = rospy.Time.now()
        self.start_time = rospy.Time.now()
        self.state_history = {
            'position': deque(),
            'linear_vel': deque(),
            'orientation_q': deque(),
            'angular_vel': deque(),
        }
        self.target_trajectory = None
        self.history_length = history_length
        self.sampling_period = rospy.Duration(sampling_period)
        self.waypoint_epsilon = waypoint_epsilon
        self.current_state = None

        # If control period is unspecified, do something that makes sense
        if control_period is None:
            self.control_period = self.sampling_period
        else:
            self.control_period = rospy.Duration(control_period)

        # rospy.wait_for_service('sim/vehicle/set_pose', timeout=5.0)
        self.set_pose = rospy.ServiceProxy('sim/vehicle/set_pose', SimSetPose)

    def start(self):
        self.control_manager = rospy.Timer(self.control_period, self.run)

    def stop(self):
        self.control_manager.shutdown()

    @sub8_utils.thread_lock(lock)
    def dynamic_reconfig(self, config, level):
        rospy.logwarn("Reconfiguring contoller gains at level {}".format(level))
        self.cfg = config
        return config

    def trajectory_cb(self, msg):
        '''
        Message structure:
            - header: frame, timestamp
            - waypoint[]:
                - pose
                - twist
        '''
        print msg
        self.state_history = {
            'position': deque(),
            'linear_vel': deque(),
            'orientation_q': deque(),
            'angular_vel': deque(),
        }
        rospy.logwarn("Recieving trajectory!")
        self.target_trajectory = {
            'position': deque(),
            'linear_vel': deque(),
            'orientation_q': deque(),
            'angular_vel': deque(),
        }
        for struct_waypoint in msg.trajectory:
            # Deserialize to numpy
            pose, twist = sub8_utils.posetwist_to_numpy(struct_waypoint)
            (position, orientation_q), (linear_vel, angular_vel) = pose, twist

            # TODO: Less repeated code
            self.target_trajectory['position'].append(position)
            self.target_trajectory['linear_vel'].append(linear_vel)
            self.target_trajectory['orientation_q'].append(orientation_q)
            self.target_trajectory['angular_vel'].append(angular_vel)

    def odom_cb(self, msg):
        '''
        Message structure defined in ROS specification
            [1] http://docs.ros.org/jade/api/nav_msgs/html/msg/Odometry.html
        '''

        # Using ropsy.Time.now() so we can use simulated accelerated time (not CPU time)
        pose, twist, pose_cov, twist_cov = sub8_utils.odometry_to_numpy(msg)
        (position, orientation_q), (linear_vel, angular_vel) = pose, twist

        self.current_state = {
            'position': position,
            'linear_vel': linear_vel,
            'orientation_q': orientation_q,
            'angular_vel': angular_vel,
        }
        time_of = rospy.Time.now()
        if (time_of - self.last_sample) < self.sampling_period:
            return
        else:
            self.last_sample = time_of

        # For now, we'll ignore pose and twist covariance (Implementing for controls final project)

        # Don't include states that are "too old"
        for state_variable in self.state_variables:
            if len(self.state_history[state_variable]) > self.history_length:
                self.state_history[state_variable].popleft()

        self.state_history['position'].append(position)
        self.state_history['linear_vel'].append(linear_vel)
        self.state_history['orientation_q'].append(orientation_q)
        self.state_history['angular_vel'].append(angular_vel)

        # Hold at last target state
        if self.target_trajectory is None:
            return
        if len(self.target_trajectory['position']) > 1:
            p_error = np.linalg.norm(position - self.target_trajectory['position'][0])
            v_error = np.linalg.norm(linear_vel - self.target_trajectory['linear_vel'][0])

            if p_error + v_error < self.waypoint_epsilon:
                rospy.logwarn("Waypoint achieved!")
                for state in self.state_variables:

                    self.target_trajectory[state].popleft()  # Remove a target state!

    def send_wrench(self, force, torque):
        '''
        Specify wrench in Newtons, Newton-meters
            (Are you supposed to say Newtons-meter? Newtons-Meters?)
        '''
        wrench_msg = geometry_msgs.WrenchStamped(
            header=sub8_utils.make_header(),
            wrench=geometry_msgs.Wrench(
                force=geometry_msgs.Vector3(*force),
                torque=geometry_msgs.Vector3(*torque)
            )
        )
        self.wrench_pub.publish(wrench_msg)

    def orientation_error(self, q_a, q_b):
        '''Error from q_a to q_b
        '''
        M_a = transformations.quaternion_matrix(q_a)[:3, :3]
        M_b = transformations.quaternion_matrix(q_b)[:3, :3]
        error_matrix = M_b.dot(np.transpose(M_a))
        try:
            lie_alg_error = np.real(linalg.logm(error_matrix))
        except Exception:
            # We get an Exception("Internal Inconsistency") when the error is zero
            # No error!
            return np.array([0.0, 0.0, 0.0])

        angle_axis = sub8_utils.deskew(lie_alg_error)
        assert np.linalg.norm(angle_axis) < (2 * np.pi) + 0.01, "uh-oh, unnormalized {}".format(angle_axis)
        return angle_axis

    @sub8_utils.thread_lock(lock)
    def run(self, time):
        '''
            Do the bookkeeping before calling "execute"
        '''
        # Address no trajectory existing
        if self.target_trajectory is None:
            return
        if len(self.state_history['position']) < 2:
            return

        # #### Setup target/current state stuff ##### #

        # Last recieved state
        current_position = self.current_state['position']
        current_linear_vel = self.current_state['linear_vel']
        current_orientation_q = self.current_state['orientation_q']
        current_angular_vel = self.current_state['angular_vel']

        # First waypoint to go to (stack, fifo)
        target_position = self.target_trajectory['position'][0]
        target_linear_vel = self.target_trajectory['linear_vel'][0]
        target_orientation_q = self.target_trajectory['orientation_q'][0]
        target_angular_vel = self.target_trajectory['angular_vel'][0]

        # ##### Setup Done ##### #

        self.execute((current_position, current_linear_vel, current_orientation_q, current_angular_vel),
                     (target_position, target_linear_vel, target_orientation_q, target_angular_vel), self.state_history)

    def execute(self, (current_position, current_linear_vel, current_orientation_q, current_angular_vel),
                (target_position, target_linear_vel, target_orientation_q, target_angular_vel), state_history):
        '''This is the default 'execute' implementation
           Your code should implement this method
        '''
        orientation_est = transformations.quaternion_matrix(current_orientation_q)[:3, :3]

        error_position = target_position - current_position
        error_linear_vel = target_linear_vel - current_linear_vel
        error_orientation = self.orientation_error(current_orientation_q, target_orientation_q)
        error_angular_vel = target_angular_vel - current_angular_vel

        world_force = (self.cfg['kp_trans'] * error_position) + (self.cfg['kd_trans'] * error_linear_vel)

        # Feed forward gravity
        world_force += (self.cfg['sub_mass'] * 9.81) * np.array([0.0, 0.0, 1.0])

        # Rotate force into body frame for mapper to work
        body_force = orientation_est.dot(world_force)

        # ORIENTATION CONTROL (NOT FOR PROJECT!!!!)
        # I did do a Lyapunov-based proof of this section

        # Might be negative of error_angular_vel
        world_torque = (self.cfg['kp_angle'] * error_orientation) + (self.cfg['kd_angle'] * error_angular_vel)
        body_torque = orientation_est.dot(-world_torque)

        # Send the wrench to the thruster mapper!
        self.send_wrench(body_force, body_torque)

if __name__ == '__main__':
    controller = Controller(odom_topic='/odom', sampling_period=0.01, control_period=0.01,
                            history_length=100, waypoint_epsilon=1)
    controller.start()
    rospy.spin()
