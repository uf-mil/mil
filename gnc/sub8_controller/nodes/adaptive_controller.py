#! /usr/bin/env python
import copy
from collections import deque
from base_controller import Controller
import matplotlib.pyplot as plt
import numpy as np
import rospy
from tf import transformations


class AdaptiveController(Controller):
    def __init__(self, odom_topic='/truth/odom', sampling_period=0.1, control_period=None,
                 history_length=100, waypoint_epsilon=1, plot=False):

        super(self.__class__, self).__init__(odom_topic, sampling_period, control_period,
                                             history_length, waypoint_epsilon)

        self.alpha = np.diag([1e-4, 1e-4, 1e-4])
        self.Kp_trans = np.diag([15.0, 15.0, 45.0])
        self.Kd_trans = np.diag([15.0, 15.0, 15.0])

        self.q_history = []

        self.theta_hat_history = []
        self.wrench_history = []
        self.wrench_times = []
        self.error_history = []
        if plot:
            rospy.on_shutdown(self.end)

        self.adaptive_queue = deque()
        self.gamma = np.diag([1e-2, 1e-3])
        self.theta_hat = np.array([
            99.1,
            0.2
        ], dtype=np.float32
        )
        self.stopped = False

    def end(self):
        ropsy.logwarn('Plotting results')
        # wrench_norms = np.array(map(np.linalg.norm, self.wrench_history))
        self.target_trajectory = None

        wrench_hist = np.vstack(
            map(lambda o: np.clip(o, [-250, -250, -250], [250, 250, 250]), self.wrench_history)
        )

        q_hist = np.array(self.q_history[:len(wrench_hist)])
        errors = np.array(self.error_history[:len(wrench_hist)])
        # error_norms = np.array(map(np.linalg.norm, self.error_history[:len(wrench_hist)]))
        times = np.array(self.wrench_times[:len(wrench_hist)])
        theta_hat_hist = np.vstack(self.theta_hat_history[:len(wrench_hist)])

        plt.figure(1)
        plt.plot(times, wrench_hist[:, 0], label='force: x')
        plt.plot(times, wrench_hist[:, 1], label='force: y')
        plt.plot(times, wrench_hist[:, 2], label='force: z')
        plt.legend()

        plt.figure(2)
        plt.plot(times, theta_hat_hist[:, 0], label='mass')
        # plt.plot(times, theta_hat_hist[:, 1], label='viscous friction')
        # plt.plot(times, (error_norms * 100), label='error')
        plt.plot(times, errors[:, 0] * 100, label='Error: x (cm)')
        plt.plot(times, errors[:, 1] * 100, label='Error: y (cm)')
        plt.plot(times, errors[:, 2] * 100, label='Error: z (cm)')

        plt.legend()
        plt.show()
        np.savez('analysis', wrenches=wrench_hist, states=q_hist, times=times)

    def get_Y(self, edot, qdot):

        g = np.transpose([[0.0, 0.0, 1.0]])
        edot_T = np.transpose([edot])
        qdot_T = np.transpose([qdot])

        # NOT using edot_T in the mass estimation, because we are not using actual inertial mass
        # Y = np.hstack([g, qdot_T *])

        # With water resistance
        # Y = np.hstack([g, edot_T * np.linalg.norm(edot_T), self.alpha.dot(edot_T)])
        # Y = np.hstack([-g, edot_T * np.linalg.norm(edot_T)])
        Y = np.hstack([-g + self.alpha.dot(edot_T), -qdot_T])
        return Y

    def adaptive_update(self, e, edot, qdot):
        dt = self.sampling_period.to_sec()
        # theta_hat = np.copy(self.theta_hat)

        r = self.Kp_trans.dot(e) + self.Kd_trans.dot(edot)
        Y = self.get_Y(edot, qdot)
        print self.gamma.dot(Y.transpose().dot(r))
        self.theta_hat += self.gamma.dot(np.transpose(Y).dot(r)) * dt

        # return np.clip(self.theta_hat, [0.0, 0.0], [np.inf, np.inf])
        return self.theta_hat

    def execute(self, current, target, state_history):
        current_position, current_linear_vel, current_orientation_q, current_angular_vel = current
        target_position, target_linear_vel, target_orientation_q, target_angular_vel = target
        orientation_est = transformations.quaternion_matrix(current_orientation_q)[:3, :3]

        error_position = current_position - target_position
        error_linear_vel = current_linear_vel - target_linear_vel
        error_orientation = self.orientation_error(target_orientation_q, current_orientation_q)
        error_angular_vel = current_angular_vel - target_angular_vel

        theta_hat = self.adaptive_update(error_position, error_linear_vel, current_linear_vel)

        # kr = (self.cfg['kp_trans'] * error_position) + (self.cfg['kd_trans'] * -error_linear_vel)
        kr = self.Kp_trans.dot(error_position) + self.Kd_trans.dot(error_linear_vel)

        # world_force = -(self.get_Y(error_linear_vel, current_linear_vel).dot(theta_hat) - kr)
        # world_force = -kr + np.array([0.0, 0.0, 99.1])
        world_force = -kr - self.get_Y(error_linear_vel, current_linear_vel).dot(theta_hat)
        world_force = np.clip(world_force, [-250, -250, -250], [250, 250, 250])
        # print 'm:', theta_hat[0], '| f:', theta_hat[1]
        # print 'force:', world_force

        body_force = orientation_est.dot(world_force)

        # ORIENTATION CONTROL (NOT FOR PROJECT!!!!)
        # I did not do a Lyapunov-based proof of this section

        # Might be negative of error_angular_vel
        world_torque = (self.cfg['kp_angle'] * error_orientation) + (self.cfg['kd_angle'] * error_angular_vel)
        body_torque = orientation_est.dot(world_torque)

        # Send the wrench to the thruster mapper!
        self.q_history.append((current_position, current_linear_vel))
        self.error_history.append(error_position)
        self.wrench_history.append(world_force)
        self.theta_hat_history.append(np.copy(theta_hat))
        self.wrench_times.append((rospy.Time.now() - self.start_time).to_sec())
        self.send_wrench(body_force, body_torque)


if __name__ == '__main__':
    # controller = Controller(odom_topic='/truth/odom', sampling_period=0.01, control_period=0.01,
                # history_length=100, waypoint_epsilon=1)
    controller = AdaptiveController(odom_topic='/odom', sampling_period=0.1, control_period=0.1,
                                    history_length=100, waypoint_epsilon=1, plot=True)
    controller.start()
    rospy.spin()
