from __future__ import division

import numpy

from tf import transformations
from uf_common.orientation_helpers import quat_to_rotvec


class Controller(object):
    '''
    config is a dict of k, ks, alpha, beta

    input poses must all be in world frame/velocities must all be in body frame (follows convention from Odometry message)
    output wrench is in body frame
    '''

    def __init__(self, config):
        self.config = config

        self.reset()

    def reset(self):
        self._rise_term = numpy.zeros(6)
        self._rise_term_int_prev = numpy.zeros(6)

    def update(self, dt, desired, current):
        (p, o), (p_dot, o_dot) = current
        (desired_p, desired_o), (desired_p_dot, desired_o_dot), (desired_p_dotdot, desired_o_dotdot) = desired
        world_from_body = transformations.quaternion_matrix(o)[:3, :3]
        x_dot = numpy.concatenate([
            world_from_body.dot(p_dot),
            world_from_body.dot(o_dot),
        ])

        # world_from_desiredbody = transformations.quaternion_matrix(desired_o)[:3, :3]
        desired_x_dot = numpy.concatenate([
            world_from_body.dot(desired_p_dot),
            world_from_body.dot(desired_o_dot),
        ])
        desired_x_dotdot = numpy.concatenate([
            world_from_body.dot(desired_p_dotdot),
            world_from_body.dot(desired_o_dotdot),
        ])

        error_position_world = numpy.concatenate([
            desired_p - p,
            quat_to_rotvec(transformations.quaternion_multiply(
                desired_o,
                transformations.quaternion_inverse(o),
            )),
        ])
        if self.config['two_d_mode']:
            error_position_world = error_position_world * [1, 1, 0, 0, 0, 1]

        world_from_body2 = numpy.zeros((6, 6))
        world_from_body2[:3, :3] = world_from_body2[3:, 3:] = world_from_body

        # Permitting lambda assignment b/c legacy
        body_gain = lambda x: world_from_body2.dot(x).dot(world_from_body2.T)  # noqa

        error_velocity_world = (desired_x_dot + body_gain(numpy.diag(self.config['k'])).dot(error_position_world)) - x_dot
        if self.config['two_d_mode']:
            error_velocity_world = error_velocity_world * [1, 1, 0, 0, 0, 1]

        pd_output = body_gain(numpy.diag(self.config['ks'])).dot(error_velocity_world)

        output = pd_output
        if self.config['use_rise']:
            rise_term_int = body_gain(numpy.diag(self.config['ks'] * self.config['alpha'])).dot(error_velocity_world) + \
                body_gain(numpy.diag(self.config['beta'])).dot(numpy.sign(error_velocity_world))

            self._rise_term = self._rise_term + dt / 2 * (rise_term_int + self._rise_term_int_prev)
            self._rise_term_int_prev = rise_term_int

            output = output + self._rise_term
        else:
            # zero rise term so it doesn't wind up over time
            self._rise_term = numpy.zeros(6)
            self._rise_term_int_prev = numpy.zeros(6)
        output = output + body_gain(numpy.diag(self.config['accel_feedforward'])).dot(desired_x_dotdot)
        output = output + body_gain(numpy.diag(self.config['vel_feedforward'])).dot(desired_x_dot)

        # Permitting lambda assignment b/c legacy
        wrench_from_vec = lambda output: (world_from_body.T.dot(output[0:3]), world_from_body.T.dot(output[3:6]))  # noqa
        return wrench_from_vec(pd_output), wrench_from_vec(output)
