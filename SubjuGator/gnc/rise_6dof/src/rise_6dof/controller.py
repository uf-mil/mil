from typing import Dict, Tuple, Union

import numpy
from mil_msgs.msg import PoseTwist
from mil_ros_tools.geometry_helpers import quat_to_rotvec
from tf import transformations

numpy.set_printoptions(suppress=True, linewidth=130)

DEBUG = False


class Controller:
    """
    Input poses must all be in world frame/velocities must all be in body frame
    (follows convention from Odometry message). The output wrench should be in the
    body frame.
    """

    def __init__(self, config: Dict[str, Union[numpy.ndarray, bool]]):
        """
        Args:
            config (Dict[str, Union[np.ndarray, bool]]): The configuration to use
                with the controller.
        """
        self.config = config
        self.reset()

    def reset(self) -> None:
        """
        Resets the controller with zero values.
        """
        self._rise_term = numpy.zeros(6)
        self._rise_term_int_prev = numpy.zeros(6)

    def update(
        self, dt: float, desired: PoseTwist, current: PoseTwist
    ) -> Tuple[
        Tuple[numpy.ndarray, numpy.ndarray], Tuple[numpy.ndarray, numpy.ndarray]
    ]:
        """
        Updates the controllers with a few values.

        Args:
            dt (float): ???
            desired (PoseTwist): ???
            current (PoseTwist): ???
        """
        if DEBUG:
            print("=" * 130)
        (p, o), (p_dot, o_dot) = current
        (
            (desired_p, desired_o),
            (desired_p_dot, desired_o_dot),
            (desired_p_dotdot, desired_o_dotdot),
        ) = desired
        world_from_body = transformations.quaternion_matrix(o)[:3, :3]
        x_dot = numpy.concatenate(
            [
                world_from_body.dot(p_dot),
                world_from_body.dot(o_dot),
            ]
        )

        # world_from_desiredbody = transformations.quaternion_matrix(desired_o)[:3, :3]
        desired_x_dot = numpy.concatenate(
            [
                world_from_body.dot(desired_p_dot),
                world_from_body.dot(desired_o_dot),
            ]
        )
        desired_x_dotdot = numpy.concatenate(
            [
                world_from_body.dot(desired_p_dotdot),
                world_from_body.dot(desired_o_dotdot),
            ]
        )

        if DEBUG:
            print("{:20s} {:30}".format("Desired Velocity:", desired_x_dot))

        error_position_world = numpy.concatenate(
            [
                desired_p - p,
                quat_to_rotvec(
                    transformations.quaternion_multiply(
                        desired_o,
                        transformations.quaternion_inverse(o),
                    )
                ),
            ]
        )
        if self.config["two_d_mode"]:
            error_position_world = error_position_world * [1, 1, 0, 0, 0, 1]

        world_from_body2 = numpy.zeros((6, 6))
        world_from_body2[:3, :3] = world_from_body2[3:, 3:] = world_from_body

        # Permitting lambda assignment b/c legacy
        body_gain = lambda x: world_from_body2.dot(x).dot(world_from_body2.T)  # noqa

        error_velocity_world = (
            desired_x_dot
            + body_gain(numpy.diag(self.config["k"])).dot(error_position_world)
        ) - x_dot

        if DEBUG:
            print("{:20s} {:30}".format("Error velocity:", error_velocity_world))

        if self.config["two_d_mode"]:
            error_velocity_world = error_velocity_world * [1, 1, 0, 0, 0, 1]

        pd_output = body_gain(numpy.diag(self.config["ks"])).dot(error_velocity_world)

        if DEBUG:
            print("{:20s} {:30}".format("pd_output:", pd_output))

        output = pd_output
        if self.config["use_rise"]:
            rise_term_int = body_gain(
                numpy.diag(self.config["ks"] * self.config["alpha"])
            ).dot(error_velocity_world) + body_gain(
                numpy.diag(self.config["beta"])
            ).dot(
                numpy.sign(error_velocity_world)
            )

            self._rise_term = self._rise_term + dt / 2 * (
                rise_term_int + self._rise_term_int_prev
            )
            self._rise_term_int_prev = rise_term_int

            output = output + self._rise_term
        else:
            # zero rise term so it doesn't wind up over time
            self._rise_term = numpy.zeros(6)
            self._rise_term_int_prev = numpy.zeros(6)
        output = output + body_gain(numpy.diag(self.config["accel_feedforward"])).dot(
            desired_x_dotdot
        )
        output = output + body_gain(numpy.diag(self.config["vel_feedforward"])).dot(
            desired_x_dot
        )

        # Permitting lambda assignment b/c legacy
        def wrench_from_vec(output):
            return world_from_body.T.dot(output[0:3]), world_from_body.T.dot(
                output[3:6]
            )

        if DEBUG:
            print("{:6} {}".format("PD:", wrench_from_vec(pd_output)))
            print("{:6} {}".format("PID(R):", wrench_from_vec(output)))
        return wrench_from_vec(pd_output), wrench_from_vec(output)
