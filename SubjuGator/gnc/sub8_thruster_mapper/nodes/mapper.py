#!/usr/bin/env python3
from __future__ import annotations

import threading
from typing import Tuple

import numpy as np
import rospy
from geometry_msgs.msg import WrenchStamped
from mil_ros_tools import msg_helpers, rosmsg_to_numpy, thread_lock, wait_for_param
from scipy.optimize import minimize
from sub8_msgs.msg import Thrust, ThrusterCmd
from sub8_msgs.srv import (
    BMatrix,
    BMatrixRequest,
    BMatrixResponse,
    UpdateThrusterLayout,
    UpdateThrusterLayoutRequest,
    UpdateThrusterLayoutResponse,
)

lock = threading.Lock()


class ThrusterMapper:
    _min_command_time = rospy.Duration(0.05)
    min_commandable_thrust = 1e-2  # Newtons

    def __init__(self):
        """
        The  thruster mapper receives a desired wrench from the controller, maps it to
        each of the thrusters based on a specified layout, and then sends each thruster
        a specific effort mapped from the amount of force desired.

        This class also has the ability to update the thruster layout if it is specified
        that a thruster was lost.

        The layout should be a dictionary of the form used in ``thruster_mapper.launch``,
        which points to ``sub8_thruster_mapper/config/thruster_layout.yaml``.

        For example:

        .. code-block:: python3

            thrusters:
              FLH: {
                node_id:  0,
                position:  [0.2678, 0.2795, 0.0],
                direction: [-0.866, 0.5, 0.0],
                thrust_bounds: [-81.85, 99.7],
                calib: {
                  forward: [5.24818e-10, -1.52077e-07, 1.63132e-05, -0.000803, 0.026558, 0.023288],
                  backward: [1.34799e-09, 3.08795e-07, 2.66502e-05, 0.0010796, 0.0304559, -0.033722]
                }
              }
              FLV: {
                ...
              }
              ...

        Attributes:
            thruster_layout (dict): The thruster layout dictionary specified in the launchfile
            that points to the YAML file.
            thruster_name_map (list): The ordered list of thruster names.
            B (np.ndarray): The control-input matrix.
            num_thrusters (int): The number of thrusters in the system.
            thruster_bounds (list): The upper and lower bounds for each thruster.
            min_command_time (rospy.Duration): The minimum time between commands.
            min_commandable_thrust (float): The minimum amount of thrust in Newtowns that can
            be commanded.
        """
        self.num_thrusters = 0
        rospy.init_node("thruster_mapper")
        self.last_command_time = rospy.Time.now()
        self.thruster_layout = wait_for_param("thruster_layout")
        self.thruster_name_map = []
        self.dropped_thrusters = []
        self.B = self.generate_B(self.thruster_layout)
        self.Binv = np.linalg.pinv(self.B)
        self.min_thrusts, self.max_thrusts = self.get_ranges()
        self.default_min_thrusts, self.default_max_thrusts = np.copy(
            self.min_thrusts
        ), np.copy(self.max_thrusts)
        self.update_layout_server = rospy.Service(
            "update_thruster_layout", UpdateThrusterLayout, self.update_layout
        )

        # Expose B matrix through a srv
        self.b_matrix_server = rospy.Service("b_matrix", BMatrix, self.get_b_matrix)

        self.wrench_sub = rospy.Subscriber(
            "wrench", WrenchStamped, self.request_wrench_cb, queue_size=1
        )
        self.actual_wrench_pub = rospy.Publisher(
            "wrench_actual", WrenchStamped, queue_size=1
        )
        self.wrench_error_pub = rospy.Publisher(
            "wrench_error", WrenchStamped, queue_size=1
        )
        self.thruster_pub = rospy.Publisher("thrusters/thrust", Thrust, queue_size=1)

    @thread_lock(lock)
    def update_layout(
        self, srv: UpdateThrusterLayoutRequest
    ) -> UpdateThrusterLayoutResponse:
        """
        Update the physical thruster layout.
        This should only be done in a thruster-out event.
        """
        rospy.logwarn("Layout in update...")
        self.dropped_thrusters = srv.dropped_thrusters
        rospy.logwarn(f"Missing thrusters: {self.dropped_thrusters}")

        # Reset min and max thrusts, this will be overwritten by any dropped thrusters
        self.min_thrusts = np.copy(self.default_min_thrusts)
        self.max_thrusts = np.copy(self.default_max_thrusts)

        for thruster_name in self.dropped_thrusters:
            thruster_index = self.thruster_name_map.index(thruster_name)
            self.min_thrusts[thruster_index] = -self.min_commandable_thrust * 0.5
            self.max_thrusts[thruster_index] = self.min_commandable_thrust * 0.5

        rospy.logwarn("Layout updated")
        return UpdateThrusterLayoutResponse()

    def get_ranges(self) -> tuple[np.ndarray, np.ndarray]:
        """
        Get upper and lower thrust limits for each thruster.
        Returns:
            minima, maxima (Tuple[np.ndarray, np.ndarray]): The minimum and maximum
            thrust for each thruster.
        """
        minima = np.array(
            [
                self.thruster_layout["thrusters"][x]["thrust_bounds"][0]
                for x in self.thruster_name_map
            ]
        )
        maxima = np.array(
            [
                self.thruster_layout["thrusters"][x]["thrust_bounds"][1]
                for x in self.thruster_name_map
            ]
        )
        return minima, maxima

    def get_thruster_wrench(
        self, position: list[float], direction: list[float]
    ) -> np.ndarray:
        """
        Compute a single column of B, or the wrench created by a particular thruster

        Args:
            position (list): The position of the thruster in the body frame.
            direction (list): The direction of the thruster in the body frame.

        Returns:
            wrench (np.ndarray): The wrench created by the thruster.
        """
        assert np.isclose(
            1.0, np.linalg.norm(direction), atol=1e-3
        ), "Direction must be a unit vector"
        forces = direction
        torques = np.cross(position, forces)
        wrench_column = np.hstack([forces, torques])
        return np.transpose(wrench_column)

    def get_b_matrix(self, srv: BMatrixRequest) -> BMatrixResponse:
        """
        Return a copy of the B matrix flattened into a 1-D row-major order list
        Args:
            srv (BMatrixRequest): The request message.
        Returns:
            response (BMatrixResponse): The response message.
        """
        return BMatrixResponse(self.B.flatten())

    def generate_B(self, layout: dict) -> np.ndarray:
        """
        Construct the control-input matrix.
        Each column represents the wrench generated by a single thruster.

        The single letter "B" is conventionally used to refer to a matrix which converts
         a vector of control inputs to a wrench.

        Meaning where u = [thrust_1, ... thrust_n],
         B * u = [Fx, Fy, Fz, Tx, Ty, Tz]

        Args:
            layout (dict): The thruster layout dictionary specified in the launchfile
        Returns:
            B (np.ndarray): The control-input matrix.
        """
        # Maintain an ordered list, tracking which column corresponds to which thruster
        self.thruster_name_map = []
        self.thruster_bounds = []
        B = []
        self.num_thrusters = 0
        for thruster_name, thruster_info in layout["thrusters"].items():
            # Assemble the B matrix by columns
            self.thruster_name_map.append(thruster_name)
            wrench_column = self.get_thruster_wrench(
                thruster_info["position"], thruster_info["direction"]
            )
            self.num_thrusters += 1
            B.append(wrench_column)
        return np.transpose(np.array(B))

    def map(self, wrench: np.ndarray) -> np.ndarray:
        """
        Maps a wrench to the thruster space. To achieve this, a constrained
        optimization problem is solved using Sequential Least Squares Programming
        (SLSQP) to find the minimum thrusts that produce the desired wrench subject
        to the minimum and maximum thrust constraints.
        Args:
            wrench (np.ndarray): The wrench to map.
        Returns:
            thrusts (np.ndarray): The thrusts for each thruster.
            success (bool): Whether the mapping was successful.

        TODO:
        - Account for variable thrusters
        """
        thrust_cost = np.diag([1e-4] * self.num_thrusters)

        def objective(u) -> float:
            """
            Tikhonov-regularized least-squares cost function
               https://en.wikipedia.org/wiki/Tikhonov_regularization
            Minimize
                norm((B * u) - wrench) + (u.T * R * u)
            Subject to
                min_u < u < max_u
            Where
                R defines the cost of firing the thrusters
                u is a vector where u[n] is the thrust output by thruster_n

            We make R as small as possible to avoid leaving the solution space of
            B*u = wrench.
            """
            error_cost = np.linalg.norm(self.B.dot(u) - wrench) ** 2
            effort_cost = np.transpose(u).dot(thrust_cost).dot(u)
            return error_cost + effort_cost

        def obj_jacobian(u) -> np.ndarray:
            """
            Compute the jacobian of the objective function.

            [1] Scalar-By-Matrix derivative identities [Online]
                Available: https://en.wikipedia.org/wiki/Matrix_calculus#Scalar-by-vector_identities
            """
            error_jacobian = 2 * self.B.T.dot(self.B.dot(u) - wrench)
            effort_jacobian = np.transpose(u).dot(2 * thrust_cost)
            return error_jacobian + effort_jacobian

        # Initialize minimization at the analytical solution of the unconstrained problem
        minimization = minimize(
            method="slsqp",
            fun=objective,
            jac=obj_jacobian,
            x0=np.clip(self.Binv.dot(wrench), self.min_thrusts, self.max_thrusts),
            bounds=list(zip(self.min_thrusts, self.max_thrusts)),
            tol=1e-6,
        )
        return minimization.x, minimization.success

    @thread_lock(lock)
    def request_wrench_cb(self, msg: WrenchStamped) -> None:
        """
        Callback for requesting a wrench. The callback gets the wrench
        message and attempts to map the wrench to the thruster space. Then
        publishes the resulting thrust for each thruster, as well
        as the actual and error wrench.
        Args:
            msg (WrenchStamped): The wrench to be applied.
        """
        time_now = rospy.Time.now()
        if (time_now - self.last_command_time) < self._min_command_time:
            return
        else:
            self.last_command_time = rospy.Time.now()

        force = rosmsg_to_numpy(msg.wrench.force)
        torque = rosmsg_to_numpy(msg.wrench.torque)
        wrench = np.hstack([force, torque])

        success = False
        while not success:
            u, success = self.map(wrench)
            if not success:
                # If we fail to compute, shrink the wrench
                wrench = wrench * 0.75
                continue

            thrust_cmds = []
            # Assemble the list of thrust commands to send
            for name, thrust in zip(self.thruster_name_map, u):
                # > Can speed this up by avoiding appends
                if name in self.dropped_thrusters:
                    thrust = 0  # Sending a command packet is an opportunity to detect thruster recovery

                # Simulate thruster deadband
                if np.fabs(thrust) < self.min_commandable_thrust:
                    thrust = 0

                thrust_cmds.append(ThrusterCmd(name=name, thrust=thrust))

        actual_wrench = self.B.dot(u)
        self.actual_wrench_pub.publish(
            msg_helpers.make_wrench_stamped(
                actual_wrench[:3], actual_wrench[3:], frame="/base_link"
            )
        )
        mapper_wrench_error = wrench - actual_wrench
        self.wrench_error_pub.publish(
            msg_helpers.make_wrench_stamped(
                mapper_wrench_error[:3], mapper_wrench_error[3:], frame="/base_link"
            )
        )
        self.thruster_pub.publish(thrust_cmds)


if __name__ == "__main__":
    mapper = ThrusterMapper()
    rospy.spin()
