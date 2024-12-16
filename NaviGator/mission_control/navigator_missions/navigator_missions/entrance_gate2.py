#!/usr/bin/env python3
import math

import numpy as np
from mil_tools import quaternion_matrix
from std_srvs.srv import SetBoolRequest

from .navigator import NaviGatorMission


class EntranceGate2(NaviGatorMission):
    async def run(
        self,
        args,
        *,
        scan_code: bool = False,
        return_to_start: bool = True,
        circle: bool = True,
    ):
        circle_radius = 5
        circle_direction = "cw"
        yaw_offset = 1.57
        self.traversal_distance = 3

        await self.set_classifier_enabled.wait_for_service()
        await self.set_classifier_enabled(SetBoolRequest(data=True))

        # Inspect Gates
        await self.change_wrench("/wrench/autonomous")
        await self.move.yaw_left(20, "deg").go()
        await self.move.yaw_right(40, "deg").go()

        # Find the gates
        print("finding gates")
        self.gate_results = await self.find_gates()
        self.gate_centers = self.gate_results[0]
        self.gates_line = self.gate_results[1]
        self.gate_totems = self.gate_results[2]
        print("defined gates")

        # Which gate do we go through?
        self.pinger_gate = 2

        # Calculate traversal points
        traversal_points = await self.get_perpendicular_points(
            self.gate_centers[self.pinger_gate],
            self.traversal_distance,
        )

        # Go through the gate
        self.send_feedback("Navigating through gate")
        await self.move.set_position(traversal_points[0]).look_at(
            traversal_points[1],
        ).go()
        await self.move.set_position(traversal_points[1]).go()

        if scan_code:
            pass
        elif return_to_start and circle:
            # Approach buoy we will rotate around
            buoy = await self.get_sorted_objects("black_cylinder", n=1)
            buoy = buoy[1][0]
            vect = (
                self.unit_vector(self.gate_centers[0], self.gate_centers[1])
                * circle_radius
            )
            if self.pinger_gate > 0:
                vect = -vect
                circle_direction = "ccw"
                yaw_offset = -1.57
            start = buoy + vect
            await self.move.set_position(start).go()

            # Rotate around buoy
            print("beginning spiral movement")
            await self.move.d_spiral_point(
                buoy,
                circle_radius,
                4,
                0.75,
                circle_direction,
                yaw_offset,
            )

        if return_to_start:
            # Go back through start gate
            self.send_feedback("Navigating through original gate")
            await self.move.set_position(traversal_points[1]).look_at(
                traversal_points[0],
            ).go()
            await self.move.set_position(traversal_points[0]).go()

            # Then move a little passed the exit
            await self.move.forward(5).go()
            print("GO NAVIGATOR")

        self.send_feedback("Done with start gate!")

    """

        Math Utilities

    """

    @staticmethod
    def unit_vector(p1, p2):
        """
        Return a vector given two points
        https://stackoverflow.com/questions/20677795/how-do-i-compute-the-intersection-point-of-two-lines-in-python
        """
        vect = np.array([p1[0] - p2[0], p1[1] - p2[1], 0])
        return vect / np.linalg.norm(vect)

    @staticmethod
    def line(p1, p2):
        """
        Return equation of a line given two 2D points
        https://stackoverflow.com/questions/20677795/how-do-i-compute-the-intersection-point-of-two-lines-in-python
        """
        A = p1[1] - p2[1]
        B = p2[0] - p1[0]
        C = p1[0] * p2[1] - p2[0] * p1[1]
        return A, B, -C

    @staticmethod
    def perpendicular(L):
        """
        Calculate the line perpendicular to another line
        """
        v1 = [L[1], -L[0], 0]
        r = v1[0] ** 2 + v1[1] ** 2
        v1 = [v1[0] / r, v1[1] / r]
        v2 = [0, 0, 1]
        pvec = np.cross(v1, v2)
        r = math.sqrt(pvec[0] ** 2 + pvec[1] ** 2)
        return [pvec[0] / r, pvec[1] / r, 0]

    """

        Perception Utilities

    """

    async def find_gates(self):
        # This mission assumes we are starting off looking at the start gate task

        # Get five closest buoys
        buoys = await self.get_sorted_objects("all", n=5)

        # Determine their locations relative to boat
        pose = self.pose
        p = pose[0]
        q_mat = quaternion_matrix(pose[1])
        positions_local = np.array(
            [(q_mat.T.dot(position - p)) for position in buoys[1]],
        )
        positions_local_x = np.array(positions_local[:, 0])  # positive is forward
        positions_local_y = np.array(positions_local[:, 1])  # positive is to the left

        # Sort the x and y axes and delete the black buoy from y axis
        argsort_x = np.argsort(np.array(positions_local_x))
        argsort_y = np.argsort(np.array(positions_local_y))
        argsort_y = np.delete(argsort_y, np.where(argsort_y == argsort_x[-1]))

        # Label buoys
        await self.pcodar_label(buoys[0][argsort_x[-1]].id, "black_cylinder")
        for i, buoy_index in enumerate(argsort_y):
            if i == 0:
                try:
                    t4 = await self.get_sorted_objects("green_cylinder", n=1)
                    t4 = t4[1][0][:2]
                except Exception:
                    t4 = buoys[1][buoy_index][:2]
                    await self.pcodar_label(buoys[0][buoy_index].id, "green_cylinder")
            elif i == 1:
                try:
                    white_totems = await self.get_sorted_objects("white_cylinder", n=2)
                    t3 = white_totems[1][0][:2]
                except Exception:
                    t3 = buoys[1][buoy_index][:2]
                    await self.pcodar_label(buoys[0][buoy_index].id, "white_cylinder")
            elif i == 2:
                try:
                    white_totems = await self.get_sorted_objects("white_cylinder", n=2)
                    t2 = white_totems[1][1][:2]
                except Exception:
                    t2 = buoys[1][buoy_index][:2]
                    await self.pcodar_label(buoys[0][buoy_index].id, "white_cylinder")
            elif i == 3:
                try:
                    t1 = await self.get_sorted_objects("red_cylinder", n=1)
                    t1 = t1[1][0][:2]
                except Exception:
                    t1 = buoys[1][buoy_index][:2]
                    await self.pcodar_label(buoys[0][buoy_index].id, "red_cylinder")

        # Make sure the two white totems get ordered properly
        if (t2[0] - t1[0]) ** 2 + (t2[1] - t1[1]) ** 2 < (t3[0] - t1[0]) ** 2 + (
            t3[1] - t1[1]
        ) ** 2:
            gate_totems = [t1, t2, t3, t4]
        else:
            gate_totems = [t1, t3, t2, t4]

        # Calculate the center points of each gate
        gate_centers = [
            (
                (gate_totems[0][0] + gate_totems[1][0]) / 2,
                (gate_totems[0][1] + gate_totems[1][1]) / 2,
            ),
            (
                (gate_totems[1][0] + gate_totems[2][0]) / 2,
                (gate_totems[1][1] + gate_totems[2][1]) / 2,
            ),
            (
                (gate_totems[2][0] + gate_totems[3][0]) / 2,
                (gate_totems[2][1] + gate_totems[3][1]) / 2,
            ),
        ]

        # Calculate the line that goes through the gates
        gates_line = self.line(gate_centers[0], gate_centers[2])
        return (gate_centers, gates_line, gate_totems)

    """

        Navigation Utilities

    """

    async def get_perpendicular_points(
        self,
        center_point,
        offset_distance,
        boat_pose=None,
    ):
        # Find the perpendicular line
        perpendicular_vector = self.perpendicular(self.gates_line)

        if boat_pose is None:
            boat_pose = await self.tx_pose()
            boat_pose = boat_pose[0]

        # Find the two points on either side of the line
        perpendicular_points = [
            (
                center_point[0] + perpendicular_vector[0] * offset_distance,
                center_point[1] + perpendicular_vector[1] * offset_distance,
            ),
            (
                center_point[0] + perpendicular_vector[0] * -offset_distance,
                center_point[1] + perpendicular_vector[1] * -offset_distance,
            ),
        ]

        # Sort them such that the point on the same side of the boat is first
        if (perpendicular_points[0][0] - boat_pose[0]) ** 2 + (
            perpendicular_points[0][1] - boat_pose[1]
        ) ** 2 > (perpendicular_points[1][0] - boat_pose[0]) ** 2 + (
            perpendicular_points[1][1] - boat_pose[1]
        ) ** 2:
            perpendicular_points = [perpendicular_points[1], perpendicular_points[0]]

        perpendicular_points_np = []

        # Turn the points into 3D numpy points with U=0
        for goal_point in perpendicular_points:
            point = np.array(goal_point)
            point = np.append(point, [0])
            perpendicular_points_np.append(point)

        return perpendicular_points_np
