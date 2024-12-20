#!/usr/bin/env python3

import numpy as np
from mil_tools import rosmsg_to_numpy
from navigator_msgs.srv import MoveToWaypointRequest, TwoClosestConesRequest
from std_srvs.srv import SetBoolRequest

from .vrx import Vrx

___author___ = "Alex Perez"


class VrxNavigation2(Vrx):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    @staticmethod
    def get_gate(one, two, position):
        one = one[:2]
        two = two[:2]
        position = position[:2]
        delta = (one - two)[:2]
        rot_right = np.array([[0, -1], [1, 0]], dtype=np.float)
        perp_vec = rot_right.dot(delta)
        perp_vec = perp_vec / np.linalg.norm(perp_vec)
        center = (one + two) / 2.0
        distances = np.array(
            [
                np.linalg.norm((center + perp_vec) - position),
                np.linalg.norm((center - perp_vec) - position),
            ],
        )
        if np.argmin(distances) == 0:
            perp_vec = -perp_vec
        return np.array([center[0], center[1], 0.0]), np.array(
            [perp_vec[0], perp_vec[1], 0.0],
        )

    async def go_thru_gate(self, gate, BEFORE=5.0, AFTER=4.0):
        center, vec = gate
        before_position = center - (vec * BEFORE)
        after_position = center + (vec * AFTER)
        await self.move.set_position(before_position).look_at(center).go()
        if AFTER > 0:
            await self.move.look_at(after_position).set_position(after_position).go()

    async def go_thru_gate2(self, gate, BEFORE=5.0, AFTER=4.0):
        center, vec = gate
        before_position = center - (vec * BEFORE)
        after_position = center + (vec * AFTER)

        req = MoveToWaypointRequest()
        req.target_p.position.x = before_position[0]
        req.target_p.position.y = before_position[1]
        req.target_p.position.z = before_position[2]
        await self.set_long_waypoint(req)

        if AFTER > 0:
            req.target_p.position.x = after_position[0]
            req.target_p.position.y = after_position[1]
            req.target_p.position.z = after_position[2]
            await self.set_long_waypoint(req)

    async def go_through_next_two_buoys(self):
        buoys = await self.get_two_closest_cones(TwoClosestConesRequest())

        self.task_done = buoys.no_more_buoys

        print(buoys)

        if self.task_done:
            return

        pos1 = rosmsg_to_numpy(buoys.object1)
        pos2 = rosmsg_to_numpy(buoys.object2)
        gate = self.get_gate(pos1, pos2, (await self.tx_pose())[0])
        await self.go_thru_gate(gate)

    async def run(self, parameters):
        self.objects_passed = set()
        self.task_done = False
        gates_passed = 0

        # Wait a bit for PCDAR to get setup
        await self.nh.sleep(10.0)
        await self.move.set_orientation([0, 0, 0.7068, 0.7073]).go()
        await self.set_vrx_classifier_enabled(SetBoolRequest(data=True))
        await self.nh.sleep(4)

        # await self.move.forward(5).go()

        while not self.task_done or gates_passed == 5:
            await self.go_through_next_two_buoys()
            gates_passed += 1

        await self.set_vrx_classifier_enabled(SetBoolRequest(data=False))
