#!/usr/bin/env python3

from .vrx import Vrx

___author___ = "Kevin Allen"


class VrxStationKeeping(Vrx):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    async def run(self, parameters):
        await self.wait_for_task_such_that(
            lambda task: task.state in ["ready", "running"]
        )

        self.send_feedback("Waiting for station keeping goal")
        goal_msg = await self.get_latching_msg(self.station_keep_goal)
        goal_pose = await self.geo_pose_to_enu_pose(goal_msg.pose)
        self.send_feedback(f"Going to {goal_pose}")
        await self.move.set_position(goal_pose[0]).set_orientation(goal_pose[1]).go(
            blind=True
        )
