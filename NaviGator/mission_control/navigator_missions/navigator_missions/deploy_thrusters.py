#!/usr/bin/env python3

from .navigator import NaviGatorMission


class DeployThrusters(NaviGatorMission):
    async def run(self, parameters):
        self.send_feedback("Deploying thrusters")
        await self.deploy_thrusters()
        return "Success"
