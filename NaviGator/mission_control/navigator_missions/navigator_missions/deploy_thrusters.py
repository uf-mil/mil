#!/usr/bin/env python3

from .navigator import Navigator


class DeployThrusters(Navigator):
    async def run(self, parameters):
        self.send_feedback("Deploying thrusters")
        await self.deploy_thrusters()
        return "Success"
