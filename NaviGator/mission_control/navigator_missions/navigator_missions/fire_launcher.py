#!/usr/bin/env python3
import txros

from .navigator import Navigator


class FireLauncher(Navigator):
    async def run(self, parameters):
        await self.fire_launcher()
        return "Success"
