#!/usr/bin/env python3
from .navigator import Navigator


class ReloadLauncher(Navigator):
    async def run(self, parameters):
        await self.reload_launcher()
        return "Success"
