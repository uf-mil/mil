#!/usr/bin/env python3
from .navigator import Navigator


class GrinchDeploy(Navigator):
    """
    Deploy the grinch
    """

    async def run(self, parameters):
        await self.deploy_grinch()
        return True
