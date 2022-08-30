#!/usr/bin/env python3
import txros
from twisted.internet import defer

from .navigator import Navigator


class GrinchDeploy(Navigator):
    """
    Deploy the grinch
    """

    async def run(self, parameters):
        await self.deploy_grinch()
        return True
