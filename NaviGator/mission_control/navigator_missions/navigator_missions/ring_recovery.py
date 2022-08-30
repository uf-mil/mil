#!/usr/bin/env python3
import txros
from twisted.internet import defer

from .navigator import Navigator


class RingRecovery(Navigator):
    """
    Completes Ring Recovery challenge by circling the marker totem
    while spinning the grinch
    """

    RADIUS = 2.5
    DEPLOY_DISTANCE = 10.0

    async def get_marker_totem(self):
        _, totem = await self.get_sorted_objects("totem_gator", n=1)
        return totem.reshape(3)

    async def run(self, parameters):
        self.send_feedback("Fetching position of marker totem")
        totem = await self.get_marker_totem()

        # Switch to auotnomous
        await self.change_wrench("autonomous")

        # Move in front of it, but back a little
        self.send_feedback("Moving in front of marker totem")
        await self.move.look_at(totem).set_position(totem).backward(
            self.DEPLOY_DISTANCE
        ).go()

        # Deploy the grinch mechanism into the water
        self.send_feedback("Deploying grinch")
        await self.deploy_grinch()

        # Move back to let mechanism drop
        await self.move.backward(5.0).go()

        # Move in closer to begin circle
        self.send_feedback("Updating position of marker totem")
        totem = await self.get_marker_totem()
        self.send_feedback("Moving closer")
        await self.move.look_at(totem).set_position(totem).backward(self.RADIUS).go(
            blind=True
        )

        # Circle totem
        self.send_feedback("Spinning grinch")
        d = self.spin_grinch()
        self.send_feedback("Circling marker totem")
        await self.move.circle_point(totem).go(blind=True)
        self.send_feedback("Stopping grinch")
        await d.cancel()

        # Backup from totem to safely retract grinch
        self.send_feedback("Backing up")
        await self.move.backward(self.DEPLOY_DISTANCE).go(move_type="skid", blind=True)

        # Retract the mechanism so we can move on
        self.send_feedback("Retracting grinch")
        await self.retract_grinch()

        return "Success!"
