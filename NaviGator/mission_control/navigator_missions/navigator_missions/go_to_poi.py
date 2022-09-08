#!/usr/bin/env python3
from .navigator import Navigator


class GoToPOI(Navigator):
    """
    Moves NaviGator to a point of interest
    """

    @classmethod
    def decode_parameters(cls, parameters):
        parameters = parameters.split()
        if len(parameters) != 1:
            raise Exception("Only one parameter accepted")
        return parameters[0]

    async def run(self, poi):
        self.send_feedback("Waiting for " + poi)
        position = await self.poi.get(poi)
        self.send_feedback(f"Moving to {poi} at {position[0:2]}")
        await self.change_wrench("autonomous")
        await self.move.look_at(position).set_position(position).go()
        return "Success"
