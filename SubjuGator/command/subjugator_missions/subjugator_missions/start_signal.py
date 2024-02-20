#!/usr/bin/env python3
from .sub_singleton import SubjuGatorMission


class StartSignal(SubjuGatorMission):
    buoy_positions = [[5, 10, 3], [6, 8, 5], [3, 12, 10]]  # x, y, z displacements

    async def run(self, args):
        for i in len(self.buoy_positions):
            self.send_feedback(f"Travelling to Buoy {i}")
            await self.go(
                self.move()
                .right(self.buoy_positions[i][0])
                .up(self.buoy_positions[i][1].forward(self.buoy_positions[i][2])),
            )
            self.send_feedback("Back to Origin")
            await self.go(
                self.move()
                .right(-self.buoy_positions[i][0])
                .up(-self.buoy_positions[i][1].forward(-self.buoy_positions[i][2])),
            )
