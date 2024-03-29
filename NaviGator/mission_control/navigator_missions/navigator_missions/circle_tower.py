#!/usr/bin/env python3
from __future__ import annotations

from .navigator import NaviGatorMission


class CircleTower(NaviGatorMission):
    """
    Simple mission to circle totems once they have been labeled, does not
    have searching functionality found
    """

    CIRCLE_DISTANCE = 5.0  # Distance around totem to circle
    DIRECTIONS: dict[str, str] = {
        "RED": "cw",
        "GREEN": "ccw",
        "BLUE": "cw",
        "YELLOW": "ccw",
        "WHITE": "ccw",
    }

    @classmethod
    def decode_parameters(cls, parameters):
        return parameters.split()

    async def run(self, parameters):
        # Default to R G B pattern
        if len(parameters) == 0:
            colors = ["RED", "GREEN", "BLUE"]
            if self.net_stc_results is not None:
                for i in range(0, 3):
                    val = self.net_stc_results[i]
                    if val == "R":
                        colors[i] = "RED"
                    elif val == "G":
                        colors[i] = "GREEN"
                    elif val == "B":
                        colors[i] = "BLUE"
                self.send_feedback(
                    "Colors specified, running {}".format(" ".join(colors)),
                )
            else:
                self.send_feedback(
                    "No colors Specified, defaulting to {}".format(" ".join(colors)),
                )
        else:
            # Make sure they are valid colors
            for color in parameters:
                if color not in self.DIRECTIONS:
                    raise Exception(f"Color {color} is not known.")
            colors = parameters

        colors = self.net_stc_result

        # Get each totem's position
        targets = []
        for color in colors:
            res = await self.get_sorted_objects(
                "totem_" + color.lower(),
                n=1,
                throw=False,
            )
            if res is None:
                self.send_feedback(f"Totem {color} not found")
                continue
            position = res[1][0]
            self.send_feedback(f"Totem {color} found!")
            targets.append((position, color))
            await self.nh.sleep(0.1)

        # If none found, mission is failed
        if len(targets) == 0:
            return False

        await self.nh.sleep(0.1)

        # Circle each totem
        for target in targets:
            color = target[1]
            position = target[0]
            direction = self.DIRECTIONS[color]
            self.send_feedback(f"Attempting to circle {color} {direction}")
            self.send_feedback("Moving in front of totem")
            await self.nh.sleep(0.1)

            # Move close to totem
            move = (
                self.move.look_at(position)
                .set_position(position)
                .backward(self.CIRCLE_DISTANCE)
            )

            # Rotate for faster rotate
            move = move.yaw_left(1.57) if direction == "cw" else move.yaw_right(1.57)
            await move.go()

            self.send_feedback("Circling!")
            await self.nh.sleep(0.1)
            res = await self.move.circle_point(
                position,
                direction=direction,
                revolutions=1.3,
            ).go()
            self.send_feedback("Done circling")
            await self.nh.sleep(0.1)
        return True
