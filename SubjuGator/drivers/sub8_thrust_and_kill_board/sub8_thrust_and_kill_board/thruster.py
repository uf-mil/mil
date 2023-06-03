from __future__ import annotations

from typing import Any

from numpy import clip, polyval


def make_thruster_dictionary(dictionary) -> tuple[dict[str, Thruster], dict[str, int]]:
    """
    Make a dictionary mapping thruster names to :class:`Thruster` objects
    and a dictionary mapping thruster names to node IDs.
    """
    ret = {}
    name_id_map = {}
    for thruster, content in dictionary.items():
        ret[thruster] = Thruster.from_dict(content)
        name_id_map[thruster] = content["node_id"]
    return ret, name_id_map


class Thruster:
    """
    Models the force (thrust) to PWM (effort) of a thruster.

    Attributes:
        forward_calibration (???): ???
        backward_calibration (???): ???
    """

    def __init__(self, forward_calibration, backward_calibration):
        self.forward_calibration = forward_calibration
        self.backward_calibration = backward_calibration

    @classmethod
    def from_dict(cls, data: dict[str, dict[str, Any]]):
        """
        Constructs the class from a dictionary. The dictionary should be formatted
        as so:

        .. code-block:: python3

            {
                "calib": {
                    "forward": ...,
                    "backward": ...,
                }
            }

        Args:
            data (Dict[str, Dict[str, Any]]): The dictionary containing the initialization info.
        """
        forward_calibration = data["calib"]["forward"]
        backward_calibration = data["calib"]["backward"]
        return cls(forward_calibration, backward_calibration)

    def effort_from_thrust_unclipped(self, thrust: Any):
        if thrust < 0:
            return polyval(self.backward_calibration, thrust)
        else:
            return polyval(self.forward_calibration, thrust)

    def effort_from_thrust(self, thrust: Any):
        """
        Attempts to find the effort from a particular value of thrust.

        Args:
            thrust (???): The amount of thrust.
        """
        unclipped = self.effort_from_thrust_unclipped(thrust)
        # Theoretically can limit to .66 under 16V assumptions or .5 under 12V assumptions... So do both (.5 + 66)/2
        return clip(unclipped, -0.58, 0.58)
