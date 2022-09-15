from typing import Type, Union

from .exceptions import ParametersException


def MakeWait(base: Type):
    """
    Create a Wait mission with the specified base mission. Used by
    robotics platforms to reuse this mission with a different base mission.

    Args:
        base (Type): The base class to construct the Wait mission from.

    Returns:
        Wait: The constructed waiting mission.
    """

    class Wait(base):
        """
        Class to simply sleep for some time (by default 1 second).
        """

        DEFAULT_SECONDS = 1.0

        @classmethod
        def decode_parameters(cls, parameters: str):
            if parameters != "":
                try:
                    return float(parameters)
                except ValueError:
                    raise ParametersException("must be a number")
            return cls.DEFAULT_SECONDS

        async def run(self, time: Union[float, int]):
            self.send_feedback(f"waiting for {time} seconds")
            await self.nh.sleep(time)
            return "Done waiting."

    return Wait
