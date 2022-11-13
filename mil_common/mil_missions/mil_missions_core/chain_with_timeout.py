import asyncio
import json
from typing import Any, Dict, List, Type

from twisted.python import failure

from .exceptions import ParametersException, SubmissionException, TimeoutException


def MakeChainWithTimeout(base: Type):
    """
    Generate a ``ChainWithTimeout`` mission with the BaseMission specified. Used by
    individual robotics platforms to reuse this example mission.

    For documentation on the ``ChainWithTimeout`` class, you will need to view
    the source code.

    Args:
        base (Type): A base class to use when constructing the ``ChainWithTimeout`` class.

    Returns:
        ChainWithTimeout: The constructed class.
    """

    class ChainWithTimeout(base):
        """
        Example of a mission which runs an arbitrary number of other missions in a linear order. This
        is the mission used by the rqt plugin under the "Chained Missions" section.
        """

        async def run_submission_with_timeout(
            self, mission: str, timeout: float, parameters: str
        ):
            """
            Runs a child mission, throwing an exception if more time than specified in timeout
            passes before the mission finishes executing.

            Args:
                mission (str): The name of the mission to run.
                timeout (float): The amount of time to wait before throwing a
                    :class:`mil_missions_core.TimeoutException`.
                parameters (str): The parameters to send to the mission.

            Raises:
                TimeoutException: The timeout expired and the mission did not complete.
                Exception: The name of the mission provided was not found.

            Returns:
                ???
            """
            submission = self.run_submission(mission, parameters)
            if timeout == 0:  # Timeout of zero means no timeout
                result = await submission
                return result
            timeout_fut = self.nh.sleep(timeout)
            result, index = await asyncio.wait(
                [submission, timeout_fut], return_when=asyncio.FIRST_COMPLETED
            )
            if index == 0:
                timeout_fut.cancel()
                return result
            if index == 1:
                submission.cancel()
                raise TimeoutException(timeout)

        @classmethod
        def decode_parameters(cls, parameters: str) -> Dict[Any, Any]:
            """
            Goes through list of missions to chain and fills in missing attributes,
            like timeout with defaults. If something is invalid, raise an exception.

            The parameters are primarily parsed using :meth:`json.loads`.

            Args:
                parameters (str): The parameters to decode.

            Raises:
                ParametersException: An issue was found with the parameters. The
                    :attr:`~mil_missions_core.ParametersException.msg` explains what
                    issue was found.

            Returns:
                Dict[Any, Any]: The parsed parameters.
            """
            parameters = json.loads(parameters)

            if not isinstance(parameters, dict):
                raise ParametersException("must be a dictionary")
            if "missions" not in parameters:
                raise ParametersException('must have "missions" list')
            if not isinstance(parameters["missions"], list):
                raise ParametersException('"missions" attribute must be a list')
            for mission in parameters["missions"]:
                if "mission" not in mission:
                    raise Exception('invalid parameters, missing attribute "mission"')
                if not cls.has_mission(mission["mission"]):
                    raise Exception(
                        'mission "{}" not available'.format(mission["mission"])
                    )
                if "parameters" not in mission:
                    mission["parameters"] = ""
                try:
                    mission["parameters"] = cls.get_mission(
                        mission["mission"]
                    ).decode_parameters(mission["parameters"])
                except Exception as e:
                    raise ParametersException(
                        "Invalid parameters for {}: {}".format(
                            mission["mission"], str(e)
                        )
                    )
                if "timeout" not in mission:
                    mission["timeout"] = 0
                if "required" not in mission:
                    mission["required"] = True
            return parameters

        async def run(self, parameters: Dict[str, List]):
            """
            Runs a list of child missions specified in the parameters with optional timeouts.

            Args:
                parameters (Dict[str, List]): A dictionary containing at least a
                    ``"missions"`` key which is equal to a list of mission objects that
                    can be ran.
            """
            for mission in parameters["missions"]:  # Run each child mission linearly

                def cb(final):
                    """
                    Called when a submission finishes. If it succeeded, print the result in feedback.
                    If it failed or timedout, print the failure and stop the whole chain if that
                    mission is required.
                    """
                    if isinstance(final, failure.Failure):
                        self.send_feedback(
                            "{} FAILED: {}".format(
                                mission["mission"], final.getErrorMessage()
                            )
                        )
                        if mission[
                            "required"
                        ]:  # Fail whole chain if a required mission times out or fails
                            raise SubmissionException(
                                mission["mission"], final.getErrorMessage()
                            )
                    else:
                        self.send_feedback(
                            "{} SUCCEEDED: {}".format(mission["mission"], final)
                        )
                        print("NO FAIL BRO")

                fut = self.run_submission_with_timeout(
                    mission["mission"], mission["timeout"], mission["parameters"]
                )
                fut.add_done_callback(cb)
                await fut
            self.send_feedback("Done with all")
            return "All missions complete or skipped."

    return ChainWithTimeout
