from typing import Union

from mil_missions_core import BaseMission


class MissionException(Exception):
    """
    An exception representing a general error which occurred during the execution
    of a mission.

    Attributes:
        message (str): The message explaining what failed.
        parameters (Dict[Any, Any]): ???
    """

    def __init__(self, message):
        super(Exception, self).__init__(message)


class TimeoutException(Exception):
    """
    Represents an exception from a mission or submission not finishing within
    the requested time.

    Inherits from :class:`Exception`.

    .. container:: operations

        .. describe:: str(x)

            Prints an explanatory error message using the provided timeout.

    Attributes:
        timeout (Union[int, float]): The amount of seconds in which the mission/submission
            should have finished in.
    """

    def __init__(self, timeout: Union[float, int]):
        self.timeout = timeout

    def __str__(self):
        return f"failed to finish within {self.timeout} seconds"


class ParametersException(MissionException):
    """
    Represents an exception from a mission or submission where the mission's parameters
    had an error or inconsistency.

    Inherits from :class:`Exception`.

    .. container:: operations

        .. describe:: str(x)

            Prints an explanatory error message using the provided message explaining
            what error occurred.

    Attributes:
        msg (str): A message explaining what error was found in the parameters.
    """

    def __init__(self, msg: str):
        self.msg = msg

    def __str__(self):
        return f"invalid parameters: {self.msg}"


class SubmissionException(MissionException):
    """
    Represents an exception encountered while running a submission.

    Keeps the name of the submission which failed, so the user knowns where
    the failure occurred.

    Inherits from :class:`Exception`.

    .. container:: operations

        .. describe:: str(x)

            Prints an explanatory error message using the provided mission name
            and exception that occurred.

    Attributes:
        mission (str): The name of the failed mission.
        exception (Exception): The exception experienced by the mission.
    """

    def __init__(self, mission: BaseMission, exception: Exception):
        self.mission = mission
        self.exception = exception
        super().__init__(
            f"A {exception.__class__.__name__} exception occurred while running {mission}.",
        )

    def __str__(self):
        return f"Exception occurred in {self.mission} mission: {self.exception.__class__.__name__}: {self.exception}"
