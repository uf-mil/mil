from __future__ import annotations

import json


class BaseMission:
    """
    The base for all missions used in mil_missions. Lots of this class
    is just documentation for the various functions that real missions
    can overload. Individual ROS robotics platforms should extend this
    base class to provide interfaces to the particular systems on the robot.

    .. container:: operations

        .. describe:: str(x)

            Prints the name of the mission.
    """

    nh = None
    mission_runner = None

    def __init__(self, parent=None):
        self.parent = parent

    @classmethod
    def name(cls) -> str:
        """
        Override for real missions to return a string for how the mission
        with be referenced in the GUI/CLI. For example, a mission implemented
        in class ``MyCoolMission`` might implement.

        .. code-block:: python

            class MyCoolMission:
                @classmethod
                def name(cls):
                    return 'My cool mission'

        Returns:
            str: The name of the mission. By default, simply returns the class'
            ``__name__`` method.
        """
        return cls.__name__

    @classmethod
    async def setup(cls) -> None:
        """
        Used to setup individual child missions. This is called after the base
        mission is setup using :meth:`~.setup_base`.

        This method should be overridden for all child missions who wish to have a
        resource ready for when the mission begins.

        Any resource setup in this method should be shutdown using the :meth:`~.shutdown`
        method.

        .. code-block:: python

            class MovementMission(MyFancyRobotMission):
                @classmethod
                async def setup(cls):
                    self.my_sub = await self.nh.subscribe("/my_topic", MyMessage)

                @classmethod
                async def shutdown(cls):
                    await self.my_sub.shutdown()
        """

    @classmethod
    async def setup_base(cls, mission_runner) -> None:
        """
        Sets up a base mission, used to generate individual child missions that perform
        individual actions. This method should set up resources needed by all child
        missions, so that they will be available when the child mission begins.

        This method should only be used for base missions, and there should be just
        one base mission per individual robotic system.

        .. code-block:: python

            class MyFancyRobotMission:
                @classmethod
                async def setup_base(cls, mission_runner):
                    await super(cls).setup_base(mission_runner)

        Args:
            mission_runner (:class:`MissionRunner`): The mission runner that will
                run the missions. Used to allow the individual missions to send
                feedback to the mission runner.
        """
        cls.mission_runner = mission_runner
        cls.nh = cls.mission_runner.nh

    @classmethod
    async def shutdown(cls) -> None:
        """
        Shuts down a child mission. This is called when the mission server is shutting
        down all individual child missions.

        Any resources that were setup using :meth:`~.setup` should be considered for
        shutdown using this method.
        """
        pass

    @classmethod
    async def shutdown_base(cls) -> None:
        """
        Shuts down a base mission. This is called when the mission server is shutting
        down, and can be used to ensure that resources are properly closed. This
        is called before each individual child mission is shutdown using :meth:`~.shutdown`.

        Any resources that were setup using :meth:`~.setup_base` should be considered for
        shutdown using this method.
        """
        pass

    def send_feedback(self, message: str) -> None:
        """
        Send a string as feedback to any clients monitoring this mission. If the
        mission is a child mission, it will call the send_feedback_child of its
        parent, allowing missions to choose how to use the feedback from its children.
        """
        if self.parent:
            self.parent.send_feedback_child(message, self)
        else:
            self.mission_runner.send_feedback(message)

    def send_feedback_child(self, message: str, child: BaseMission):
        """
        Called by child missions when sending feedback. By default sends this feedback prefixed
        with the name of the child mission.
        """
        self.send_feedback(f"{child.name()}: {message}")

    @classmethod
    def has_mission(cls, name: str):
        """
        Returns true if the mission runner has a mission with specified name.
        """
        return cls.mission_runner.has_mission(name)

    @classmethod
    def get_mission(cls, name: str):
        """
        Returns the mission with the specified name.
        """
        return cls.mission_runner.get_mission(name)

    async def run_submission(self, name: str, parameters: str = "") -> None:
        """
        Runs another mission available to the mission server, returning the string
        result of the missions execution.

        Args:
            name (str): The name of the submission to spawn as a string. If this
                mission is unknown, raise an exception.
            parameters (str): Parameters to pass to the run function of the submission. Note,
                this function does not call decode_parameters, so parent
                missions need to do this or otherwise ensure the parameters are in
                the format expected by the child. Defaults to an empty string.

        Raises:
            Exception: The submission name is unrecognized - therefore, no submission
                can be run.

        Returns:
            Optional[str]: The result of the mission with the given name.
        """
        if not self.has_mission(name):
            raise Exception(f"Cannot run_submission, '{name}' unrecognized")
        mission = self.mission_runner.missions[name](parent=self)
        return await mission.run(parameters)

    @classmethod
    def decode_parameters(cls, parameters: str) -> dict | str:
        """
        Process parameters string from new mission goal or submission. Should return the
        processes parameters which will be passed to the run function. By default
        returns the json decoded object in the string or, if this fails, just
        the original string.

        If this function throws an exception (such as a ParametersException), the
        mission will be aborted.
        """
        try:
            return json.loads(parameters)
        except ValueError:
            return parameters

    def run(self, parameters):
        """
        The actual body of the mission. Should attempt to execute whatever is expected
        of the mission, using the interfaces set up in :meth:`~.setup` or :meth:`~.setup_base` to
        command actuator movements, read perception output, etc. Should use :meth:`~.send_feedback`
        to update clients about what the mission is doing at the moment.

        If something goes wrong, raise an exception describing what went wrong
        and the mission will be aborted.

        If it executes successfully, return with a string to send a final result
        to the connected clients. Missions can also spawn other missions in the
        run function using :meth:`.run_submission`.

        Args:
            parameters: Arguments to modify the behavior of the mission. By default
                will be a json decoded object from the string passed in the goal,
                but can be changed by overriding decode_parameters.
        """

    def __str__(self) -> str:
        return self.__class__.__qualname__
