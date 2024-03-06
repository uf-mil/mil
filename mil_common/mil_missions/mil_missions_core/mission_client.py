#!/usr/bin/env python3
from __future__ import annotations

from typing import Any, Callable

import rospy
from actionlib import SimpleActionClient
from mil_missions.msg import DoMissionAction, DoMissionGoal, DoMissionResult
from roboteq_msgs.msg import Feedback


class MissionClient(SimpleActionClient):
    """
    Extends SimpleActionClient to do bootstrap of making a mission
    client for you.
    """

    NS = "/mission"
    LIST_PARAM = "/available_missions"

    def __init__(self):
        SimpleActionClient.__init__(self, self.NS, DoMissionAction)

    @classmethod
    def available_missions(cls) -> Any | None:
        """
        Returns a list of available missions or None if parameter is not set.

        Returns:
            Optional[Any]: The list of available missions. This is typed as Any
            currently because it is not yet known what type is returned by the parameter
            request.
        """
        if not rospy.has_param(cls.LIST_PARAM):
            return None
        return rospy.get_param(cls.LIST_PARAM)

    def cancel_mission(self):
        """
        Send a goal to cancel the current mission.
        """
        self.cancel_all_goals()

    def get_result(self) -> DoMissionResult | None:
        return super().get_result()

    def get_state(self) -> int | None:
        return super().get_state()

    def run_mission(
        self,
        mission: str,
        parameters: str = "",
        done_cb: Callable[[int, Any], None] | None = None,
        active_cb: Callable[[], None] | None = None,
        feedback_cb: Callable[[Feedback], None] | None = None,
    ) -> None:
        """
        Send a goal to start a new mission with the specified parameters.

        Args:
            mission (str): The name of the mission to start.
            parameters (str): The parameters to send to the mission. Defaults to an
                empty string.
            done_cb (Optional[Callable[[int, Any], None]]): The callback function to send to the action
                client to execute when the goal is done. The callable should take two
                parameters: an integer (specifically, an enumerated integer value from
                :class:`actionlib_msgs.msg._GoalStatus.GoalStatus`) and the result.
            active_cb (Optional[Callable[[], None]]): The callback function that gets called
                on transitions to an active state.
            feedback_cb (Optional[Callable[[Any], None]]): Callback that is executed when feedback
                for the goal is received. The callback should take one parameter: the
                feedback received.
        """
        goal = DoMissionGoal(mission=mission, parameters=parameters)
        return self.send_goal(
            goal,
            done_cb=done_cb,
            active_cb=active_cb,
            feedback_cb=feedback_cb,
        )
