#!/usr/bin/env python3
from typing import Any, Callable, Optional

import rospy
from actionlib import SimpleActionClient
from mil_missions.msg import DoMissionAction, DoMissionGoal
from roboteq_msgs.msg import Feedback


class MissionClient(SimpleActionClient):
    """
    Extends SimpleActionClient to do boostrap of making a mission
    client for you.
    """

    NS = "/mission"
    LIST_PARAM = "/available_missions"

    def __init__(self):
        SimpleActionClient.__init__(self, self.NS, DoMissionAction)

    @classmethod
    def available_missions(cls) -> Optional[Any]:
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

    def run_mission(
        self,
        mission: str,
        parameters: str = "",
        done_cb: Optional[Callable[[int, Any], None]] = None,
        active_cb: Optional[Callable[[], None]] = None,
        feedback_cb: Optional[Callable[[Feedback], None]] = None,
    ):
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
            goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb
        )
