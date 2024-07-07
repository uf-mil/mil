#!/usr/bin/env python3
import os

from actionlib import SimpleActionClient, TerminalState
from mil_missions_core import MissionClient
from mil_msgs.msg import BagOnlineAction, BagOnlineGoal
from ros_alarms import Alarm, HandlerBase


class Kill(HandlerBase):
    alarm_name = "kill"

    def __init__(self):
        self._killed = False
        self.initial_alarm = Alarm(
            self.alarm_name,
            True,
            node_name="alarm_server",
            problem_description="Initial kill",
        )
        self.bag_client = SimpleActionClient("/online_bagger/bag", BagOnlineAction)
        self.task_client = MissionClient()
        self.first = True

    def _online_bagger_cb(self, status, result):
        if status == 3:
            self.get_logger().info(f"KILL BAG WRITTEN TO {result.filename}")
        else:
            self.get_logger().warn(
                f"KILL BAG {TerminalState.to_string(status)}, status: {result.status}",
            )

    def _kill_task_cb(self, status, result):
        if status == 3:
            self.get_logger().info("Killed task success!")
            return
        self.get_logger().warn(
            f"Killed task failed ({TerminalState.to_string(status)}): {result.result}",
        )

    def raised(self, alarm):
        self._killed = True
        if self.first:
            self.first = False
            return
        if "BAG_ALWAYS" not in os.environ or "bag_kill" not in os.environ:
            self.get_logger().warn(
                "BAG_ALWAYS or BAG_KILL not set. Not making kill bag.",
            )
        else:
            goal = BagOnlineGoal(bag_name="kill.bag")
            goal.topics = os.environ["BAG_ALWAYS"] + " " + os.environ["BAG_KILL"]
            self.bag_client.send_goal(goal, done_cb=self._online_bagger_cb)
        self.task_client.run_mission("Killed", done_cb=self._kill_task_cb)

    def cleared(self, alarm):
        self._killed = False

    def meta_predicate(self, meta_alarm, alarms):
        ignore = []

        # Don't kill on low battery, only on critical
        # if alarms['battery-voltage'].raised and alarms['battery-voltage'].severity < 2:
        #     ignore.append('battery-voltage')

        # Raised if any alarms besides the two above are raised
        raised = [
            name
            for name, alarm in alarms.items()
            if name not in ignore and alarm.raised
        ]
        if len(raised):
            return Alarm(
                "kill",
                True,
                node_name=self.get_name(),
                problem_description="Killed by meta alarm(s) " + ", ".join(raised),
                parameters={"Raised": raised},
            )
        return self._killed
