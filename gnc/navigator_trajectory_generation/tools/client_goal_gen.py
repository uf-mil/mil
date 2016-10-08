#!/usr/bin/env python
from __future__ import division

import txros
from txros import action
from twisted.internet import defer

import numpy as np
import navigator_tools
from navigator_msgs.msg import MoveToWaypointAction, MoveToWaypointActionGoal, MoveToWaypointActionResult

print_t = navigator_tools.print_t

@txros.util.cancellableInlineCallbacks
def main():
    nh = yield txros.NodeHandle.from_argv('lqrrt_tester', anonymous=True)
    t = lambda: nh.get_time().to_sec()

    ac = action.ActionClient(nh, '/move_to', MoveToWaypointAction)

    while True:
        goal = MoveToWaypointActionGoal().goal
        pose = navigator_tools.random_pose([-65, -110, 0], [130, 90, 0])

        goal.target.pose = pose
        goal.target.twist = navigator_tools.numpy_to_twist(np.zeros(3), np.zeros(3))
        goal.goal_bias = 0.6

        print_t(pose, t())
        result = ac.send_goal(goal)
        print_t("Waiting for response...", t())
        response = yield result.get_result()
        print_t("Done!", t())
        print_t(response, t())

        print "\n\n"

        yield nh.sleep(5)

txros.util.launch_main(main)
