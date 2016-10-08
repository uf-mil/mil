#!/usr/bin/env python
from __future__ import division

import txros
from twisted.internet import defer

import navigator_tools
from navigator_msgs.msg import MoveToWaypointAction, MoveToWaypointActionFeedback, MoveToWaypointActionResult


@util.cancellableInlineCallbacks
def main():
    nh = yield txros.NodeHandle.from_argv('lqrrt_tester', anonymous=True)
    
    ac = action.ActionClient(nh, '/move_to', TestAction)

    goal_manager = ac.send_goal(TestGoal(
        goal=x,
    ))
    print 'sent goal'
    
    while True:
        result, index = yield defer.DeferredList([goal_manager.get_feedback(), goal_manager.get_result()], fireOnOneCallback=True, fireOnOneErrback=True)
        
        if index == 0: # feedback
            print 'got feedback', result.feedback
        else:
            assert index == 1 # result
            
            assert result.result == x + 1000, result.result
            print 'success'
            
            break

util.launch_main(main)
