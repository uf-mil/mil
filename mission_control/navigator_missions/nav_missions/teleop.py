#!/usr/bin/env python
import txros
from twisted.internet import defer
import numpy as np
from sensor_msgs.msg import Joy  # We all need a little joy in our lives

wait_for_completition = defer.Deferred()


def watch_dog(msg):
    if msg.buttons[0] or msg.buttons[12]:
        wait_for_completition.callback(None)


@txros.util.cancellableInlineCallbacks
def main(navigator):
    print "TELEOP: You are now in control!"
    print "TELEOP: Switch back to autonomous on the controller (d-pad right or A) to resume autonomous operation."
    yield navigator.change_wrench("rc")

    rc_sub = navigator.nh.subscribe("/joy", Joy, watch_dog)

    yield wait_for_completition

    # Once we get here, the boat is back autonomous
    yield navigator.change_wrench("autonomous")  # Not nessicary but for saftey
    print "TELEOP: Putting boat back into a mission-ary position."

    result = navigator.fetch_result()
    defer.returnValue(result)