#!/usr/bin/env python
import txros
import mil_tools as nt
from twisted.internet import defer


@txros.util.cancellableInlineCallbacks
def myfunc(navigator, **kwargs):
    pos = yield navigator.tx_pose
    pos = pos[0]

    exploring = ["Exploring1", "Exploring2", "Exploring3", "Exploring4"]

    for e in exploring:
        try:
            objects = yield navigator.database_query(e)
            o = objects.objects[0]
            print o.name
            pos = nt.rosmsg_to_numpy(o.position)
            yield navigator.move.set_position(pos).go()
            nt.fprint(o.name, msg_color='green')

        except:
            nt.fprint("Missing Marker", msg_color="red")


@txros.util.cancellableInlineCallbacks
def main(navigator, **kwargs):
    navigator.change_wrench("autonomous")
    yield navigator.nh.sleep(.1)
    good = yield myfunc(navigator)
    defer.returnValue(good)
