#!/usr/bin/env python
import txros
import navigator_tools as nt
from navigator_tools import DBHelper, fprint
from twisted.internet import threads, defer
from navigator_tools import rosmsg_to_numpy
import numpy as np
from txros import util


def get_closest_objects(position, objects):
    num = len(objects)
    idx = 3
    if num >= 3:
        idx = num
    return sorted(objects, key=lambda x: np.linalg.norm(position - rosmsg_to_numpy(x.position)))[:idx]


def wait_for_object(objec, helper, nh):
    helper.set_looking_for(objec)
    while not helper.found():
        nh.sleep(1)


@txros.util.cancellableInlineCallbacks
def main(navigator, **kwargs):
    def func():
        center_marker = kwargs["center_marker"]
        looking_for = kwargs["looking_for"]

        db = yield DBHelper(navigator.nh).init_(navigator=navigator)
        objs = yield db.get_unknown_and_low_conf()
        print[x.name for x in objs]
        print[x.id for x in objs]
        if len(objs) == 0:
            fprint("SPIRALING, NO OBJECTS FOUND", msg_color="green")
            # UNCOMMENT
            yield navigator.move.spiral(nt.rosmsg_to_numpy(center_marker.position)).go()
            navigator.nh.sleep(20)

        position = yield navigator.tx_pose()
        position = position[0]
        objects = get_closest_objects(position, objs)
        moving = navigator.move.set_position(nt.rosmsg_to_numpy(objects[0].position)).go()
        mydef = moving
        for o in objects[1:]:
            fprint("MOVING TO OBJECT WITH ID {}".format(o.id), msg_color="green")
            l = navigator.move.set_position(nt.rosmsg_to_numpy(objects[0].position)).go()
            mydef.chainDeffered(l)
            mydef = l

        moving.addErrback(lambda x: x)
        yield threads.deferToThread(wait_for_object(looking_for, db, navigator.nh))
        fprint("EXPLORER FOUND OBJECT", msg_color="blue")
        moving.cancel()

    good = True
    try:
        util.wrap_timeout(func, 60)
    except util.TimoutError:
        good = False

    defer.returnValue(good)
