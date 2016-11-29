#!/usr/bin/env python
import txros
import navigator_tools as nt
from navigator_tools import DBHelper, fprint
from twisted.internet import threads, defer
from navigator_tools import rosmsg_to_numpy
import numpy as np
from txros import util


def get_closest_objects(position, objects, max_len=3):
    num = len(objects)
    idx = max_len
    if num < max_len:
        idx = num
    objects = sorted(objects, key=lambda x: np.linalg.norm(position - rosmsg_to_numpy(x.position)))
    return objects[:idx]


@txros.util.cancellableInlineCallbacks
def wait_for_object(objec, helper, nh):
    helper.set_looking_for(objec)
    while not helper.is_found_func():
        yield nh.sleep(1)


@txros.util.cancellableInlineCallbacks
def go_to_objects(navigator, position, objs):
    objects = get_closest_objects(position, objs)
    for o in objects:
        fprint("MOVING TO OBJECT WITH ID {}".format(o.id), msg_color="green")
        yield navigator.nh.sleep(5)
        pos = nt.rosmsg_to_numpy(o.position)
        yield navigator.move.look_at(pos).set_position(pos).backward(7).go()


@txros.util.cancellableInlineCallbacks
def myfunc(navigator, looking_for, center_marker, timeout=60):
    db = yield DBHelper(navigator.nh).init_(navigator=navigator)
    objs = yield db.get_unknown_and_low_conf()
    print[x.name for x in objs]
    print[x.id for x in objs]
    print[x.confidence for x in objs]

    if len(objs) == 0:
        fprint("SPIRALING, NO OBJECTS FOUND", msg_color="green")
        moving = navigator.move.spiral(nt.rosmsg_to_numpy(center_marker.position)).go()
        navigator.nh.sleep(60)
    else:
        position = yield navigator.tx_pose
        position = position[0]
        objs = get_closest_objects(position, objs)
        moving = go_to_objects(navigator, position, objs)

    moving.addErrback(lambda x: x)
    try:
        waiting = wait_for_object(looking_for, db, navigator.nh)
        waiting.addErrback(lambda x: x)
        yield util.wrap_timeout(waiting, timeout)
    except util.TimeoutError:
        moving.cancel()
        fprint("NO OBJECT FOUND", msg_color="blue")
        defer.returnValue(False)

    fprint("EXPLORER FOUND OBJECT", msg_color="blue")
    moving.cancel()
    defer.returnValue(True)


@txros.util.cancellableInlineCallbacks
def main(navigator, **kwargs):
    # center_marker = kwargs["center_marker"]
    # looking_for = kwargs["looking_for"]

    center_marker = "ScanTheCode"
    looking_for = "dne"

    navigator.change_wrench("autonomous")
    yield navigator.nh.sleep(.1)
    good = yield myfunc(navigator, looking_for, center_marker)
    defer.returnValue(good)
