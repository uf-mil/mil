#!/usr/bin/env python
import txros
import navigator_tools as nt
from navigator_tools import DBHelper
from twisted.internet import threads, defer


@txros.util.cancellableInlineCallbacks
def main(navigator, circle_marker):
    db = DBHelper()
    # Get all the objects in the db
    resp = yield navigator.database_query(circle_marker)
    spiral = navigator.move.spiral(nt.rosmsg_to_numpy(resp.objects[0].position)).go()
    spiral.addErrback(lambda x: x)
    result = yield threads.deferToThread(db.wait_for_additional_objects(timeout=30))
    spiral.cancel()
    defer.returnValue(result)
