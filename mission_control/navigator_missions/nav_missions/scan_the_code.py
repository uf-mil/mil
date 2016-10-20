#!/usr/bin/env python
import txros
import numpy as np
from navigator_msgs.srv import ObjectDBSingleQuery, ObjectDBSingleQueryRequest


@txros.util.cancellableInlineCallbacks
def main(navigator):
    navigator.change_wrench("autonomous")
    serv = navigator.nh.get_service_client("/database/single", ObjectDBSingleQuery)
    req = ObjectDBSingleQueryRequest()
    req.name = 'scan_the_code'
    ans = yield serv(req)
    print ans
    if not ans.found:
        return

    obj = ans.object

    stc_position = np.array([obj.position.x, obj.position.y, obj.position.z])

    diff = np.subtract(stc_position, navigator.move.position)
    diff = np.array([diff[0], diff[1], 0])
    length = np.linalg.norm(diff)
    radius = 4
    if(length - radius >= 0):
        d = navigator.move.rel_position(diff * (length - radius) / (length)).look_at(stc_position).go()
        yield d
    yield navigator.vision_request("scan_the_code_activate")
    pattern = navigator.move.circle_point(stc_position, radius + 2, granularity=8)
    searcher = navigator.search(vision_proxy='/vision/scan_the_code_status', search_pattern=pattern)
    yield searcher.start_search(spotings_req=1, speed=.9)
