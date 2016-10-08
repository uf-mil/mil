#!/usr/bin/env python
import numpy as np
import txros
import navigator_tools
from twisted.internet import defer
from navigator_msgs.srv import Bounds, BoundsResponse, \
                               CoordinateConversion, CoordinateConversionRequest


@txros.util.cancellableInlineCallbacks
def got_request(nh, req, convert):
    to_frame = "enu" if req.to_frame == '' else req.to_frame

    resp = BoundsResponse(enforce=False)
    if (yield nh.has_param("/bounds/enforce")) and (yield nh.get_param("/bounds/enforce")):
        # We want to enforce the bounds that were set
        lla_bounds = yield nh.get_param("/bounds/lla")
        bounds = []
        for lla_bound in lla_bounds:
            bound = yield convert(CoordinateConversionRequest(frame="lla", point=np.append(lla_bound, 0)))
            bounds.append(navigator_tools.numpy_to_point(getattr(bound, to_frame)))

        resp = BoundsResponse(enforce=True, bounds=bounds)

    defer.returnValue(resp)


@txros.util.cancellableInlineCallbacks
def main():
    nh = yield txros.NodeHandle.from_argv("bounds_server")

    convert = nh.get_service_client("/convert", CoordinateConversion)
    nh.advertise_service('/get_bounds', Bounds, lambda req: got_request(nh, req, convert))

    yield defer.Deferred() # never exit


txros.util.launch_main(main)