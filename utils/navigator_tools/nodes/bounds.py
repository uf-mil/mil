#!/usr/bin/env python
import numpy as np

import txros
import navigator_tools
from twisted.internet import defer
from coordinate_conversion_server import Converter
from navigator_msgs.srv import Bounds, BoundsResponse, \
                               CoordinateConversionRequest


class Param():
    @classmethod
    @txros.util.cancellableInlineCallbacks
    def get(cls, nh, param):
        try:
            p = yield nh.get_param(param)
            print param, p
            defer.returnValue(cls(p))

        except txros.rosxmlrpc.Error:
            # Param not found
            defer.returnValue(cls(None))

    def __init__(self, value):
        self.value = value
        self.found = value is not None
        self.found_and_true = self.value and self.found


@txros.util.cancellableInlineCallbacks
def got_request(nh, req, convert):
    to_frame = "enu" if req.to_frame == '' else req.to_frame

    resp = BoundsResponse(enforce=False)

     # Lets get all these defers started up
    enforce = yield Param.get(nh, "/bounds/enforce")
    lla_bounds = yield Param.get(nh, "/bounds/lla")

    if enforce.found_and_true:
        bounds = []
        for lla_bound in lla_bounds.value:
            bound = yield convert.request(CoordinateConversionRequest(frame="lla", point=np.append(lla_bound, 0)))
            bounds.append(navigator_tools.numpy_to_point(getattr(bound, to_frame)))

        resp = BoundsResponse(enforce=True, bounds=bounds)

    defer.returnValue(resp)


@txros.util.cancellableInlineCallbacks
def main():
    nh = yield txros.NodeHandle.from_argv("bounds_server")

    convert = Converter()
    yield convert.init(nh)
    nh.advertise_service('/get_bounds', Bounds, lambda req: got_request(nh, req, convert))

    yield defer.Deferred() # never exit

if __name__ == "__main__":
    txros.util.launch_main(main)