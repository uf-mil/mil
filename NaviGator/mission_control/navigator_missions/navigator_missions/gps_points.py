#!/usr/bin/env python
import txros
import numpy as np
from navigator import Navigator


class GPSWaypoints(Navigator):
    DEFAULT_WAYPOINTS = np.array([[29.534882, -82.303701], [29.534614, -82.303594], [29.534418, -82.303839],
                                  [29.534801, -82.303917], [29.534660, -82.304180], [29.534554, -82.304366]])

    def verify_parameters(self, parameters):
        arr = np.array(parameters, dtype=np.float)
        if len(arr.shape) == 1:  # In case just one point is given
            arr = np.array([parameters], dtype=np.float)
        if len(arr.shape) != 2 or arr.shape[1] != 2:
            raise Exception('parameters must be a Nx2 array')
        return arr

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        if parameters == '':
            parameters = self.DEFAULT_WAYPOINTS
        points = self.verify_parameters(parameters)
        for point in points:
            self.send_feedback('Going to point {}'.format(point))
            m = self.move.to_lat_long(*point)
            lookat = self.move.look_at(m.pose[0])
            yield lookat.go()
            yield m.set_orientation(lookat.pose[1]).go()
