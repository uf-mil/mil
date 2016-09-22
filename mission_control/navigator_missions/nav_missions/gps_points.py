#!/usr/bin/env python
import txros
import numpy as np

@txros.util.cancellableInlineCallbacks
def main(navigator):
    points = np.array([[29.534882, -82.303701], [29.534614, -82.303594], [29.534418, -82.303839], [29.534801, -82.303917], [29.534660, -82.304180], [29.534554, -82.304366]])

    for point in points:
      m = navigator.move.to_lat_long(*point)
      lookat = navigator.move.look_at(m.pose[0])
      yield lookat.go()
      yield m.set_orientation(lookat.pose[1]).go()

    print "Done!"
