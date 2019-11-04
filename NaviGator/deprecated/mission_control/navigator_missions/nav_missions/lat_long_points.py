#!/usr/bin/env python
import txros


@txros.util.cancellableInlineCallbacks
def main(navigator):
    while True:
        target_raw = raw_input("Lat, Long: ")
        if target_raw == 'q':
            break

        target = map(float, target_raw.split(','))
        waypoint = yield navigator.move.to_lat_long(*target)

        go = raw_input("Move {} meters? (y/n) ".format(waypoint.distance))
        if go == 'y':
            yield waypoint.go()
        elif go == 'q':
            break