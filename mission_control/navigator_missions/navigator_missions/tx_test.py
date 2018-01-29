#!/usr/bin/env python
import txros


@txros.util.cancellableInlineCallbacks
def main(navigator):
    m = navigator.move.forward(1)

    yield m.go()
    print "Done!"
