#!/usr/bin/env python
import txros
from navigator_tools import fprint, MissingPerceptionObject


@txros.util.cancellableInlineCallbacks
def main(navigator, attempts):
    fprint("{} running".format(__name__), msg_color='red')
    yield navigator.nh.sleep(1)
    if attempts == 1:
        fprint("Raising Perception Object", msg_color='red')
        raise MissingPerceptionObject("stc")
    else:
        fprint("{} stopped running".format(__name__), msg_color='red')


def safe_exit(navigator, err):
    print "SAFE EXIT"
