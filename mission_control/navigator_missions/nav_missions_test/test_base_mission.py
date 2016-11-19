#!/usr/bin/env python
import txros
from navigator_tools import fprint
from twisted.internet import defer


@txros.util.cancellableInlineCallbacks
def main(navigator):
    nh = navigator.nh
    fprint("{} running1".format(__name__), msg_color='red')
    yield nh.sleep(2)
    fprint("{} running2".format(__name__), msg_color='red')
    yield nh.sleep(2)
    fprint("{} running3".format(__name__), msg_color='red')
    yield nh.sleep(3)
    fprint("{} stopped running".format(__name__), msg_color='red')
    defer.returnValue(True)
