#!/usr/bin/env python
import txros
from navigator_tools import fprint
from twisted.internet import defer


@txros.util.cancellableInlineCallbacks
def main(navigator, **kwargs):
    nh = navigator.nh
    fprint("{} running".format(__name__), msg_color='red')
    yield nh.sleep(2)
    defer.returnValue(False)
