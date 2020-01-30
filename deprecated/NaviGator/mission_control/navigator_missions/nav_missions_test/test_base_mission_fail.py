#!/usr/bin/env python
import txros
from mil_misc_tools.text_effects import fprint
from twisted.internet import defer


@txros.util.cancellableInlineCallbacks
def main(navigator, **kwargs):
    nh = navigator.nh
    fprint("{} running".format(__name__), msg_color='red')
    yield nh.sleep(2)
    defer.returnValue(False)
