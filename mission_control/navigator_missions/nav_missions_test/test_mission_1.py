#!/usr/bin/env python
import txros
from navigator_tools import fprint


@txros.util.cancellableInlineCallbacks
def main(navigator):
    nh = navigator.nh
    fprint("{} running".format(__name__), msg_color='red')
    yield nh.sleep(5)
    fprint("{} stopped running".format(__name__), msg_color='red')
