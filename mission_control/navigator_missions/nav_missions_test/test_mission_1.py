#!/usr/bin/env python
import txros
from navigator_tools import fprint


@txros.util.cancellableInlineCallbacks
def main(navigator):
    nh = navigator.nh
    fprint("{} running".format(__name__), msg_color='red')
    yield nh.sleep(1)
    fprint("{} stopped running".format(__name__), msg_color='red')
