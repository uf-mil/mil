#!/usr/bin/env python
import txros
from navigator_tools import fprint


@txros.util.cancellableInlineCallbacks
def main(navigator, **kwargs):
    nh = navigator.nh

    fprint("{} running".format(__name__), msg_color='red')
    yield nh.sleep(6)
    fprint("{} stopped running, you should have stopped by now".format(__name__), msg_color='red')
