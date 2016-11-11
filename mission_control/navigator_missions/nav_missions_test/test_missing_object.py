#!/usr/bin/env python
import txros
from navigator_tools import fprint
from navigator_tools import MissingPerceptionObject


@txros.util.cancellableInlineCallbacks
def main(navigator):
    nh = navigator.nh

    fprint("{} running".format(__name__), msg_color='red')
    yield nh.sleep(3)
    fprint("{} stopped running, raising exception".format(__name__), msg_color='red')

    raise MissingPerceptionObject("scan_the_code")
