#!/usr/bin/env python
import txros
from navigator_tools import fprint
import numpy as np


@txros.util.cancellableInlineCallbacks
def main(navigator, attempts):
    nh = navigator.nh
    fprint("{} running".format(__name__), msg_color='red')
    yield nh.sleep(15)
    fprint("{} stopped running".format(__name__), msg_color='red')
