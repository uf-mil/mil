#!/usr/bin/env python
import txros
import numpy as np
from navigator_tools import fprint

@txros.util.cancellableInlineCallbacks
def main(navigator, **kwargs):
    fprint("STARTING FIND THE BREAK, GETTING A BOUY AS A TEST", msg_color="green")
    yield navigator.database_query("buoy")
    yield navigator.nh.sleep(5)
    fprint("COMPLETED FIND THE BREAK", msg_color="green")