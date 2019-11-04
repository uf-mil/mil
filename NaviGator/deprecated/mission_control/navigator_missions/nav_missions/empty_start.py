#!/usr/bin/env python
import txros
import numpy as np
from mil_misc_tools.text_effects import fprint

@txros.util.cancellableInlineCallbacks
def main(navigator, **kwargs):
    fprint("STARTING EMPTY TEST", msg_color="green")
    yield navigator.nh.sleep(5)
    fprint("COMPLETED EMPTY TEST", msg_color="green")