#!/usr/bin/env python
import txros
import numpy as np
from mil_misc_tools.text_effects import fprint


@txros.util.cancellableInlineCallbacks
def main(navigator, **kwargs):
    fprint("STARTING FIND THE BREAK, GETTING A BOUY AS A TEST", msg_color="green")
    # Get all objects within a certain radius of type unknown or buoy
    # Get all combinations and find the ones with the correct distance
    fprint("COMPLETED FIND THE BREAK", msg_color="green")
