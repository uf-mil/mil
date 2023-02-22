#!/usr/bin/env python
import axros
from mil_misc_tools.text_effects import fprint


@axros.util.cancellableInlineCallbacks
def main(navigator, **kwargs):
    fprint("STARTING CORAL SURVEY, GETTING A TOWER BOUY AS A TEST", msg_color="green")
    yield navigator.database_query("tower")
    yield navigator.nh.sleep(5)
    fprint("COMPLETED CORAL SURVEY", msg_color="green")
