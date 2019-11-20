#!/usr/bin/env python
from __future__ import division
import txros
import numpy as np
from twisted.internet import defer
from vrx import Vrx
from mil_tools import rosmsg_to_numpy

___author___ = "Kevin Allen"


class VrxPerception(Vrx):
    def __init__(self, *args, **kwargs):
        super(VrxPerception, self).__init__(*args, **kwargs)

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        print 'perception'
        yield self.wait_for_task_such_that(lambda task: task.state in ['running'])
        while True:
            msg = yield self.task_info_sub.get_next_message()
            objects = yield self.database_query()
            print objects
            print msg.elapsed_time
