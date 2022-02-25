#!/usr/bin/env python
from __future__ import division
import txros
import numpy as np
from twisted.internet import defer
from vrx import Vrx
from mil_tools import rosmsg_to_numpy
from std_srvs.srv import SetBoolRequest
from mil_tools import quaternion_matrix
from mil_msgs.srv import ObjectDBQuery, ObjectDBQueryRequest, ObjectDBQueryResponse

___author___ = "Kevin Allen"


class VrxNavigation2(Vrx):
    def __init__(self, *args, **kwargs):
        super(VrxNavigation2, self).__init__(*args, **kwargs)

    

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        '''
        TODO: check for new objects in background, cancel move
              somefucking how handle case where gates litteraly loop back and head the other direction
        '''
        self.objects_passed = set()
        # Wait a bit for PCDAR to get setup
        yield self.nh.sleep(3.0)
        yield self.set_vrx_classifier_enabled(SetBoolRequest(data=True))
        
        service_response = ObjectDBQueryRequest()
        service_response.name = "all"

        response = yield self.database_response(service_response)
        
        print("length of objects array: " + str(len(response.objects)))
        for i in range(len(response.objects)):
            print(response.objects[i].pose)

        yield self.move.forward(10, 'm').go()

        response = yield self.database_response(service_response)
        print("length of objects array: " + str(len(response.objects)))
        for i in range(len(response.objects)):
            print(response.objects[i].pose)

        yield self.move.yaw_left(45, "deg").go()

        response = yield self.database_response(service_response)
        print("length of objects array: " + str(len(response.objects)))
        for i in range(len(response.objects)):
            print(response.objects[i].pose)

        yield self.move.forward(10, 'm').go()

        response = yield self.database_response(service_response)
        print("length of objects array: " + str(len(response.objects)))
        for i in range(len(response.objects)):
            print(response.objects[i].pose)

        #yield self.prepare_to_enter()
        #yield self.wait_for_task_such_that(lambda task: task.state =='running')
        #yield self.move.forward(7.0).go()
        #while not (yield self.do_next_gate()):
        #    pass
        #self.send_feedback('This is the last gate! Going through!')
        #yield self.move.forward(10).go()
        #yield self.set_vrx_classifier_enabled(SetBoolRequest(data=False))

