#!/usr/bin/env python
import txros
import rospy
import std_srvs.srv
from navigator_msgs.msg import DockShape

error_threshold = 0.01
move_per_iteration = 1.0;

@txros.util.cancellableInlineCallbacks
def main(navigator):
    shooterFire = rospy.ServiceProxy("/shooter/fire", std_srvs.srv.Trigger)
    
    try:    
        resp = yield navigator.vision_request("get_shape")
        error = float(resp.symbol.CenterX)/resp.symbol.img_width - 0.5
        print error
    except:
        print "error excepted"
        error = 1
    
    while abs(error) > error_threshold:
        try:
            resp = yield navigator.vision_request("get_shape")
            error = float(resp.symbol.CenterX)/resp.symbol.img_width - 0.5
            print error
            if error < 0:
                yield navigator.move.forward(move_per_iteration).go()
                print "Moving Forward"
            elif error > 0:
                yield navigator.move.backward(move_per_iteration).go()
                print "Moving Backward"
        except:
            print "Erorr excepted"
    yield navigator.move.go()
    shooterFire()
