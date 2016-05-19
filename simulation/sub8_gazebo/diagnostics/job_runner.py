#!/usr/bin/env python
from __future__ import division

import rospkg
import txros
from sub8_gazebo_tools import BagManager
from sub8 import tx_sub

from sub8_gazebo.srv import RunJob, RunJobRequest
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from gazebo_msgs.msg import ModelState
from kill_handling.srv import SetKill, SetKillRequest
from kill_handling.msg import Kill

from twisted.internet import reactor
import numpy as np
import time
import os

rospack = rospkg.RosPack()
DIAG_OUT_DIR = os.path.join(rospack.get_path('sub8_gazebo'), 'diagnostics/')

'''
In its current implementation, to run a mission you should create a mission script that listens to
'/gazebo/job_runner/mission_start' for a start service. Right now the service doesn't provide anything but could
easily provide something to the mission file if we want to. The mission script should return with the status of that run
(pass or fail) and perhapse a short message that will be logged. When this script gets that signal, it will reset the gazebo
world and run the same mission again. The plan is to dynamically bag data by keeping a cache and if something goes wrong keep
the bag. Mission files should be able to catch errors and report them back to this script for logging.

FUTURE:
 - Job Queue
'''


class JobManager(object):
    def __init__(self, nh, loop_count=10000):
        self.nh = nh
        self.timedout = False

        self.bagger = BagManager(self.nh, DIAG_OUT_DIR)
        self.loop_count = loop_count
        self.successes = 0
        self.fails = 0

        self.run_job = nh.get_service_client('/gazebo/job_runner/mission_start', RunJob)


    @txros.util.cancellableInlineCallbacks
    def run_job_loop(self):
        print "Looking for jobs..."
        current_loop = 0

        self.bagger.start_caching()

        while current_loop < self.loop_count:
            yield self.nh.sleep(2.0)
            start_time = self.nh.get_time()

            try:
                response = yield self.run_job(RunJobRequest())
            except:
                print "No service provider found."
                continue

            current_loop += 1
            print response
            if response.success:
                print "Success"
                self.successes += 1
            else:
                print "Fail"
                print
                self.fails += 1
                yield self.bagger.dump()

            print "Writing report to file. Do not exit."
            elapsed = self.nh.get_time() - start_time
            try:
                with open(DIAG_OUT_DIR + 'diag.txt', 'a') as f:
                    f.write("Response: {0} occured at {1} (Duration: {2}).\n".format(response.success, int(time.time()), elapsed))
                    f.write(response.message + '\n')
                    f.write("{0} Successes, {1} Fails, {2} Total \n.".format(self.successes, self.fails, current_loop))
                    f.write("-----------------------------------------------------------")
                    f.write('\n')
            except IOError:
                print "Diagnostics file not found. Creating it now."
                with open(DIAG_OUT_DIR + 'diag.txt', 'w') as f:
                    f.write("Response: {0} occured at {1} (Duration: {2}).\n".format(response.success, int(time.time()), elapsed))
                    f.write(response.message + '\n')
                    f.write("{0} Successes, {1} Fails, {2} Total \n.".format(self.successes, self.fails, current_loop))
                    f.write("-----------------------------------------------------------")
                    f.write('\n')
            print
            print "Done!"

            print "------------------------------------------------------------------"
            print "{0} Successes, {1} Fails, {2} Total".format(self.successes, self.fails, current_loop)
            print "------------------------------------------------------------------"
            print

            if current_loop > 5 and self.fails / current_loop > .9:
                print "Too many errors. Stopping as to not fill HDD with bags."
                self.loop_count = 0
            #self.reset_gazebo()
        print
        print "Job Finished!"
        print "Writing report to file. Do not exit."
        with open(DIAG_OUT_DIR + 'diag.txt', 'a') as f:
            f.write("{0} Successes, {1} Fails, {2} Total \n.".format(self.successes, self.fails, current_loop))
            f.write("Time of completion: {0}. \n".format(int(time.time())))
            f.write("-----------------------------------------------------------")
            f.write('\n')

    def reset_gazebo(self):
        '''
        Reset sub position without breaking the simulator.
        This could send a signal to the mission node so that it can set the sub position wherever it wants.
        '''
        print "Reseting Gazebo"
        return

    @staticmethod
    @txros.util.cancellableInlineCallbacks
    def set_model_position(nh, pose, twist=None, model='sub8'):
        '''
        Set the position of 'model' to 'pose'.
        It may be helpful to kill the sub before moving it.

        TODO: Make this a service? Might as well just use the ros service gazebo provides? Maybe not.
        '''
        set_state = nh.get_service_client('/gazebo/set_model_state', SetModelState)

        if twist is None:
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0

        model_state = ModelState()
        model_state.model_name = model
        model_state.pose = pose
        model_state.twist = twist

        if model == 'sub8':
            kill = nh.get_service_client('/set_kill', SetKill)
            yield kill(SetKillRequest(kill=Kill(id='initial', active=False)))
            yield kill(SetKillRequest(kill=Kill(active=True)))
            yield nh.sleep(.1)
            yield set_state(SetModelStateRequest(model_state))
            yield nh.sleep(.1)
            yield kill(SetKillRequest(kill=Kill(active=False)))
        else:
            set_state(SetModelStateRequest(model_state))


@txros.util.cancellableInlineCallbacks
def main():
        print "Starting Job Runner."
        nh = yield txros.NodeHandle.from_argv('job_runner_controller')
        j = JobManager(nh)
        yield j.run_job_loop()

if __name__ == '__main__':
    reactor.callWhenRunning(main)
    reactor.run()
