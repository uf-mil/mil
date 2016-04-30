#!/usr/bin/env python
from __future__ import division

import rospy
import rosbag
import rospkg
import txros
from sub8_msgs.srv import RunJob, RunJobRequest
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from gazebo_msgs.msg import ModelState
from sub8 import tx_sub
from kill_handling.srv import SetKill, SetKillRequest
from kill_handling.msg import Kill

from twisted.internet import reactor
import signal
import numpy as np
import time
import os

rospack = rospkg.RosPack()
DIAG_OUT_URI = os.path.join(rospack.get_path('sub8_gazebo'), 'diagnostics/')

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


class BagManager(object):
    def __init__(self, nh, time_step=.5, buffer_time=30):
        self.nh = nh

        self.time_step = time_step
        self.buffer_time = buffer_time
        self.buffer_array_size = int(buffer_time / time_step)
        #self.bag_name = DIAG_OUT_URI + bag_name

        self.cache_dict = {}
        self.cache_index = 0
        self.cache = [None] * self.buffer_array_size
        self.dumping = False

    @txros.util.cancellableInlineCallbacks
    def start_caching(self):
        '''
        Caches bags using big dicts.
        {topic_name_1: subscriber_1, topic_name_2: subscriber_2, ...}
        Every 'time_step' seconds it will save all of the subscribers' most recent message. This will continue and will fill
            a cache array that will store 'buffer_time' seconds worth of messages before erasing the oldest messages.

        TODO: Make sub list come from a YAML using eval() to define message types.
        '''
        # Subscribers here
        self.odom_sub = yield self.nh.subscribe('/odom', Odometry)
        #self.alarms_sub = yield self.nh.subscribe('/odom', Odometry)
        self.c3_goal = yield self.nh.subscribe('/c3_trajectory_generator/waypoint', PoseStamped)

        self.cache_dict = {'/odom': self.odom_sub,
                           '/c3_trajectory_generator/waypoint': self.c3_goal}

        while True:
            yield txros.util.wall_sleep(self.time_step)

            if self.dumping:
                yield txros.util.wall_sleep(1)

            # Add to cache
            msgs = []
            for key in self.cache_dict:
                msg = self.cache_dict[key].get_last_message()
                msg_time = self.cache_dict[key].get_last_message_time()
                if msg is not None:
                    msgs.append([key, (yield msg), (yield msg_time)])
                else:
                    print "There's a problem recording {0}".format(key)
            self.write_to_cache(msgs)

        self.dump()

    @txros.util.cancellableInlineCallbacks
    def dump(self, post_record_time=1):
        '''
        Save cached bags and save the next 'buffer_time' seconds.
        '''
        self.dumping = True
        bag = rosbag.Bag(DIAG_OUT_URI + str(int(time.time())) + '.bag', 'w')

        gen = self.read_from_cache()

        print "Saving post-fail data. Do not exit."
        for i in range(int(post_record_time / self.time_step)):
            for key in self.cache_dict:
                msg = self.cache_dict[key].get_last_message()
                msg_time = self.cache_dict[key].get_last_message_time()
                if msg is not None:
                    bag.write(key, (yield msg), (yield msg_time))

            yield txros.util.wall_sleep(self.time_step)

        print "Saving pre-fail data. Do not exit."
        # We've written the post crash stuff now let's write the pre-crash data.
        for i in range(self.buffer_array_size):
            msgs = next(gen)
            if msgs is not None:
                for msg in msgs:
                    bag.write(msg[0], msg[1], t=msg[2])
        bag.close()
        self.dumping = False

    def write_to_cache(self, data):
        self.cache[self.cache_index] = data

        self.cache_index += 1
        if self.cache_index >= self.buffer_array_size:
            self.cache_index = 0

    def read_from_cache(self):
        index = self.cache_index
        while True:
            yield self.cache[index]
            index += 1
            if index >= self.buffer_array_size:
                index = 0


class JobManager(object):
    def __init__(self, nh, loop_count=100, timeout=15):
        self.nh = nh
        self.timeout = timeout
        self.timedout = False

        self.bagger = BagManager(self.nh)
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
            yield txros.util.wall_sleep(2.0)

            try:
                response = yield self.run_job(RunJobRequest())
            except:
                print "No service provider found."
                continue
            print "Job Found!"

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
                try:
                    with open(DIAG_OUT_URI + 'diag.txt', 'a') as f:
                        f.write("Status: {0} occured at {1} .\n".format(response.success, int(time.time())))
                        f.write(response.message + '\n')
                        f.write("-----------------------------------------------------------")
                        f.write("\n")
                except IOError:
                    print "Diagnostics file not found. Creating it now."
                    with open(DIAG_OUT_URI + 'diag.txt', 'w') as f:
                        f.write("Status: {0} occured at {1} .\n".format(response.success, int(time.time())))
                        f.write(response.message + '\n')
                        f.write("-----------------------------------------------------------")
                        f.write("\n")
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
        with open(DIAG_OUT_URI + 'diag.txt', 'a') as f:
            f.write("{0} Successes, {1} Fails, {2} Total \n.".format(self.successes, self.fails, current_loop))
            f.write("Time of completion: {0}. \n".format(int(time.time())))
            f.write("-----------------------------------------------------------")
            f.write("\n")

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
        print "Setting position"
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
            yield kill(SetKillRequest(kill=Kill(active=True)))
            yield txros.util.wall_sleep(.1)
            yield set_state(SetModelStateRequest(model_state))
            yield txros.util.wall_sleep(.1)
            yield kill(SetKillRequest(kill=Kill(active=False)))
        else:
            print "Set"
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