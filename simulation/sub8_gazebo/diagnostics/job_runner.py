#!/usr/bin/env python
from __future__ import division

import rospy
import rosbag
import rospkg
import txros
from sub8 import tx_sub

from sub8_gazebo.srv import RunJob, RunJobRequest
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from gazebo_msgs.msg import ModelState
from kill_handling.srv import SetKill, SetKillRequest
from kill_handling.msg import Kill

from twisted.internet import reactor
import signal
import numpy as np
import time
import os
import yaml

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


class BagManager(object):
    def __init__(self, nh):
        self.nh = nh

        self.make_dict()
        self.buffer_array_size = int(self.pre_cache_time / self.time_step)

        self.cache_index = 0
        self.cache = [None] * self.buffer_array_size
        self.dumping = False

    @txros.util.cancellableInlineCallbacks
    def start_caching(self):
        '''
        Caches bags using big dicts.
        {topic_name_1: subscriber_1, topic_name_2: subscriber_2, ...}
        Every 'time_step' seconds it will save all of the subscribers' most recent message. This will continue and will fill
            a cache array that will store 'pre_cache_time' seconds worth of messages before erasing the oldest messages.
        '''

        # Used to avoid adding copies of the same message.
        last_timestamps = dict(self.cache_dict)
        while True:
            yield self.nh.sleep(self.time_step)

            if self.dumping:
                yield self.nh.sleep(1)

            # Add to cache
            msgs = []
            for key in self.cache_dict:
                msg = self.cache_dict[key].get_last_message()
                msg_time = yield self.cache_dict[key].get_last_message_time()

                if msg is not None:
                    if (self.nh.get_time() > rospy.Time.from_sec(30)) and (msg_time < self.nh.get_time() - rospy.Duration(30)):
                        # This fixes a negative time exception if we are within the first 30 seconds of time.
                        continue

                    if msg_time == last_timestamps[key]:
                        continue
                    last_timestamps[key] = msg_time

                    msgs.append([key, (yield msg), (yield msg_time)])
                else:
                    print "There's a problem recording {0}".format(key)
            self.write_to_cache(msgs)

        self.dump()

    @txros.util.cancellableInlineCallbacks
    def make_dict(self):
        '''
        Generate caching dictionary from a yaml file.
        '''
        with open(DIAG_OUT_DIR + 'messages_to_bag.yaml', 'r') as f:
            messages_to_bag = yaml.load(f)

        # Set bagging parameters
        self.time_step = messages_to_bag['PARAMS']['time_step']
        self.pre_cache_time = messages_to_bag['PARAMS']['pre_cache_time']
        self.post_cache_time = messages_to_bag['PARAMS']['post_cache_time']

        self.cache_dict = {}
        msgs = messages_to_bag['MESSAGES']
        for msg in msgs.values():
            # Get message information
            msg_topic = msg['message_topic']
            msg_type = msg['message_type']
            msg_name = msg['message_name']

            # Import the message
            exec("from {0}.msg import {1}".format(msg_type, msg_name))

            # Create subscriber and add to dictionary
            self.cache_dict[msg_topic] = yield self.nh.subscribe(msg_topic, eval(msg_name))

    @txros.util.cancellableInlineCallbacks
    def dump(self):
        '''
        Save cached bags and save the next 'post_cache_time' seconds.
        '''
        self.dumping = True
        bag = rosbag.Bag(DIAG_OUT_DIR + 'bags/' + str(int(time.time())) + '.bag', 'w')

        gen = self.read_from_cache()

        print "Saving post-fail data. Do not exit."
        for i in range(int(self.post_cache_time / self.time_step)):
            for key in self.cache_dict:
                msg = self.cache_dict[key].get_last_message()
                msg_time = self.cache_dict[key].get_last_message_time()
                if msg is not None:
                    if msg_time < self.nh.get_time() - rospy.Duration(.5):
                        # If the message is too old, forget about it.
                        continue

                    if msg_time == last_timestamps[key]:
                        continue
                    last_timestamps[key] = msg_time

                    msgs.append([key, (yield msg), (yield msg_time)])
                else:
                    print "There's a problem recording {0}".format(key)

            yield self.nh.sleep(self.time_step)

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
    def __init__(self, nh, loop_count=10000):
        self.nh = nh
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