#!/usr/bin/env python
from __future__ import division

import rospy
import rosbag
import txros
from sub8_msgs.srv import RunJob, RunJobRequest
from geometry_msgs.msg import PoseStamped

from twisted.internet import reactor
import signal
import numpy as np

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
    def __init__(self, nh, bag_name, time_step=.1, buffer_time=10):
        self.nh = nh

        self.time_step = time_step
        self.buffer_time = buffer_time
        self.buffer_array_size = int(buffer_time / time_step)
        self.bag_name = bag_name

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
        self.test_sub = yield self.nh.subscribe('/pose_sim', PoseStamped)

        self.cache_dict = {'/test': self.test_sub}

        while True:
            yield txros.util.wall_sleep(self.time_step)

            if self.dumping:
                yield txros.util.wall_sleep(1)

            # Add to cache
            msgs = []
            for key in self.cache_dict:
                msg = self.test_sub.get_last_message()
                msg_time = self.test_sub.get_last_message_time()
                msgs.append([key, (yield msg), (yield msg_time)])
            self.write_to_cache(msgs)

        self.dump()

    @txros.util.cancellableInlineCallbacks
    def dump(self, post_record_time=2):
        '''
        Save cached bags and save the next 'buffer_time' seconds.
        '''
        self.dumping = True
        bag = rosbag.Bag(self.bag_name, 'w')

        gen = self.read_from_cache()

        print "Saving post-fail data. Do not exit."
        for i in range(int(post_record_time / self.time_step)):
            for key in self.cache_dict:
                msg = self.test_sub.get_last_message()
                msg_time = self.test_sub.get_last_message_time()
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
        print "Done!"

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
    def __init__(self, nh, loop_count=10, timeout=60):
        self.nh = nh
        self.timeout = timeout
        self.timedout = False

        self.bagger = BagManager(self.nh, 'test.bag')
        self.loop_count = loop_count

        self.run_job = nh.get_service_client('/gazebo/job_runner/mission_start', RunJob)

    @txros.util.cancellableInlineCallbacks
    def run_job_loop(self):
        current_loop = 0

        self.bagger.start_caching()

        while current_loop < self.loop_count:
            yield txros.util.wall_sleep(2.0)

            try:
                response = yield self.run_job(RunJobRequest())
            except:
                print "No service provider found."
                continue

            current_loop += 1
            print response
            if response.success:
                print "Success"
            else:
                print "Fail"
                yield self.bagger.dump()

            print
            self.reset_gazebo()

    def reset_gazebo(self):
        '''
        Reset sub position without breaking the simulator.
        This could send a signal to the mission node so that it can set the sub position wherever it wants.
        '''
        print "Reseting Gazebo"
        return


@txros.util.cancellableInlineCallbacks
def main():
        print "Starting"
        nh = yield txros.NodeHandle.from_argv('test')
        print "nh done"
        j = JobManager(nh)

        print "j created"
        yield j.run_job_loop()

if __name__ == '__main__':
    reactor.callWhenRunning(main)
    reactor.run()