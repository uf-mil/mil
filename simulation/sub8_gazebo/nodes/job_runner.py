#!/usr/bin/env python
from __future__ import division

import rospy
import rosbag
import txros
from sub8_msgs.srv import RunJob
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
    def __init__(self, nh, time_step=.1, buffer_time=10):
        print "bags"
        self.nh = nh
        self.bag = rosbag.Bag('test.bag', 'w')

        self.time_step = time_step
        self.buffer_time = buffer_time
        self.buffer_array_size = int(buffer_time / time_step)

        self.cache_dict = {}
        self.cache_index = 0
        self.cache = np.empty(self.buffer_array_size, dtype=object)

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

        for i in range(self.buffer_array_size):
            yield txros.util.wall_sleep(self.time_step)

            # Add to cache
            print "Caching most recent messages."
            msgs = []
            for key in self.cache_dict:
                msg = self.test_sub.get_last_message()
                msg_time = self.test_sub.get_last_message_time()
                msgs.append([key, (yield msg), (yield msg_time)])

            self.write_to_cache(msgs)

        self.dump()

    def dump(self, pose_record_time=0):
        '''
        Save cached bags and save the next 'buffer_time' seconds.
        '''
        gen = self.read_from_cache()

        for i in range(self.buffer_array_size):
            msgs = next(gen)
            if msgs is not None:
                for msg in msgs:
                    self.bag.write(msg[0], msg[1], t=msg[2])

        self.bag.close()
        print "Done!"

    def write_to_cache(self, data):
        self.cache[self.cache_index] = data
        print self.cache

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
    @txros.util.cancellableInlineCallbacks
    def __init__(self, loop_count=10, timeout=60):
        self.nh = yield txros.NodeHandle.from_argv('gazebo_job_manager')
        next_pose = self.nh.get_service_client('/gazebo/job_runner/mission_start', RunJob)

        self.bagger = BagManager(self.nh)
        self.loop_count = loop_count

    @txros.util.cancellableInlineCallbacks
    def run_job(self):
        current_loop = 0

        while current_loop < self.loop_count:
            yield txros.util.wall_sleep(2.0)

            response = yield

    def reset_gazebo(self):
        '''
        Reset sub position without breaking the simulator.
        This could send a signal to the mission node so that it can set the sub position wherever it wants.
        '''
        return


@txros.util.cancellableInlineCallbacks
def main():
        print "Starting"
        nh = yield txros.NodeHandle.from_argv('test')
        print "nh done"
        b = BagManager(nh)
        print "Bag init done"
        yield b.start_caching()

if __name__ == '__main__':
    reactor.callWhenRunning(main)
    reactor.run()