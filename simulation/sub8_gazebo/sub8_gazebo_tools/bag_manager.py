import rospy
import rosbag
import time
import txros
import os
import yaml


class BagManager(object):
    def __init__(self, nh, diag_dir):
        self.nh = nh
        self.diag_dir = diag_dir

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
                    print "There's a problem cacheing {0}".format(key)
            self.write_to_cache(msgs)

        self.dump()

    @txros.util.cancellableInlineCallbacks
    def make_dict(self):
        '''
        Generate caching dictionary from a yaml file.
        '''
        with open(self.diag_dir + 'messages_to_bag.yaml', 'r') as f:
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
        # Make bag directory if it doesn't exist
        directory = os.path.join(self.diag_dir, 'bags')
        if not os.path.exists(directory):
            os.makedirs(directory)

        bag = rosbag.Bag(os.path.join(directory, str(int(time.time())) + '.bag'), 'w')

        gen = self.read_from_cache()

        last_timestamps = dict(self.cache_dict)

        print "Saving post-fail data. Do not exit."
        for i in range(int(self.post_cache_time / self.time_step)):
            for key in self.cache_dict:
                msg = self.cache_dict[key].get_last_message()
                msg_time = self.cache_dict[key].get_last_message_time()
                if msg is not None:
                    if msg_time < self.nh.get_time() - rospy.Duration(.5):
                        # If the message is too old, forget about it.
                        continue

                    # Lang -- this doesn't work
                    if msg_time == last_timestamps[key]:
                        continue
                    # last_timestamps[key] = msg_time
                    bag.write(key, (yield msg), t=(yield msg_time))
                    # msgs.append([key, (yield msg), (yield msg_time)])
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
