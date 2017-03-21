#!/usr/bin/env python

from __future__ import division 
import rospy
import rosbag
import rostopic
from std_srvs.srv import SetBool
import os
import rospkg
import time
import math
import resource
from collections import deque
import itertools

"""
To call this service call the '/online_bagger' service with the SetBool Parameter set to True.
Doing this will generate a rosbag with the last x number of seconds saved

for example:

    bagging_server = rospy.ServiceProxy('/online_bagger', SetBool)
    bag_status = bagging_server(True)
"""


class OnlineBagger(object):

    def __init__(self):
        """
        - Make dictionary of dequeues.
        - Subscribe to set of topics defined by the yaml file in directory
        - Have a streaming option that can be turned on and off in the callback
        - Bag last x seconds of data with client
        """

        self.n = 0  # number of iterations
        self.streaming = True
        self.dumping = False

        self.get_params()
        self.make_dicts()
        self.subscribe()
        rospy.loginfo('subscriber success: %s', self.subscriber_list)

        self.bagging_service = rospy.Service('/online_bagger/dump', SetBool, 
            self.start_bagging)

    def get_params(self):
        """Retrieve parameters from param server."""

        self.topics = rospy.get_param('/online_bagger/message_topics')
        self.ram_limit = rospy.get_param('/online_bagger/ram_limit') * 1.073e9  # gigabytes to bytes
        self.stream_time = rospy.get_param('/online_bagger/stream_time')
        self.dir = rospy.get_param('/online_bagger/bag_package_path')

        rospy.loginfo('stream_time: %s seconds', self.stream_time)
        rospy.loginfo('ram_limit: %f gb', rospy.get_param('/online_bagger/ram_limit'))

    def make_dicts(self):
        """
        make dictionaries with deques() that will be filled with topics
        with all of the topics defined by the yaml.
        """
        class sliceable_deque(deque):
            def __getitem__(self, index):
                if isinstance(index, slice):
                    return type(self)(itertools.islice(self, index.start,
                                                       index.stop, index.step))
                return deque.__getitem__(self, index)


        self.topic_messages = {}
        self.subscriber_list = []  

        for topic in self.topics:
            self.topic_messages[topic] = sliceable_deque(deque())
            self.subscriber_list.append([topic, False])

        rospy.loginfo('failed topics: %s', self.subscriber_list)

    def subscribe(self):
        """
        Immediately initiated with instance of the BagStream class:
        subscribes to the set of topics defined in the yaml configuration file

        Function checks subscription status True/False of each topic
        if True: topic has already been sucessfully subscribed to
        if False: topic still needs to be subscribed to and
        subscriber will be run.

        Each element in self.subscriber list is a list [topic, Bool]
        where the Bool tells the current status of the subscriber (sucess/failure).
        """

        for topic in self.subscriber_list:
            if topic[1]:
                pass
            else:
                msg_class = rostopic.get_topic_class(topic[0])
                if msg_class[1] == None:
                    pass
                else:
                    rospy.Subscriber(topic[0], msg_class[0], 
                        lambda msg, _topic=topic[0]: self.bagger_callback(msg, _topic))

                    topic[1] = True  # successful subscription

    def get_oldest_topic_time(self, topic):
        """
        Returns oldest timestamp for a given topic as a rospy.Time type
        """
        return self.topic_messages[topic][0][0]

    def get_newest_topic_time(self, topic):
        """
        Returns newest timestamp for a given topic as a rospy.Time type
        """
        return self.topic_messages[topic][-1][0]

    def get_topic_duration(self, topic):
        """
        Returns current time duration of topic 
        """

        return self.get_newest_topic_time(topic) - self.get_oldest_topic_time(topic)

    def get_header_time(self, msg):
        if hasattr(msg, 'header'):
            return msg.header.stamp
        else:
            return rospy.get_rostime()
       
    def get_ram_usage(self):
        """
        return current ram usage
        """
        self.usage = resource.getrusage(resource.RUSAGE_SELF)
        self.ram_usage = getattr(self.usage, 'ru_maxrss')


        return self.ram_usage*1e3

    def set_ram_limit(self, topic):
        ''' Limit ram usage by removing oldest messages '''

        if self.get_ram_usage() > self.ram_limit:
            self.topic_messages[topic].popleft()

    def bagger_callback(self, msg, topic):
        """
        streaming callback function, stops streaming during bagging process
        also pops off msgs from dequeue if length of stream is longer than
        stream time specified in yaml parameters

        stores all data in a dictionary.
        the keys are topic names
        the values are dequeues of tuples
        each tuple is a time stamp of rostime and the message for that topic

        if any errors are occuring you can uncomment the lines with
        if self.n % 50  == 0 to display every 50th message, time diff and topic
        this will not give every 50th message on each topic, it provides every
        50th message accross all topics.

        stream, callback function does nothing if streaming is not active

        Continually resubscribes to any failed topics.
        """


        if not(self.streaming):
            return

        self.n = self.n + 1
        time = self.get_header_time(msg)

        self.topic_messages[topic].append((time, msg))

        # verify streaming is popping off and recording topics
        # if not self.n % 100:
        #     rospy.loginfo('time_diff: %s', time_diff.to_sec())
        #     rospy.loginfo('topic: %s', topic)
        #     rospy.loginfo('topic type: %s', type(msg))
        #     rospy.loginfo('ram size: %s', self.ram_usage/1e3)
        #     rospy.loginfo('number of messages: %s', self.get_topic_message_count(topic))

        self.set_ram_limit(topic)

        self.subscribe()

    def get_subscribed_topics(self):

        return 'ooka'

    def get_topic_message_count(self, topic):
        """
        Return number of messages available in a topic
        """
        return len(self.topic_messages[topic])

    def get_all_message_count(self):
        # returns total number of messages across all topics
        message_count = 0
        for topic in self.topic_messages.keys():
            message_count = message_count + get_topic_message_count(topic)
            return message_count

    def set_bag_directory(self):
        """
        create ros bag save directory 
        """
        rospack = rospkg.RosPack()
        self.directory = os.path.join(rospack.get_path(self.dir), 'bags/')

        rospy.loginfo('bag directory: %s', self.directory)

        if not os.path.exists(self.directory):
            os.makedirs(self.directory)

    def get_time_index(self, topic, requested_seconds):
        """
        Returns the index for the time index for a topic at 'n' seconds from the end of the dequeue
        
        For example, to bag the last 10 seconds of data, the index for 10 seconds back from the most
        recent message can be obtained with this function
        
        If the desired time length of the bag is greater than the available messages it will output a 
        message and return how ever many seconds of data are avaiable at the moment.

        seconds is of a number type (not a rospy.Time type) (ie. int, float)
        """

        message_count = self.get_topic_message_count(topic)

        topic_duration = self.get_topic_duration(topic).to_sec()

        ratio = requested_seconds / topic_duration
        
        index = math.floor(message_count * (1 - min(ratio, 1))) 

        self.bag_report = ('The requested %s seconds were bagged', requested_seconds)

        if not index:
            self.bag_report = ('Only %s seconds of the request %s seconds were available, all messages were bagged', topic_duration, requested_seconds)
        return int(index)
 
    def start_bagging(self, req):
        """
        Dump all data in dictionary to bags, temporarily stops streaming
        during the bagging process, resumes streaming when over.
        """
        self.dumping = req.data
        self.streaming = False

        self.set_bag_directory()

        bag = rosbag.Bag(os.path.join(self.directory, str(int(time.time())) + '.bag'), 'w')

        rospy.loginfo('dumping value: %s', self.dumping)
        rospy.loginfo('bagging commencing!')

        self.requested_seconds = int(10)

        for topic in self.topic_messages.keys():
            rospy.loginfo('topic: %s', topic)
            self.i = self.get_time_index(topic, self.requested_seconds)
            self.m = 0  # message number in a given topic
            for msgs in self.topic_messages[topic][self.i:]:
                self.m = self.m + 1
                bag.write(topic, msgs[1], t=msgs[0])
                if self.m % 100 == 0:  # print every 100th topic, type and time
                    rospy.loginfo('topic: %s, message type: %s, time: %s', 
                        topic, type(msgs[1]), type(msgs[0]))

        rospy.loginfo('Bag Report: %s', self.bag_report)
        bag.close()
        rospy.loginfo('bagging finished!')
        self.streaming = True

        message = 'bagging finished'
        return (True, message)

if __name__ == "__main__":
    rospy.init_node('online_bagger')
    stream = OnlineBagger()
    rospy.spin()
