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
import datetime

"""
To call call service call the '/online_bagger' service with the SetBool
Parameter set to True. Doing this will generate a rosbag with the
last x number of seconds saved

for example:

    bagging_server = rospy.ServiceProxy('/online_bagger', SetBool)
    bag_status = bagging_server(True)
"""


class OnlineBagger(object):

    def __init__(self):
        """
        Make dictionary of dequeues.
        Subscribe to set of topics defined by the yaml file in directory
        Stream topics up to a given ram limit, dump oldest messages when limit is reached
        Set up service to bag n seconds of data default to all of available data
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

        """
        Retrieve parameters from param server.
        """

        self.topics = rospy.get_param('/online_bagger/message_topics')
        self.ram_limit = rospy.get_param('/online_bagger/ram_limit') * 1.073e9  # gigabytes to bytes
        # self.stream_time = rospy.get_param('/online_bagger/stream_time')
        self.dir = rospy.get_param('/online_bagger/bag_package_path')
        self.bag_name = rospy.get_param('/online_bagger/bag_name')

        if not self.bag_name:
            self.bag_name = str(datetime.date.today()) + '-' + str(datetime.datetime.now().time())[0:8]

        # rospy.loginfo('stream_time: %s seconds', self.stream_time)
        rospy.loginfo('ram_limit: %f gb', rospy.get_param('/online_bagger/ram_limit'))

    def make_dicts(self):

        """
        Make dictionary with sliceable deques() that will be filled with messages and time stamps.

        Subscriber list contains all of the topics and their subscription status:
        A True status for a given topic corresponds to a successful subscription
        A False status indicates a failed subscription

        For Example:
        self.subscriber_list[0] = [['/odom', False], ['/absodom', True]]

        Indicates that '/odom' has not been subscribed to, but '/absodom' has been subscribed to

        self.topic_messages is a dictionary of deques containing a list of tuples.
        Dictionary Keys contain topic names
        Each value for each topic contains a deque
        Each deque contains a list of tuples
        Each tuple contains a message and its associated time stamp

        For example:
        '/odom' is  a potential topic name
        self.topic_message['/odom']  is a deque
        self.topic_message['/odom'][0]  is the oldest message available in the deque
        and its time stamp if available
        """

        # Courtesy Kindall
        # http://stackoverflow.com/questions/10003143/how-to-slice-a-deque
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
        Subscribe to the topics defined in the yaml configuration file

        Function checks subscription status True/False of each topic
        if True: topic has already been sucessfully subscribed to
        if False: topic still needs to be subscribed to and
        subscriber will be run.

        Each element in self.subscriber list is a list [topic, Bool]
        where the Bool tells the current status of the subscriber (sucess/failure).
        """

        for topic in self.subscriber_list:
            if not topic[1]:
                msg_class = rostopic.get_topic_class(topic[0])
                if msg_class[1] is None:
                    pass
                else:
                    rospy.Subscriber(topic[0], msg_class[0],
                                     lambda msg, _topic=topic[0]: self.bagger_callback(msg, _topic))

                    topic[1] = True  # successful subscription

    def get_oldest_topic_time(self, topic):

        """
        Return oldest timestamp for a given topic as a rospy.Time type
        """

        return self.topic_messages[topic][0][0]

    def get_newest_topic_time(self, topic):

        """
        Return newest timestamp for a given topic as a rospy.Time type
        """

        return self.topic_messages[topic][-1][0]

    def get_topic_duration(self, topic):

        """
        Return current time duration of topic
        """

        return self.get_newest_topic_time(topic) - self.get_oldest_topic_time(topic)

    def get_header_time(self, msg):

        """
        Retrieve header time if available
        """

        if hasattr(msg, 'header'):
            return msg.header.stamp
        else:
            return rospy.get_rostime()

    def get_ram_usage(self):

        """
        Return current ram usage in bytes
        """

        self.usage = resource.getrusage(resource.RUSAGE_SELF)

        # getattr returns ram usage in kilobytes
        self.ram_usage = getattr(self.usage, 'ru_maxrss')

        return self.ram_usage*1e3

    def set_ram_limit(self, topic):

        """
        Limit ram usage by removing oldest messages
        """

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

        if not self.streaming:
            return

        self.n = self.n + 1
        time = self.get_header_time(msg)

        self.topic_messages[topic].append((time, msg))

        self.set_ram_limit(topic)

        # verify streaming is popping off and recording topics
        if not self.n % 100:
        #     rospy.loginfo('time_diff: %s', time_diff.to_sec())
        #     rospy.loginfo('topic: %s', topic)
        #     rospy.loginfo('topic type: %s', type(msg))
            rospy.loginfo('ram size: %s', self.ram_usage/1e3)
        #     rospy.loginfo('number of messages: %s', self.get_topic_message_count(topic))


        # self.subscribe() # loop

    def get_subscribed_topics(self):

        """
        Return subscribed and failed topics
        """

        return self.subscriber_list

    def get_number_subscribed_topics(self):
        pass

    def get_topic_message_count(self, topic):

        """
        Return number of messages available in a topic
        """

        return len(self.topic_messages[topic])

    def get_all_message_count(self):

        """
        Returns total number of messages across all topics
        """

        message_count = 0
        for topic in self.topic_messages.keys():
            message_count = message_count + get_topic_message_count(topic)
            return message_count

    def set_bag_directory(self):

        """
        Create ros bag save directory
        """

        rospack = rospkg.RosPack()
        self.directory = os.path.join(rospack.get_path(self.dir), 'bags/')

        rospy.loginfo('bag directory: %s', self.directory)
        rospy.loginfo('bag name: %s', self.bag_name)

        if not os.path.exists(self.directory):
            os.makedirs(self.directory)

        self.bag = rosbag.Bag(os.path.join(self.directory, self.bag_name + '.bag'), 'w')

    def get_time_index(self, topic, requested_seconds):
        """
        Return the index for the time index for a topic at 'n' seconds from the end of the dequeue

        For example, to bag the last 10 seconds of data, the index for 10 seconds back from the most
        recent message can be obtained with this function.

        The number of requested seconds should be the number of seoncds desired from
        the end of deque. (ie. requested_seconds = 10 )

        If the desired time length of the bag is greater than the available messages it will output a
        message and return how ever many seconds of data are avaiable at the moment.

        Seconds is of a number type (not a rospy.Time type) (ie. int, float)
        """

        topic_duration = self.get_topic_duration(topic).to_sec()

        ratio = requested_seconds / topic_duration
        index = math.floor(self.get_topic_message_count(topic) * (1 - min(ratio, 1)))

        self.bag_report = 'The requested %s seconds were bagged' % requested_seconds

        if not index:
            self.bag_report = 'Only %s seconds of the request %s seconds were available, all \
            messages were bagged' %  (topic_duration, requested_seconds)
        return int(index)

    def start_bagging(self, req):
        """
        Dump all data in dictionary to bags, temporarily stops streaming
        during the bagging process, resumes streaming when over.
        """
        self.dumping = req.data
        self.streaming = False

        self.set_bag_directory()

        rospy.loginfo('dumping value: %s', self.dumping)
        rospy.loginfo('bagging commencing!')

        self.requested_seconds = int(10)

        for topic in self.topic_messages.keys():
            rospy.loginfo('topic: %s', topic)
            self.i = self.get_time_index(topic, self.requested_seconds)
            self.m = 0  # message number in a given topic
            for msgs in self.topic_messages[topic][self.i:]:
                self.m = self.m + 1
                self.bag.write(topic, msgs[1], t=msgs[0])
                if not self.m % 100:  # print every 100th topic, type and time
                    rospy.loginfo('topic: %s, message type: %s, time: %s',
                                  topic, type(msgs[1]), type(msgs[0]))

        rospy.loginfo('Bag Report: %s', self.bag_report)
        self.bag.close()
        rospy.loginfo('bagging finished!')
        self.streaming = True

        message = 'bagging finished'
        return (True, message)

if __name__ == "__main__":
    rospy.init_node('online_bagger')
    stream = OnlineBagger()
    rospy.spin()
