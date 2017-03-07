#!/usr/bin/env python

import rospy
import os
import rosbag
import rostopic
from collections import deque
import yaml
import rospkg
from std_srvs.srv import SetBool

'''
To call this service call the '/online_bagger' service with the SetBool Parameter set to True.
Doing this will generate a rosbag with the last x number of seconds saved

for example:

    bagging_server = rospy.ServiceProxy('/online_bagger', SetBool)
    bag_status = bagging_server(True)
'''

class OnlineBagger(object):
    def __init__(self):

        '''
        - makes dictionary of dequeues
        - subscribes to set of topics defined by the yaml file in directory
        - have a streaming option that can be turned on and off in the callback
        - option to bag data
        - add a re subscribing option for anything that failed? maybe...?
        '''
 
        self.failed_topics = []
        self.n = 0  # number of iterations
        self.streaming = True
        self.dumping = False
        # self.resubscribe

        self.get_params()
        self.make_dicts()
        self.subscribe()


        self.bagging_service = rospy.Service('/online_bagger', SetBool, self.start_bagging)

    def get_params(self):

        self.topics = rospy.get_param('/online_bagger/message_topics')
        self.ram_limit = rospy.get_param('/online_bagger/ram_limit')
        self.stream_time = rospy.get_param('/online_bagger/stream_time')

        rospy.loginfo('stream_time: %s', self.stream_time)
        # rospy.loginfo('ram_limit: %s', self.ram_limit)

    def make_dicts(self):
        '''
        make dictionaries with deques() that will be filled with topics
        with all of the 5topics defined by the yaml
        '''

        self.topic_list = {}
 
        for topic in self.topics:
            self.topic_list[topic] = deque()

        rospy.loginfo('topics: %s', self.topic_list.keys())
    def subscribe(self):
        '''
        Immediately initiated with instance of the BagStream class:
        subscribes to the set of topics defined in the yaml configuration file
        '''
        # use failed topics list that can be resubscribed to at a later time?

        for topic in self.topic_list.keys():
            msg_class = rostopic.get_topic_class(topic)
            if msg_class[1] is None:
                self.failed_topics.append(topic)
            else:
                rospy.Subscriber(topic, msg_class[0], lambda msg, _topic=topic: self.callback(msg, _topic))

        rospy.loginfo('failed topics: %s', self.failed_topics)

    def get_oldest_topic_time(self, topic):
        '''
        returns oldest timestamp for a given topic
        '''
        return self.topic_list[topic][0][0]

    def get_newest_topic_time(self, topic):
        '''
        returns newest timestamp for a given topic
        '''
        return self.topic_list[topic][-1][0]


    def callback(self, msg, topic):
        '''
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
        '''

        if not(self.streaming):
            return

        self.n = self.n + 1
        self.topic_list[topic].append((rospy.get_rostime(), msg))

        time_diff = self.get_newest_topic_time(topic) - self.get_oldest_topic_time(topic)

        # verify streaming is popping off and recording topics
        # if self.n % 50 == 0:
        #     rospy.loginfo('time_diff: %s', time_diff.to_sec())
        #     rospy.loginfo('topic: %s', topic)

        if time_diff > rospy.Duration(self.stream_time):
            self.topic_list[topic].popleft()
        else:
            pass

    def start_bagging(self, req):
        '''
        dumps all data in dictionary to bags, temporarily stops streaming
        during the bagging process, resumes streaming when over
        '''
        self.dumping = req.data
        self.streaming = False
        bag = rosbag.Bag('ooka.bag', 'w')

        rospy.loginfo('dumping value: %s', self.dumping)
        rospy.loginfo('bagging commencing!')

        for topic in self.topic_list.keys():
            rospy.loginfo('topic: %s', topic)
            for msgs in self.topic_list[topic]:

                bag.write(topic, msgs[1], t=msgs[0])
                # rospy.loginfo('topic: %s, message type: %s, time: %s', topic, type(msgs[1]), type(msgs[0]))

        bag.close()
        rospy.loginfo('bagging finished!')
        self.streaming = True

        message = 'bagging finished'
        return (True, message)

if __name__ == "__main__":
    rospy.init_node('online_bagger')
    stream = OnlineBagger()
    rospy.spin()
