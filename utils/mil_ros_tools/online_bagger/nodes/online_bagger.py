#!/usr/bin/env python

import resource
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

        self.n = 0  # number of iterations
        self.streaming = True
        self.dumping = False
        # self.resubscribe

        self.get_params()
        self.make_dicts()
        self.subscribe()
        rospy.loginfo('subscriber success: %s', self.subscriber_list)


        self.bagging_service = rospy.Service('/online_bagger/dump', SetBool, self.start_bagging)

    def get_params(self):

        self.topics = rospy.get_param('/online_bagger/message_topics')
        self.ram_limit = rospy.get_param('/online_bagger/ram_limit')*1.073e9 
        self.stream_time = rospy.get_param('/online_bagger/stream_time')

        rospy.loginfo('stream_time: %s seconds', self.stream_time)
        rospy.loginfo('ram_limit: %s gb', self.ram_limit)

    # def set_ram_limit(self)

    #     # http://stackoverflow.com/questions/30269238/limit-memory-usage
    #     # _, hard = resource.getrlimit(resource.RLIMIT_DATA)
    #     resource.setrlimit(resource.RLIMIT_DATA, (self.ram_limit, hard))


    def make_dicts(self):
        '''
        make dictionaries with deques() that will be filled with topics
        with all of the 5topics defined by the yaml
        '''

        self.topic_list = {}
        self.subscriber_list = []
        for topic in self.topics:
            self.topic_list[topic] = deque()
            self.subscriber_list.append([topic, False])

        rospy.loginfo('failed topics: %s', self.subscriber_list)

    def subscribe(self):
        '''
        Immediately initiated with instance of the BagStream class:
        subscribes to the set of topics defined in the yaml configuration file

        Function checks subscription status True/False of each topic
        if True: topic has already been sucessfully subscribed to
        if False: topic still needs to be subscribed to and 
        subscriber will be run

        Each element in self.subscriber list is a list [topic, Bool]
        where the Bool tells the current status of the subscriber (sucess/failure)
        '''

        for topic in self.subscriber_list:
            if topic[1] is True:
                pass
            else:
                msg_class = rostopic.get_topic_class(topic[0])
                if msg_class[1] is None:
                    pass
                else:
                    rospy.Subscriber(topic[0], msg_class[0], lambda msg, _topic=topic[0]: self.callback(msg, _topic))
                    topic[1] = True # successful subscription

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

        Continually resubscribes to any failed topics.
        '''

        if not(self.streaming):
            return

        self.n = self.n + 1
        self.topic_list[topic].append((rospy.get_rostime(), msg))

        time_diff = self.get_newest_topic_time(topic) - self.get_oldest_topic_time(topic)

        # verify streaming is popping off and recording topics
        if self.n % 50 == 0:
            rospy.loginfo('time_diff: %s', time_diff.to_sec())
            rospy.loginfo('topic: %s', topic)

        if time_diff > rospy.Duration(self.stream_time):
            self.topic_list[topic].popleft()
        else:
            pass

        self.subscribe()



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
            self.m = 0 # message number in a given topic
            for msgs in self.topic_list[topic]:
                self.m = self.m + 1
                bag.write(topic, msgs[1], t=msgs[0])
                if self.m % 50 == 0: # print every 50th topic, type and time
                    rospy.loginfo('topic: %s, message type: %s, time: %s', topic, type(msgs[1]), type(msgs[0]))

        bag.close()
        rospy.loginfo('bagging finished!')
        self.streaming = True

        message = 'bagging finished'
        return (True, message)

if __name__ == "__main__":
    rospy.init_node('online_bagger')
    stream = OnlineBagger()
    rospy.spin()
