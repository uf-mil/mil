#!/usr/bin/env python

from __future__ import division
import rospy
import rosbag
import rostopic
import os
import resource
from collections import deque
import itertools
import datetime
from mil_msgs.srv import BaggerCommands

"""
To call call service call the '/online_bagger/dump' service with the BaggerCommands.srv
Amount and Unit. Doing this will generate a rosbag with the
last x number of seconds saved

For example:

        bagging_server = rospy.ServiceProxy('/online_bagger/dump', BaggerCommands)
        bag_status = bagging_server(bag_name, bag_time)
        bag_name = name of bag (leave blank to use default name: current date and time)
            Provide an empty string: '' to bag everything
        bag_time = float32 (leave blank to dump entire bag): 
            Provide 0.0, or 0 to bag everything
"""

class OnlineBagger(object):

    def __init__(self):

        """
        Make dictionary of dequeues.
        Subscribe to set of topics defined by the yaml file in directory
        Stream topics up to a given stream time, dump oldest messages when limit is reached
        Set up service to bag n seconds of data default to all of available data
        """

        self.successful_subscription_count = 0  # successful subscriptions
        self.iteration_count = 0  # number of iterations
        self.streaming = True
        self.get_params()
        self.make_dicts()

        self.bagging_service = rospy.Service('~dump', BaggerCommands,
                                             self.start_bagging)

        self.subscribe_loop()

        rospy.loginfo('')
        rospy.loginfo('Remaining Failed Topics:')
        self.print_subscriber_list(False)
        rospy.loginfo('')

    def print_subscriber_list(self, status):
        for topic in self.subscriber_list.keys():
            if self.subscriber_list[topic][1] == status:
                rospy.loginfo('{}, {}'.format(self.subscriber_list[topic], topic))

    def get_params(self):

        """
        Retrieve parameters from param server.
        """
        self.dir = rospy.get_param('~bag_package_path', default=None)
        # Handle bag directory for MIL bag script
        if self.dir is None and os.environ.has_key('BAG_DIR'):
            self.dir = os.environ['BAG_DIR']

        self.stream_time = rospy.get_param('~stream_time', default=30)  # seconds

        self.subscriber_list = {}
        topics_param = rospy.get_param('~topics', default=[])

        # Add topics from rosparam to subscribe list
        for topic in topics_param:
            time = topic[1] if len(topic) == 2 else self.stream_time
            self.subscriber_list[topic[0]] = (time, False)

        def add_unique_topic(topic):
            if not self.subscriber_list.has_key(topic):
                self.subscriber_list[topic] = (self.stream_time, False)

        def add_env_var(var):
            for topic in var.split():
                add_unique_topic(topic)

        # Add topics from MIL bag script environment variables
        if os.environ.has_key('BAG_ALWAYS'):
            add_env_var(os.environ['BAG_ALWAYS'])
        for key in os.environ.keys():
            if key[0:4] == 'bag_':
                add_env_var(os.environ[key])

        self.resubscribe_period = rospy.get_param('~resubscribe_period', default=3.0)

        rospy.loginfo('Default stream_time: {} seconds'.format(self.stream_time))
        rospy.loginfo('Bag Directory: {}'.format(self.dir))
    def make_dicts(self):

        """
        Make dictionary with sliceable deques() that will be filled with messages and time stamps.

        Subscriber list contains all of the topics, their stream time and their subscription status:
        A True status for a given topic corresponds to a successful subscription
        A False status indicates a failed subscription.

        Stream time for an individual topic is specified in seconds.

        For Example:
        self.subscriber_list[0:1] = [['/odom', 300 ,False], ['/absodom', 300, True]]

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
        and its time stamp if available. It is a tuple with each element: (time_stamp, msg)
        self.topic_message['/odom'][0][0] is the time stamp for the oldest message
        self.topic_message['/odom'][0][1] is the message associated with the oldest topic
        """

        self.topic_messages = {}

        class SliceableDeque(deque):
            def __getitem__(self, index):
                if isinstance(index, slice):
                    return type(self)(itertools.islice(self, index.start,
                                                       index.stop, index.step))
                return deque.__getitem__(self, index)

        for topic in self.subscriber_list:
            self.topic_messages[topic] = SliceableDeque(deque())

        rospy.loginfo('')
        rospy.loginfo('Initial subscriber_list:')
        self.print_subscriber_list(False)

    def subscribe_loop(self):

        """
        Continue to subscribe until at least one topic is successful,
        then break out of loop and be called in the callback function to prevent the function
        from locking up.
        """

        i = 0
        # if self.successful_subscription_count == 0 and not rospy.is_shutdown():
        while self.successful_subscription_count == 0 and not rospy.is_shutdown():
            self.subscribe()
            rospy.sleep(0.1)
            i = i + 1
            if i % 1000 == 0:
                rospy.logdebug('still subscribing!')
        rospy.loginfo("Subscribed to {} of {} topics, will try again every {} seconds".format(self.successful_subscription_count, 
            len(self.subscriber_list), self.resubscribe_period))
        rospy.Timer(rospy.Duration(self.resubscribe_period), self.subscribe)

    def subscribe(self, time_info=None):

        """
        Subscribe to the topics defined in the yaml configuration file

        Function checks subscription status True/False of each topic
        if True: topic has already been sucessfully subscribed to
        if False: topic still needs to be subscribed to and
        subscriber will be run.

        Each element in self.subscriber list is a list [topic, Bool]
        where the Bool tells the current status of the subscriber (sucess/failure).

        Return number of topics that failed subscription
        """

        for topic, (time, subscribed) in self.subscriber_list.items():
            if not subscribed:
                msg_class = rostopic.get_topic_class(topic)
                if msg_class[1] is not None:
                    self.successful_subscription_count += 1
                    rospy.Subscriber(topic, msg_class[0],
                                     lambda msg, _topic=topic: self.bagger_callback(msg, _topic))

                    self.subscriber_list[topic] = (time, True)

    def get_topic_duration(self, topic):

        """
        Return current time duration of topic
        """

        return self.topic_messages[topic][-1][0] - self.topic_messages[topic][0][0]

    def get_header_time(self, msg):

        """
        Retrieve header time if available
        """

        if hasattr(msg, 'header'):
            return msg.header.stamp
        else:
            return rospy.get_rostime()

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

        if topic_duration == 0:
            return 0

        ratio = requested_seconds / topic_duration
        index = int(self.get_topic_message_count(topic) * (1 - min(ratio, 1)))

        self.bag_report = 'The requested %s seconds were bagged' % requested_seconds

        if index == 0:
            self.bag_report = 'Only %s seconds of the request %s seconds were available, all \
            messages were bagged' %  (topic_duration, requested_seconds)
        return index

    def bagger_callback(self, msg, topic):

        """
        Streaming callback function, stops streaming during bagging process
        also pops off msgs from dequeue if stream size is greater than specified stream_time

        Stream, callback function does nothing if streaming is not active
        """

        if not self.streaming:
            return

        self.iteration_count = self.iteration_count + 1
        time = self.get_header_time(msg)

        self.topic_messages[topic].append((time, msg))

        time_diff = self.get_topic_duration(topic)

        # verify streaming is popping off and recording topics
        if self.iteration_count % 100 == 0:
            rospy.logdebug("{} has {} messages spanning {} seconds".format(topic, self.get_topic_message_count(topic), round(time_diff.to_sec(),2)))

        while time_diff > rospy.Duration(self.subscriber_list[topic][0]) and not rospy.is_shutdown():
            self.topic_messages[topic].popleft()
            time_diff = self.get_topic_duration(topic)

    def get_topic_message_count(self, topic):

        """
        Return number of messages available in a topic
        """

        return len(self.topic_messages[topic])

    def get_total_message_count(self):

        """
        Returns total number of messages across all topics
        """

        total_message_count = 0
        for topic in self.topic_messages.keys():
            total_message_count = total_message_count + self.get_topic_message_count(topic)

        return total_message_count

    def _get_default_filename(self):
        return str(datetime.date.today()) + '-' + str(datetime.datetime.now().time())[0:8]

    def set_bag_directory(self, filename=''):

        """
        Create ros bag save directory
        If no bag name is provided, the current date/time is used as default.
        """

        # If directory param is not set, default to $HOME/bags/<date>
        default_dir = self.dir
        if default_dir == None:
            default_dir = os.path.join(os.environ['HOME'], 'bags')
        default_dir = os.path.join(default_dir, str(datetime.date.today()))
        # Split filename from directory
        bag_dir, bag_name = os.path.split(filename)
        bag_dir = os.path.join(default_dir, bag_dir)
        if not os.path.exists(bag_dir):
            os.makedirs(bag_dir)

        # Create default filename if only directory specified
        if bag_name == '':
            bag_name = self._get_default_filename()
        # Make sure filename ends in .bag, add otherwise
        if bag_name[-4:] != '.bag':
            bag_name = bag_name+'.bag'
        rospy.loginfo('bag directory: %s', bag_dir)
        rospy.loginfo('bag name: %s', bag_name)

        self.bag = rosbag.Bag(os.path.join(bag_dir, bag_name), 'w')

    def set_bagger_status(self):

        """
        Set status of online bagger
        """

        self.bagger_status = 'Message Count: {}'.format(str(self.get_total_message_count())) \

    def start_bagging(self, req):

        """
        Dump all data in dictionary to bags, temporarily stops streaming
        during the bagging process, resumes streaming when over.
        """

        self.streaming = False
        self.set_bag_directory(req.bag_name)
        self.set_bagger_status()

        requested_seconds = req.bag_time

        selected_topics = req.topics.split()
        for topic, (time, subscribed) in self.subscriber_list.items():
            if not subscribed:
                continue
            # Exclude topics that aren't in topics service argument
            # If topics argument is empty string, include all topics
            if len(selected_topics) > 0 and not topic in selected_topics:
                continue
            if len(self.topic_messages[topic]) == 0:
                continue

            rospy.loginfo('topic: %s', topic)

            # if no number is provided or a zero is given, bag all messages
            types = (0, 0.0, None)
            if req.bag_time in types:
                bag_index = 0
                self.bag_report = 'All messages were bagged'

            # get time index the normal way
            else:
                bag_index = self.get_time_index(topic, requested_seconds)
            rospy.loginfo('topic: {}, bag_index: {}'.format(topic, bag_index))
            messages = 0  # message number in a given topic
            for msgs in self.topic_messages[topic][bag_index:]:
                messages = messages + 1
                self.bag.write(topic, msgs[1], t=msgs[0])
                if messages % 100 == 0:  # print every 100th topic, type and time
                    rospy.loginfo('topic: %s, message type: %s, time: %s',
                    topic, type(msgs[1]), type(msgs[0]))

            # empty deque when done writing to bag
            self.topic_messages[topic].clear()

        rospy.loginfo('Bag Report: {}'.format(self.bagger_status))
        rospy.loginfo('Subscriber List:')
        self.print_subscriber_list(True)
        self.bag.close()
        rospy.loginfo('bagging finished!')

        self.streaming = True
        return self.bagger_status

if __name__ == "__main__":
    rospy.init_node('online_bagger')
    stream = OnlineBagger()
    rospy.spin()
