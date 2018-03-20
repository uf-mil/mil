#!/usr/bin/env python

from __future__ import division
import rospy
import rosbag
import rostopic
import os
from collections import deque
import itertools
import datetime
from actionlib import SimpleActionServer, SimpleActionClient, TerminalState
from mil_msgs.msg import BagOnlineAction, BagOnlineResult, BagOnlineFeedback, BagOnlineGoal
import argparse
from tqdm import tqdm

"""
Online Bagger is a node which subscribes to a list of ros topics,
maintaining a buffer of the most recent n seconds. Parts or all of
these buffered topics can be written to a bag file by
sending a new goal to the /online_bagger/bag action server.

When run with the -c flag, instead runs an action client which connects
to online bagger, triggering a bag write and displaying a progress bar
as it writes.

"""


class OnlineBagger(object):
    BAG_TOPIC = '/online_bagger/bag'

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
        if len(self.subscriber_list) == 0:
            rospy.logwarn('No topics selected to subscribe to. Closing.')
            rospy.signal_shutdown('No topics to subscribe to')
            return
        self.make_dicts()

        self._action_server = SimpleActionServer(OnlineBagger.BAG_TOPIC, BagOnlineAction,
                                                 execute_cb=self.start_bagging, auto_start=False)
        self.subscribe_loop()
        rospy.loginfo('Remaining Failed Topics: {}\n'.format(
            self.get_subscriber_list(False)))
        self._action_server.start()

    def get_subscriber_list(self, status):
        """
        Get string of all topics, if their subscribe status matches the input (True / False)
        Outputs each topics: time_buffer(float in seconds), subscribe_statue(bool), topic(string)
        """
        sub_list = ''
        for topic in self.subscriber_list.keys():
            if self.subscriber_list[topic][1] == status:
                sub_list = sub_list + \
                    '\n{:13}, {}'.format(self.subscriber_list[topic], topic)
        return sub_list

    def get_params(self):
        """
        Retrieve parameters from param server.
        """
        self.dir = rospy.get_param('~bag_package_path', default=None)
        # Handle bag directory for MIL bag script
        if self.dir is None and 'BAG_DIR' in os.environ:
            self.dir = os.environ['BAG_DIR']

        self.stream_time = rospy.get_param(
            '~stream_time', default=30)  # seconds
        self.resubscribe_period = rospy.get_param(
            '~resubscribe_period', default=3.0)  # seconds
        self.dated_folder = rospy.get_param(
            '~dated_folder', default=True)  # bool

        self.subscriber_list = {}
        topics_param = rospy.get_param('~topics', default=[])

        # Add topics from rosparam to subscribe list
        for topic in topics_param:
            time = topic[1] if len(topic) == 2 else self.stream_time
            self.subscriber_list[topic[0]] = (time, False)

        def add_unique_topic(topic):
            if topic not in self.subscriber_list:
                self.subscriber_list[topic] = (self.stream_time, False)

        def add_env_var(var):
            for topic in var.split():
                add_unique_topic(topic)

        # Add topics from MIL bag script environment variables
        if 'BAG_ALWAYS' in os.environ:
            add_env_var(os.environ['BAG_ALWAYS'])
        for key in os.environ.keys():
            if key[0:4] == 'bag_':
                add_env_var(os.environ[key])

        rospy.loginfo(
            'Default stream_time: {} seconds'.format(self.stream_time))
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

        rospy.loginfo('Initial subscriber_list: {}'.format(
            self.get_subscriber_list(False)))

    def subscribe_loop(self):
        """
        Continue to subscribe until at least one topic is successful,
        then break out of loop and be called in the callback function to prevent the function
        from locking up.
        """
        self.resubscriber = None
        i = 0
        # if self.successful_subscription_count == 0 and not
        # rospy.is_shutdown():
        while self.successful_subscription_count == 0 and not rospy.is_shutdown():
            self.subscribe()
            rospy.sleep(0.1)
            i = i + 1
            if i % 1000 == 0:
                rospy.logdebug('still subscribing!')
        rospy.loginfo("Subscribed to {} of {} topics, will try again every {} seconds".format(
                      self.successful_subscription_count, len(self.subscriber_list), self.resubscribe_period))
        self.resubscriber = rospy.Timer(rospy.Duration(
            self.resubscribe_period), self.subscribe)

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
        if self.successful_subscription_count == len(self.subscriber_list):
            if self.resubscriber is not None:
                self.resubscriber.shutdown()
            rospy.loginfo(
                'All topics subscribed too! Shutting down resubscriber')

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
            rospy.logdebug("{} has {} messages spanning {} seconds".format(
                topic, self.get_topic_message_count(topic), round(time_diff.to_sec(), 2)))

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
            total_message_count = total_message_count + \
                self.get_topic_message_count(topic)

        return total_message_count

    def _get_default_filename(self):
        return str(datetime.date.today()) + '-' + str(datetime.datetime.now().time())[0:8]

    def get_bag_name(self, filename=''):
        """
        Create ros bag save directory
        If no bag name is provided, the current date/time is used as default.
        """
        # If directory param is not set, default to $HOME/bags/<date>
        default_dir = self.dir
        if default_dir is None:
            default_dir = os.path.join(os.environ['HOME'], 'bags')

        # if dated folder param is set to True, append current date to
        # directory
        if self.dated_folder is True:
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
            bag_name = bag_name + '.bag'
        return os.path.join(bag_dir, bag_name)

    def start_bagging(self, req):
        """
        Dump all data in dictionary to bags, temporarily stops streaming
        during the bagging process, resumes streaming when over.
        If bagging is already false because of an active call to this service
        """
        result = BagOnlineResult()
        if self.streaming is False:
            result.status = 'Bag Request came in while bagging, priority given to prior request'
            result.success = False
            self._action_server.set_aborted(result)
            return

        try:
            self.streaming = False
            result.filename = self.get_bag_name(req.bag_name)
            bag = rosbag.Bag(result.filename, 'w')

            requested_seconds = req.bag_time

            selected_topics = req.topics.split()

            feedback = BagOnlineFeedback()
            total_messages = 0
            bag_topics = {}
            for topic, (time, subscribed) in self.subscriber_list.iteritems():
                if not subscribed:
                    continue
                # Exclude topics that aren't in topics service argument
                # If topics argument is empty string, include all topics
                if len(selected_topics) > 0 and topic not in selected_topics:
                    continue
                if len(self.topic_messages[topic]) == 0:
                    continue

                if req.bag_time == 0:
                    index = 0
                else:
                    index = self.get_time_index(topic, requested_seconds)
                total_messages += len(self.topic_messages[topic][index:])
                bag_topics[topic] = index
            if total_messages == 0:
                result.success = False
                result.status = 'no messages'
                self._action_server.set_aborted(result)
                self.streaming = True
                bag.close()
                return
            self._action_server.publish_feedback(feedback)
            msg_inc = 0
            for topic, index in bag_topics.iteritems():
                for msgs in self.topic_messages[topic][index:]:
                    bag.write(topic, msgs[1], t=msgs[0])
                    if msg_inc % 50 == 0:  # send feedback every 50 messages
                        feedback.progress = float(msg_inc) / total_messages
                        self._action_server.publish_feedback(feedback)
                    msg_inc += 1
                    # empty deque when done writing to bag
                self.topic_messages[topic].clear()
            feedback.progress = 1.0
            self._action_server.publish_feedback(feedback)
            bag.close()
        except Exception as e:
            result.success = False
            result.status = 'Exception while writing bag: ' + str(e)
            self._action_server.set_aborted(result)
            self.streaming = True
            bag.close()
            return
        rospy.loginfo('Bag written to {}'.format(result.filename))
        result.success = True
        self._action_server.set_succeeded(result)
        self.streaming = True


class OnlineBaggerClient(object):
    '''
    Wrapper to run an action client connecting to online bagger
    and triggering a write with the specified topics, time, and
    filename. Uses tqdm to display a progress bar as the write
    occurs.
    '''
    def __init__(self, name='', topics='', time=0.0):
        self.client = SimpleActionClient(OnlineBagger.BAG_TOPIC, BagOnlineAction)
        self.goal = BagOnlineGoal(bag_name=name, topics=topics, bag_time=time)
        self.result = None
        self.total_it = 0

    def _done_cb(self, status, result):
        self.result = (status, result)

    def _feedback_cb(self, feedback):
        percentage = int(100.0 * feedback.progress)
        if percentage - self.total_it >= 1:
            self.bar.update(percentage - self.total_it)
            self.bar.refresh()
            self.total_it = percentage

    def bag(self, timeout=rospy.Duration(0)):
        self.client.wait_for_server()
        self.result = None
        self.total_it = 0
        self.bar = tqdm(desc='Writing bag', unit=' percent', total=100,
                        bar_format='{desc} {bar} {percentage:3.0f}%')
        self.client.send_goal(self.goal, done_cb=self._done_cb,
                              feedback_cb=self._feedback_cb)
        while self.result is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        self.bar.refresh()
        self.bar.close()
        (status, result) = self.result
        if status == 3:
            print 'Bag successful'
        else:
            print 'Bag {}: {}'.format(TerminalState.to_string(status), result.status)

if __name__ == "__main__":
    argv = rospy.myargv()
    parser = argparse.ArgumentParser(description='ROS node to maintain buffers to create bags of the past\
                                                  and client to call this node.')
    parser.add_argument('-c', '--client', dest='client', action='store_true',
                        help='Run as an online bagger client instead of server')
    parser.add_argument('-d', '--duration', dest='time', type=float, default=0.0,
                        help='Time in seconds to bag when running as client')
    parser.add_argument('-t', '--topics', dest='topics', type=str, nargs='+', metavar='topic',
                        help='List of topics to include in bag when running in client, bag all topics if not set')
    parser.add_argument('-n', '--name', dest='name', type=str, default='',
                        help='name of bag to create when running as client')
    args = parser.parse_args(argv[1:])
    if args.client:  # Run as actionclient
        rospy.init_node('online_bagger_client', anonymous=True)
        if args.topics is None:
            topics = ''
        else:
            topics = ''.join(args.topics)
        client = OnlineBaggerClient(name=args.name, topics=topics, time=args.time)
        client.bag()
    else:  # Run as OnlineBagger server
        rospy.init_node('online_bagger')
        bagger = OnlineBagger()
        rospy.spin()
