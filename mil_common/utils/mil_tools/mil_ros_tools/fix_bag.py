#!/usr/bin/env python
import argparse
import rosbag
import rospy
from tqdm import tqdm


class BagFixer():
    '''
    Dictionary of topics to remap. If ends in /, remaps everything after
    Otherwise, topic must match exactly
    '''

    def fix_topic(self, topic):
        for k, t in self.topic_map.iteritems():
            if topic.find(k) == 0:
                return t + topic[len(k):]
        return topic

    def fix_tf(self, tf):
        for i, t in enumerate(tf.transforms):
            tf.transforms[i].header.frame_id = self.fix_frame(
                tf.transforms[i].header.frame_id)
            tf.transforms[i].child_frame_id = self.fix_frame(
                tf.transforms[i].child_frame_id)
        return tf

    def fix_frame(self, frame):
        fixed_frame = self.frame_map.get(frame)
        return fixed_frame if fixed_frame is not None else frame

    def fix_bag(self, infile, outfile=None):
        if outfile is None:
            split = infile.rsplit('.bag', 1)
            outfile = split[0] + '_fixed.bag'
        out = rosbag.Bag(outfile, 'w')
        try:
            bag = rosbag.Bag(infile)
        except rosbag.bag.ROSBagException as e:
            print "Error opening bag {}: {}".format(infile, e)
            return
        _, _, first_time = bag.read_messages().next()
        assert(first_time is not None)
        start, stop = None, None
        if self.start is not None:
            start = first_time + rospy.Duration(self.start)
        if self.stop is not None:
            stop = first_time + rospy.Duration(self.stop)
        total_messages = bag.get_message_count()
        # This could be made signifigantly faster by using ag.get_type_and_topic_info
        # to do some preprocessing on what topics will be used / remaped /
        # processed
        for topic, msg, time in tqdm(bag.read_messages(start_time=start, end_time=stop),
                                     total=total_messages, unit=' messages', desc='Fixing bag'):
            if not self.valid_topic(topic):
                continue
            topic = self.fix_topic(topic)
            if hasattr(msg, 'header') and msg.header._type == 'std_msgs/Header':
                msg.header.frame_id = self.fix_frame(msg.header.frame_id)
            if msg._type == 'tf2_msgs/TFMessage' or msg._type == 'tf/tfMessage':
                msg = self.fix_tf(msg)
            out.write(topic, msg, time)
        out.flush()
        out.close()

    def _fix_strings(self, strings):
        if type(strings) == dict:
            return strings
        d = {}
        if strings is None:
            return d
        assert type(strings) == list
        for s in strings:
            split = s.split(':')
            assert len(split) == 2
            d[split[0]] = split[1]
        return d

    def valid_topic(self, topic):
        if topic in self.ignore_topics:
            return False
        if self.keep_topics is not None and len(self.keep_topics):
            for t in self.keep_topics:
                if topic.find(t) == 0:
                    return True
            return False
        return True

    def __init__(self, topic_map={}, frame_map={}, start=None, stop=None, keep=[], ignore=[]):
        if ignore is None:
            ignore = []
        self.topic_map = self._fix_strings(topic_map)
        self.frame_map = self._fix_strings(frame_map)
        self.start = start
        self.stop = stop
        self.keep_topics = keep if keep is not None else []
        self.ignore_topics = ignore if keep is not None else []


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Fix bag topics/frame_ids')
    parser.add_argument('--in', '-i', dest='infile', type=str, required=True,
                        help='Bag to read and fix')
    parser.add_argument('--out', '-o', type=str, dest='outfile',
                        help='Bag to output with fixed content, defaults to INPUTFILENAME_fixed.bag')
    parser.add_argument('--remap-topics', dest='topics', required=False, nargs='+', type=str,
                        metavar='oldtopic:newtopic',
                        help="list of topics to remap, seperated by a colon, ex /down_cam/:/camera/down/\
                              /my_odom:/odom\nIf ends in a slash (ex: /cameras/:/cams/,\
                              all topics after slash will be remaped")
    parser.add_argument('--remap-frames', dest='frames', required=False, nargs='+', type=str,
                        metavar='oldframe:newframe',
                        help="list of frame_ids in headers to be maped, ex: my_camera:cam_front")
    parser.add_argument('--start', required=False, type=float, metavar='TIME',
                        help="duration in bag, in seconds, to start output bag")
    parser.add_argument('--stop', required=False, type=float, metavar='TIME',
                        help="duration in bag, in seconds, to stop output bag")
    parser.add_argument('--keep-topics', dest='keep_topics', required=False, nargs='+', type=str, metavar='topic',
                        help="if used, output bag will contain only this list of topics")
    parser.add_argument('--ignore-topics', dest='ignore_topics', required=False, nargs='+', type=str, metavar='topic',
                        help="if used, output bag will not contain these topics")

    args = parser.parse_args()
    if args.keep_topics and args.ignore_topics:
        parser.error(
            'Only --keep-topics or --ignore-topics can be used at once')

    fixer = BagFixer(topic_map=args.topics, frame_map=args.frames,
                     start=args.start, stop=args.stop,
                     keep=args.keep_topics, ignore=args.ignore_topics)
    fixer.fix_bag(args.infile, args.outfile)
