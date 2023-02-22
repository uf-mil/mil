#!/usr/bin/env python
import navigator_tools
import numpy as np
import tf
import axros
from geometry_msgs.msg import Point, Pose, PoseStamped
from hydrophones.msg import ProcessedPing
from navigator_alarm import AlarmBroadcaster, AlarmListener, single_alarm
from tf import transformations as trans
from twisted.internet import defer

ping_sub = yield navigator.nh.subscribe("/hydrophones/processed", ProcessedPing)
yield ping_sub.get_next_message()
target_freq = 35000
while True:
    processed_ping = yield ping_sub.get_next_message()
    print processed_ping
    if isinstance(processed_ping, ProcessedPing):
        print "Got processed ping message:\n{}".format(processed_ping)
        if processed_ping.freq > 35000 and processed_ping.freq < 36000:
            freq_dev = abs(target_freq - processed_ping.freq)
            print "Trustworthy pinger heading"
            hydrophones_enu_p, hydrophones_enu_q = tf.lookupTransform("/hydrophones", "/enu", processed_ping.header.stamp)
            pinger_enu_p = navigator_tools.rosmsg_to_numpy(tf.transformPoint())
            dir_ = navigator_tools.rosmsg_to_numpy(processed_ping.position)
        	mv_mag = 2
            mv_hyd_frame = dir_ / np.linalg.norm(dir_)
            pinger_move = navigator.move.set_position(navigator_tools.rosmsg_to_numpy(processed_ping.position)).go()

            print "Heading towards pinger"
        else:
            print "Untrustworthy pinger heading. Freq = {} kHZ".format(processed_ping.freq)
    else:
        print "Expected ProcessedPing, got {}".format(type(processed_ping))

# Hydrophone locate mission
@axros.util.cancellableInlineCallbacks
def main(navigator):
    kill_alarm_broadcaster, kill_alarm = single_alarm('kill', action_required=True, problem_description="Killing wamv to listen to pinger")
    df = defer.Deferred().addCallback(head_for_pinger)
    df.callback(navigator)
    try:
        yield axros.util.wrap_timeout(df, 15, cancel_df_on_timeout=True)
    except axros.util.TimeoutError:
        print "Done heading towards pinger"