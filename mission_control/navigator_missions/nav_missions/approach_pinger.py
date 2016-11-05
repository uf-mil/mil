#!/usr/bin/env python
import txros
import numpy as np
from hydrophones.msg import ProcessedPing
from geometry_msgs.msg import Point, PoseStamped, Pose
from navigator_alarm import single_alarm, AlarmListener, AlarmBroadcaster
from twisted.internet import defer
import navigator_tools

@txros.util.cancellableInlineCallbacks
def head_for_pinger(navigator):
    ping_sub = yield navigator.nh.subscribe("/hydrophones/processed", ProcessedPing)
    yield ping_sub.get_next_message()
    while True:
        yield navigator.nh.sleep(1)
        processed_ping = yield ping_sub.get_last_message()
        print processed_ping
        if isinstance(processed_ping, ProcessedPing):
            print "Got processed ping message:\n{}".format(processed_ping)
            if processed_ping.freq > 35000 and processed_ping.freq < 36000:
                print "Trustworthy pinger heading"
                t = yield navigator.tf_listener.get_transform("/enu", "/hydrophones", processed_ping.header.stamp)
                base_vect = t.transform_vector(navigator_tools.point_to_numpy(processed_ping.position))
                base_vect = base_vect / np.linalg.norm(base_vect) * 10
                odom_enu = (yield navigator.tx_pose)[0]
                #pinger_move = navigator.move.set_position(navigator_tools.point_to_numpy(processed_ping.position)).go()
                pinger_move = yield navigator.move.set_position(odom_enu + base_vect).go()
                print "Heading towards pinger"
            else:
                print "Untrustworthy pinger heading. Freq = {} kHZ".format(processed_ping.freq)
        else:
            print "Expected ProcessedPing, got {}".format(type(processed_ping))

# Hydrophone locate mission
@txros.util.cancellableInlineCallbacks
def main(navigator):
    kill_alarm_broadcaster, kill_alarm = single_alarm('kill', action_required=True, problem_description="Killing wamv to listen to pinger")
    df = defer.Deferred().addCallback(head_for_pinger)
    df.callback(navigator)
    try:
        yield txros.util.wrap_timeout(df, 55, cancel_df_on_timeout=True)
    except txros.util.TimeoutError:
        print "Done heading towards pinger"
