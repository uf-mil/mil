#!/usr/bin/env python
import txros
from twisted.internet import defer
from std_msgs.msg import String
from ros_alarms import TxAlarmListener, TxHeartbeatMonitor

publish = True


@txros.util.cancellableInlineCallbacks
def do_publishing(nh):
    global publish
    pub = nh.advertise("/heartbeat", String)
    while True:
        yield nh.sleep(0.1)
        if publish:
            yield pub.publish(String("test"))


@txros.util.cancellableInlineCallbacks
def main():
    global publish
    nh = yield txros.NodeHandle.from_argv('tx_hearbeat_test')

    alarm_name = "test_alarm123"
    hbm = yield TxHeartbeatMonitor.init(nh, alarm_name, "/heartbeat", String, nowarn=True)
    monitor_df = hbm.start_monitor()

    do_publishing(nh)

    al = yield TxAlarmListener.init(nh, alarm_name)

    print "Inital Clear test"
    assert (yield al.is_cleared())
    yield nh.sleep(0.5)

    print "Heartbeat raise test"
    publish = False
    yield nh.sleep(1)
    assert (yield al.is_raised())
    yield nh.sleep(0.5)

    print "Hearbeat clear test"
    publish = True
    yield nh.sleep(1)
    assert (yield al.is_cleared())

    print "Stop monitoring test"
    monitor_df.addErrback(lambda e: e.trap(defer.CancelledError)).cancel()
    publish = False
    yield nh.sleep(1)
    assert (yield al.is_cleared())
    yield nh.sleep(0.5)

    print "Predicated test"
    publish = True

    @txros.util.cancellableInlineCallbacks
    def cb(nh, alarm):
        yield nh.sleep(1)
        defer.returnValue(False)  # Should never raise

    hbm.set_predicate(cb)
    monitor_df = hbm.start_monitor()

    yield nh.sleep(0.5)
    publish = False
    yield nh.sleep(0.5)
    assert (yield al.is_cleared())

    print "\nPassed!"

txros.util.launch_main(main)
