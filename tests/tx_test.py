#!/usr/bin/env python
import txros
from ros_alarms import TxAlarmBroadcaster, TxAlarmListener

@txros.util.cancellableInlineCallbacks
def main():
    nh = yield txros.NodeHandle.from_argv('tx_alarm_test')

    alarm_name = "test_alarm"
    ab = yield TxAlarmBroadcaster.init(nh, alarm_name)
    al = yield TxAlarmListener.init(nh, alarm_name, nowarn=True)

    assert (yield al.is_cleared())

    yield ab.raise_alarm()
    assert (yield al.is_raised())

    yield ab.raise_alarm()
    assert (yield al.is_raised())

    yield ab.clear_alarm()
    assert (yield al.is_cleared())

    var = False
    @txros.util.cancellableInlineCallbacks
    def cb(nh, alarm):
        var = True
        yield nh.sleep(1)
        print "DONE SLEEPING"

    al.add_callback(cb, call_when_raised=False)
    assert not var

    yield ab.raise_alarm()
    assert not var

    yield ab.clear_alarm()
    assert not var

    print "\nPassed"
    yield nh.sleep(2)

txros.util.launch_main(main)

