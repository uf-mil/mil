#!usr/bin/env python3
import asyncio

import txros
import uvloop
from ros_alarms import TxAlarmListener, TxHeartbeatMonitor
from std_msgs.msg import String

publish = True


async def do_publishing(nh):
    global publish
    pub = nh.advertise("/heartbeat", String)
    await pub.setup()
    while True:
        await nh.sleep(0.1)
        if publish:
            pub.publish(String("test"))


async def main():
    global publish
    nh = await txros.NodeHandle.from_argv("tx_hearbeat_test")

    alarm_name = "test_alarm123"
    hbm = await TxHeartbeatMonitor.init(
        nh, alarm_name, "/heartbeat", String, nowarn=True
    )
    monitor_df = asyncio.create_task(hbm.start_monitor())

    await do_publishing(nh)

    al = await TxAlarmListener.init(nh, alarm_name)

    print("Initial Clear test")
    assert await al.is_cleared()
    await nh.sleep(0.5)

    print("Heartbeat raise test")
    publish = False
    await nh.sleep(1)
    assert await al.is_raised()
    await nh.sleep(0.5)

    print("Hearbeat clear test")
    publish = True
    await nh.sleep(1)
    assert await al.is_cleared()

    print("Stop monitoring test")
    monitor_df.cancel()
    publish = False
    await nh.sleep(1)
    assert await al.is_cleared()
    await nh.sleep(0.5)

    print("Predicated test")
    publish = True

    async def cb(nh, alarm):
        await nh.sleep(1)
        return False  # Should never raise

    hbm.set_predicate(cb)
    monitor_df = hbm.start_monitor()

    await nh.sleep(0.5)
    publish = False
    await nh.sleep(0.5)
    assert await al.is_cleared()

    print("\nPassed!")


if __name__ == "__main__":
    uvloop.install()
    asyncio.run(main())
