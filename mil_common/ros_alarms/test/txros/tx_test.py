#!/usr/bin/env python3
import asyncio

import txros
import uvloop
from ros_alarms import TxAlarmBroadcaster, TxAlarmListener


async def main():
    nh = await txros.NodeHandle.from_argv("tx_alarm_test")

    alarm_name = "tx"
    ab = await TxAlarmBroadcaster.init(nh, alarm_name)
    al = await TxAlarmListener.init(nh, alarm_name, nowarn=True)

    assert await al.is_cleared()

    await ab.raise_alarm()
    assert await al.is_raised()

    await ab.raise_alarm()
    assert await al.is_raised()

    await ab.clear_alarm()
    assert await al.is_cleared()

    var = False

    async def cb(nh, alarm):
        global var
        var = True
        await nh.sleep(1)
        print("DONE SLEEPING")

    await al.add_callback(cb, call_when_raised=False)
    await nh.sleep(0.1)
    assert not var

    await ab.raise_alarm()
    assert not var

    await ab.clear_alarm()
    assert not var

    print("\nPassed")
    await nh.sleep(2)


if __name__ == "__main__":
    uvloop.install()
    asyncio.run(main())
