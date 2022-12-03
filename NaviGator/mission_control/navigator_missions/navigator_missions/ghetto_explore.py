#!/usr/bin/env python3
import mil_tools as nt

from .navigator import NaviGatorMission


async def myfunc(navigator: NaviGatorMission, **kwargs):
    pos = navigator.tx_pose
    pos = pos[0]

    exploring = ["Exploring1", "Exploring2", "Exploring3", "Exploring4"]

    for e in exploring:
        try:
            objects = await navigator.database_query(e)
            o = objects.objects[0]
            print(o.name)
            pos = nt.rosmsg_to_numpy(o.position)
            await navigator.move.set_position(pos).go()
            nt.fprint(o.name, msg_color="green")

        except:
            nt.fprint("Missing Marker", msg_color="red")


async def main(navigator: NaviGatorMission, **kwargs):
    await navigator.change_wrench("autonomous")
    await navigator.nh.sleep(0.1)
    good = await myfunc(navigator)
    return good
