#!/usr/bin/env python3
import mil_tools as nt
import numpy as np
from mil_misc_tools.text_effects import fprint
from mil_tools import rosmsg_to_numpy
from navigator_tools import MissingPerceptionObject


def get_closest_objects(position, objects, max_len: int = 3, max_dist: int = 30):
    num = len(objects)
    idx = max_len
    if num < max_len:
        idx = num
    objects = sorted(
        objects,
        key=lambda x: np.linalg.norm(position - rosmsg_to_numpy(x.position)),
    )
    objects = objects[:idx]
    dists = (np.linalg.norm(position - rosmsg_to_numpy(x.position)) for x in objects)
    final_objs = []
    for i, d in enumerate(dists):
        if d < max_dist:
            final_objs.append(objects[i])
        else:
            break
    return final_objs


async def wait_for_object(object, helper, nh):
    helper.set_looking_for(object)
    while not helper.is_found_func():
        await nh.sleep(1)


async def go_to_objects(navigator, position, objs):
    objects = get_closest_objects(position, objs)
    for o in objects:
        fprint(f"MOVING TO OBJECT WITH ID {o.id}", msg_color="green")
        await navigator.nh.sleep(5)
        pos = nt.rosmsg_to_numpy(o.position)
        await navigator.move.look_at(pos).set_position(pos).backward(7).go()


async def myfunc(navigator, looking_for, center_marker):
    # Updated
    high_prob_objs = ["shooter", "dock"]
    pos = await navigator.tx_pose
    pos = pos[0]

    if center_marker is None or center_marker == "None":
        return True

    try:
        center_marker = await navigator.database_query(object_name=center_marker.name)
        center_marker = center_marker.objects[0]
        print(center_marker.name)
    except Exception:
        fprint("A marker has not been set", msg_color="red")
        return False

    mark_pos = nt.rosmsg_to_numpy(center_marker.position)
    dist = np.linalg.norm(pos - mark_pos)
    if dist > 10:
        await navigator.move.look_at(mark_pos).set_position(mark_pos).go()
    return True

    if looking_for is None or looking_for == "None":
        return True

    pos = await navigator.tx_pose()
    pos = pos[0]

    if looking_for in high_prob_objs:
        try:
            obj = await navigator.database_query(object_name=looking_for)
            fprint("EXPLORER FOUND OBJECT", msg_color="blue")
            await navigator.database_query(cmd=f"lock {obj.id} {looking_for}")
            return True
        except MissingPerceptionObject:
            fprint(
                f"The object {looking_for} is not in the database",
                msg_color="red",
            )
    try:
        objs = await navigator.database_query(object_name="all")
        objs = get_closest_objects(pos, objs.objects)
    except Exception as e:
        print(e)
        return False

    # for o in objs:
    #     obj_pos = nt.rosmsg_to_numpy(o.position)
    #     await navigator.move.look_at(obj_pos).set_position(mark_pos).backward(7).go()
    #     cam_obj = await navigator.camera_database_query(object_name=looking_for, id=o.id)
    #     if cam_obj.found:
    #         await navigator.database_query(cmd="lock {} {}".format(o.id, looking_for))
    #         fprint("EXPLORER FOUND OBJECT", msg_color="blue")
    #         return (True)

    fprint("NO OBJECT FOUND", msg_color="blue")
    return False


async def main(navigator, **kwargs):
    center_marker = kwargs["center_marker"]
    looking_for = kwargs["looking_for"]

    # FOR TESTING
    # center_marker = "Shooter"
    # looking_for = "scan_the_code"

    navigator.change_wrench("autonomous")
    await navigator.nh.sleep(0.1)
    good = await myfunc(navigator, looking_for, center_marker)
    return good
