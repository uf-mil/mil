#!/usr/bin/env python3
import numpy as np
from dynamic_reconfigure.msg import BoolParameter, DoubleParameter, IntParameter
from geographic_msgs.msg import GeoPoseStamped
from mil_tools import rosmsg_to_numpy
from std_srvs.srv import SetBoolRequest

from .vrx import Vrx

___author___ = "Kevin Allen"


class VrxPerception(Vrx):
    def __init__(self, *args, **kwargs):
        self.announced = set()
        super().__init__(*args, **kwargs)

    async def get_object_map(self):
        """
        Gets the latest objects
        returns tuple (object_dict, position_dict)
        object_dict: maps object id to classification
        position_dict: maps object_id to position in enu as numpy array
        """
        current_objects_msg = (await self.database_query(name="all")).objects
        ret = {}
        positions = {}
        for obj in current_objects_msg:
            classification = obj.labeled_classification
            ret[obj.id] = classification
            positions[obj.id] = rosmsg_to_numpy(obj.pose.position)
        return (ret, positions)

    async def announce_object(
        self, obj_id, classification, position_enu, boat_position_enu
    ):
        if classification == "UNKNOWN":
            self.send_feedback(f"Ignoring UNKNOWN object {obj_id}")
            return False
        if obj_id in self.announced:
            return False
        if np.linalg.norm(position_enu - boat_position_enu) < 3.5:
            self.send_feedback(f"Ignoring {obj_id} b/c its too close")
            return False
        geo_point = await self.enu_position_to_geo_point(position_enu)
        msg = GeoPoseStamped()
        msg.header.frame_id = classification
        msg.pose.position = geo_point
        self.perception_landmark.publish(msg)
        return True

    async def run(self, parameters):
        p1 = BoolParameter(name="associator_forget_unseen", value=True)
        p2 = DoubleParameter(name="cluster_tolerance_m", value=0.3)
        p3 = DoubleParameter(name="associator_max_distance", value=0.25)
        p4 = IntParameter(name="cluster_min_points", value=1)
        p5 = IntParameter(name="persistant_cloud_filter_min_neighbors", value=1)
        await self.pcodar_set_params(bools=[p1], doubles=[p2, p3], ints=[p4, p5])
        # TODO: use PCODAR amnesia to avoid this timing fiasco
        await self.wait_for_task_such_that(lambda task: task.state in ["running"])
        await self.set_vrx_classifier_enabled(SetBoolRequest(data=True))
        objects = {}
        while True:
            await self.task_info_sub.get_next_message()
            new_objects, positions = await self.get_object_map()
            position_enu = (await self.tx_pose)[0]
            for key in new_objects:
                if key not in objects:
                    self.send_feedback(f"NEW object {key} {new_objects[key]}")
                    await self.announce_object(
                        key, new_objects[key], positions[key], position_enu
                    )
                elif objects[key] != new_objects[key]:
                    self.send_feedback(
                        "{} changed from {} to {}".format(
                            key, objects[key], new_objects[key]
                        )
                    )
                    await self.announce_object(
                        key, new_objects[key], positions[key], position_enu
                    )
            objects = new_objects
