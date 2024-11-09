#!/usr/bin/env python3
import asyncio
from enum import Enum

import numpy as np
from mil_msgs.msg import ObjectsInImage
from mil_msgs.srv import CameraToLidarTransform
from mil_tools import rosmsg_to_numpy
from navigator_msgs.srv import MessageWildlifeEncounter, MessageWildlifeEncounterRequest

from .navigator import NaviGatorMission


class MoveState(Enum):
    NOT_STARTED = 1
    RUNNING = 2
    CANCELLED = 3
    FINISHED = 4


class Wildlife2024(NaviGatorMission):
    animals_observed = {
        "blue_manatee_buoy": False,  # Manatee => Counter clockwise
        "green_iguana_buoy": False,  # Iguana => Clockwise (by choice)
        "red_python_buoy": False,  # Python => Clockwise
    }

    @classmethod
    async def setup(cls):
        cls.camsub = cls.nh.subscribe("/bbox_pub", ObjectsInImage)
        await cls.camsub.setup()

        cls.camera_lidar_tf = cls.nh.get_service_client(
            "/wamv/sensors/camera/front_right_cam/image_raw",
            CameraToLidarTransform,
        )

    @classmethod
    async def shutdown(cls):
        await cls.camsub.shutdown()

    def get_indices_of_type(self, objects, classifications):
        if isinstance(classifications, str):
            classifications = [classifications]
        return [
            i
            for i, o in enumerate(objects)
            if o.labeled_classification in classifications
            and o.id not in self.objects_passed
        ]

    async def inspect_object(self, position):
        # Go in front of the object, looking directly at it
        try:
            await self.move.look_at(position).set_position(position).backward(6.0).go()
            await self.nh.sleep(5.0)
        except asyncio.CancelledError:
            print("Cancelled Inspection")

    @staticmethod
    def object_classified(objects, obj_id):
        """
        @objects list of object messages
        @obj_id id of object
        @return True of object with obj_id is classified
        """
        for i, obj in enumerate(objects):
            if obj.id == obj_id and obj.labeled_classification != "UNKNOWN":
                return i
        return -1

    def movement_finished(self, task):
        if not task.cancelled():
            print("THE MOVEMENT HAS FINISHED")
            self.current_move_task_state = MoveState.FINISHED

    # Explore until we find one of the animals we have not seen before
    async def explore_closest_until(self, is_done, filter_and_sort) -> dict:
        """
        @condition func taking in sorted objects, positions
        @object_filter func filters and sorts
        """
        move_id_tuple = None
        service_req = None
        investigated = set()
        move_task = None

        while True:
            if move_id_tuple is not None:
                service_req = self.database_query(name="all")

                result = await service_req
                if move_task is None:
                    self.current_move_task_state = MoveState.RUNNING
                    move_task = asyncio.create_task(move_id_tuple[0])
                    move_task.add_done_callback(self.movement_finished)

                # Database query succeeded
                if self.current_move_task_state != MoveState.FINISHED:
                    service_req = None
                    objects_msg = result
                    classification_index = self.object_classified(
                        objects_msg.objects,
                        move_id_tuple[1],
                    )

                    if classification_index != -1:
                        self.send_feedback(
                            f"{move_id_tuple[1]} identified. Canceling investigation",
                        )
                        move_task.cancel()

                        move_task = None
                        self.last_move_task_state = MoveState.CANCELLED
                        self.current_move_task_state = MoveState.NOT_STARTED
                        # move_id_tuple[0].cancel()

                        await self.nh.sleep(1.0)

                        self.send_feedback(f"Investigated {move_id_tuple[1]}")
                        move_id_tuple = None

                # Move succeeded:
                else:
                    self.send_feedback(f"Investigated {move_id_tuple[1]}")
                    move_id_tuple = None
                    self.last_move_task_state = MoveState.FINISHED
                    self.current_move_task_state = MoveState.NOT_STARTED
                    move_task = None
            else:
                print("\nAwaiting for data base\n")
                objects_msg = await self.database_query(name="all")

            objects = objects_msg.objects
            positions = np.array(
                [rosmsg_to_numpy(obj.pose.position) for obj in objects],
            )
            indices = [] if len(objects) == 0 else filter_and_sort(objects, positions)
            if indices is None or len(indices) == 0:
                self.send_feedback("No objects")
                continue
            objects = [objects[i] for i in indices]
            positions = positions[indices]

            # Exit if done
            ret = is_done(objects, positions)
            labels = [obj[0].labeled_classification for obj in ret]
            self.send_feedback(f"Analyzing objects: {labels}")
            if ret is not None:
                if move_id_tuple is not None:
                    self.send_feedback("Condition met. Canceling investigation")
                    move_task.cancel()
                    self.last_move_task_state = MoveState.NOT_STARTED
                    self.current_move_task_state = MoveState.NOT_STARTED
                    move_id_tuple = None
                return ret

            if move_id_tuple is not None:
                continue

            self.send_feedback(f"ALREADY INVEST {investigated}")

            #### The following is the logic for how we decide what buoy to investigate next ####
            potential_candidate = None
            shortest_distance = 1000

            for i in range(len(objects)):
                if objects[i].id not in investigated:
                    distance = np.linalg.norm(positions[i] - self.pose[0])
                    if distance < shortest_distance:
                        shortest_distance = distance
                        print(shortest_distance)
                        print(positions[i])
                        print(
                            "POTENTIAL CANDIDATE: IDENTIFIED BY FINDING CLOSEST CONE TO INIT BOAT POS",
                            distance,
                        )
                        potential_candidate = i
                        print(positions[i])

            if potential_candidate is not None:
                # if there exists a closest buoy, go to it
                self.send_feedback(f"Investigating {objects[potential_candidate].id}")
                investigated.add(objects[potential_candidate].id)
                move = self.inspect_object(positions[potential_candidate])
                move_id_tuple = (move, objects[potential_candidate].id)
                print("USING POTENTIAL CANDIDATE")

    async def circle_animal(self, animal):
        object = animal[0]
        position = animal[1]
        label = object.labeled_classification
        self.send_feedback(f"Circling {label}...")

        # Go to point and Circle animal
        await self.move.d_spiral_point(
            position,
            6,
            4,
            1,
            (
                "ccw"
                if label == "green_iguana_buoy" or label == "red_python_buoy"
                else "cw"
            ),
            theta_offset=(
                -1.57
                if label == "green_iguana_buoy" or label == "red_python_buoy"
                else 1.57
            ),
        )

    def get_indices_of_most_confident_animals(
        self,
        objects,
        classifications=["red_python_buoy", "blue_manatee_buoy", "green_iguana_buoy"],
    ) -> dict:
        """Pass in sorted list of objects by distance from boat and extract the first instance of classifications"""
        animals_dict = {classif: -1 for classif in classifications}
        for i, obj in enumerate(objects):
            if (
                obj.labeled_classification in classifications
                and animals_dict[obj.labeled_classification] == -1
            ):
                animals_dict[obj.labeled_classification] = i
        return animals_dict

    async def find_wildlife(self):
        robot_position = (await self.tx_pose())[0]
        self.send_feedback("[wildlife] FINDING WILDLIFE")

        def filter_and_sort(objects, positions):
            distances = np.linalg.norm(positions - robot_position, axis=1)
            argsort = np.argsort(distances)
            return argsort

        def is_done(objects, positions):
            res = self.get_indices_of_most_confident_animals(objects)
            # It is only done exploring if we detect an animal we have not circled or no objects were found
            found_new_animals = []
            for key in res:
                if self.animals_observed[key] or res[key] == -1:
                    continue
                else:
                    index = res[key]
                    found_new_animals.append((objects[index], positions[index]))

            if len(found_new_animals) > 0:
                return found_new_animals
            else:
                return None

        # Get the objects found in exploration
        animals = await self.explore_closest_until(
            is_done,
            filter_and_sort,
        )

        # Go to each object and circle them accordingly
        for animal in animals:
            object = animal[0]
            label = object.labeled_classification
            # Update explore dict
            self.animals_observed[label] = True
            if label == self.chosen_animal:
                self.send_feedback(
                    f"[wildlife] starting circle of {self.chosen_animal}",
                )
                td_animal_labels = {
                    "red_python_buoy": "P",
                    "green_iguana_buoy": "I",
                    "blue_manatee_buoy": "M",
                }
                self.send_feedback("[wildlife] sending feedback")
                await self.td_feedback(
                    MessageWildlifeEncounterRequest(
                        circling_wildlife=td_animal_labels.get(label, "P"),
                        clockwise=label == "blue_manatee_buoy",
                        number_of_circles=2 if label == "red_python_buoy" else 1,
                    ),
                )
                self.send_feedback("[wildlife] sent feedback!")
                await self.circle_animal(animal)
                if self.chosen_animal == "red_python_buoy":
                    self.send_feedback("[wildlife] starting double circle of red")
                    await self.circle_animal(animal)

        # # Check if all wildlife has been circled
        # IF not all wildlife has been found call this function again
        count = 0
        for found in self.animals_observed.values():
            if not found:
                break
            count += 1

        if count < 3:
            await self.find_wildlife()
        else:
            print("ALL WILDLIFE OBSERVED!")

    async def run(self, args):
        # Check nearest objects
        self.objects_passed = set()
        try:
            self.color_sequence = await self.nh.get_param("color_sequence", str)
        except TypeError:
            print("invalid color sequence type")
            self.color_sequence = None
        if not self.color_sequence:
            self.color_sequence = "RGB"
            self.send_feedback("Invalid color sequence, defaulting to RGB")
        animals = {
            "R": "red_python_buoy",
            "G": "green_iguana_buoy",
            "B": "blue_manatee_buoy",
        }
        self.chosen_animal = animals.get(self.color_sequence[0], "red_python_buoy")
        self.send_feedback(f"[wildlife] circling {self.chosen_animal}")
        self.td_feedback = self.nh.get_service_client(
            "/wildlife_encounter_message",
            MessageWildlifeEncounter,
        )
        await self.change_wrench("autonomous")
        # Wait a bit for PCDAR to get setup
        # await self.set_classifier_enabled.wait_for_service()
        # await self.set_classifier_enabled(SetBoolRequest(data=True))
        await self.nh.sleep(3.0)
        await self.find_wildlife()
