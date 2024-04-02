#!/usr/bin/env python3
import asyncio
from enum import Enum

import numpy as np
from mil_tools import quaternion_matrix, rosmsg_to_numpy
from std_srvs.srv import SetBoolRequest

from .navigator import NaviGatorMission

___author___ = "Alex Perez and Cameron Brown"


class MoveState(Enum):
    NOT_STARTED = 1
    RUNNING = 2
    CANCELLED = 3
    FINISHED = 4


class Navigation(NaviGatorMission):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.current_move_task_state = MoveState.NOT_STARTED
        self.last_move_task_state = MoveState.NOT_STARTED

    async def inspect_object(self, position):
        # Go in front of the object, looking directly at it
        try:
            await self.move.look_at(position).set_position(position).backward(6.0).go()
            await self.nh.sleep(5.0)
        except asyncio.CancelledError:
            print("Cancelled Inspection")

    def get_index_of_type(self, objects, classifications):
        if isinstance(classifications, str):
            classifications = [classifications]
        return next(
            i
            for i, o in enumerate(objects)
            if o.labeled_classification in classifications
            and o.id not in self.objects_passed
        )

    def get_indices_of_type(self, objects, classifications):
        if isinstance(classifications, str):
            classifications = [classifications]
        return [
            i
            for i, o in enumerate(objects)
            if o.labeled_classification in classifications
            and o.id not in self.objects_passed
        ]

    @staticmethod
    def get_gate(one, two, position):
        one = one[:2]
        two = two[:2]
        position = position[:2]
        delta = (one - two)[:2]
        rot_right = np.array([[0, -1], [1, 0]], dtype=np.float)
        perp_vec = rot_right.dot(delta)
        perp_vec = perp_vec / np.linalg.norm(perp_vec)
        center = (one + two) / 2.0
        distances = np.array(
            [
                np.linalg.norm((center + perp_vec) - position),
                np.linalg.norm((center - perp_vec) - position),
            ],
        )
        if np.argmin(distances) == 0:
            perp_vec = -perp_vec
        return np.array([center[0], center[1], 0.0]), np.array(
            [perp_vec[0], perp_vec[1], 0.0],
        )

    async def go_thru_gate(self, gate, BEFORE=5.0, AFTER=4.0):
        center, vec = gate
        before_position = center - (vec * BEFORE)
        after_position = center + (vec * AFTER)
        await self.move.set_position(before_position).look_at(center).go()
        if AFTER > 0:
            await self.move.look_at(after_position).set_position(after_position).go()

    async def do_next_gate(self):
        pose = await self.tx_pose()
        p = pose[0]
        q_mat = quaternion_matrix(pose[1])

        def filter_and_sort(objects, positions):
            # filter out buoys more than filter_distance behind boat
            filter_distance = -5
            positions_local = np.array(
                [(q_mat.T.dot(position - p)) for position in positions],
            )
            positions_local_x = np.array(positions_local[:, 0])
            forward_indices = np.argwhere(positions_local_x > filter_distance).flatten()
            forward_indices = np.array(
                [
                    i
                    for i in forward_indices
                    if objects[i].id not in self.objects_passed
                ],
            )
            distances = np.linalg.norm(positions_local[forward_indices], axis=1)
            indices = forward_indices[np.argsort(distances).flatten()].tolist()
            # ids = [objects[i].id for i in indices]
            # self.send_feedback('Im attracted to {}'.format(ids))
            return indices

        def is_done(objects, positions):
            try:
                left_index = self.get_index_of_type(
                    objects,
                    ("red_cylinder", "white_cylinder"),
                )
                if objects[left_index].labeled_classification != "white_cylinder":
                    right_index = self.get_index_of_type(
                        objects,
                        ("green_cylinder", "white_cylinder"),
                    )
                else:
                    indices = self.get_indices_of_type(objects, "white_cylinder")
                    right_index = indices[1]

            except (StopIteration, IndexError):
                return None

            end = objects[left_index].labeled_classification == "white_cylinder"
            return (
                positions[left_index],
                objects[left_index],
                positions[right_index],
                objects[right_index],
                end,
            )

        left, left_obj, right, right_obj, end = await self.explore_closest_until(
            is_done,
            filter_and_sort,
        )
        self.send_feedback(
            f"Going through gate of objects {left_obj.labeled_classification} and {right_obj.labeled_classification}",
        )
        gate = self.get_gate(left, right, p)
        await self.go_thru_gate(gate)
        self.objects_passed.add(left_obj.id)
        self.objects_passed.add(right_obj.id)
        return end

    def movement_finished(self, task):
        if not task.cancelled():
            print("THE MOVEMENT HAS FINISHED")
            self.current_move_task_state = MoveState.FINISHED

    async def explore_closest_until(self, is_done, filter_and_sort):
        """
        @condition func taking in sorted objects, positions
        @object_filter func filters and sorts
        """
        move_id_tuple = None
        init_boat_pos = self.pose[0]
        cone_buoys_investigated = 0  # max will be 2
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

                        if (
                            "cylinder"
                            in objects_msg.objects[
                                classification_index
                            ].labeled_classification
                        ):
                            init_boat_pos = rosmsg_to_numpy(
                                objects_msg.objects[classification_index].pose.position,
                            )
                            print(init_boat_pos)
                            cone_buoys_investigated += 1

                            if (
                                "green"
                                in objects_msg.objects[
                                    classification_index
                                ].labeled_classification
                                and cone_buoys_investigated < 2
                            ):
                                await self.move.left(4).yaw_left(30, "deg").go()
                            elif (
                                "red"
                                in objects_msg.objects[
                                    classification_index
                                ].labeled_classification
                                and cone_buoys_investigated < 2
                            ):
                                await self.move.right(4).yaw_right(30, "deg").go()

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

            # check if there are any buoys that have "cylinder" in the name that haven't been investigated
            # obtain the closest one to the previous gate and deem that the next buoy to investigate
            for i in range(len(objects)):
                print(f"Object {i}")
                if (
                    "cylinder" in objects[i].labeled_classification
                    and objects[i].id not in investigated
                ):
                    distance = np.linalg.norm(positions[i] - init_boat_pos)
                    if distance < shortest_distance:
                        shortest_distance = distance
                        print(shortest_distance)
                        print(positions[i])
                        print(
                            "POTENTIAL CANDIDATE: IDENTIFIED THROUGH MARKER THAT HAS NOT BEEN INVESTIGATED",
                        )
                        potential_candidate = i

            # if there no known cone buoys that haven't been investigated, check if we have already investigated one
            # and find closest one within 25 meters (max width of a gate).
            if cone_buoys_investigated > 0 and potential_candidate is None:
                for i in range(len(objects)):
                    print(positions[i])
                    if (
                        objects[i].id not in investigated
                        and "round" not in objects[i].labeled_classification
                    ):
                        distance = np.linalg.norm(positions[i] - self.pose[0])
                        if distance < shortest_distance and distance <= 25:
                            shortest_distance = distance
                            print(shortest_distance)
                            print(positions[i])
                            print(
                                "POTENTIAL CANDIDATE: IDENTIFIED BY FINDING CLOSEST CONE TO ALREADY INVESTIGATED CONE (<25m)",
                            )
                            potential_candidate = i

            # if that doesn't produce any results, literally just go to closest buoy
            if potential_candidate is None:
                for i in range(len(objects)):
                    if (
                        objects[i].id not in investigated
                        and "round" not in objects[i].labeled_classification
                    ):
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

            # explore the closest buoy to potential candidate
            if potential_candidate is not None:
                # if there exists a closest buoy, go to it
                self.send_feedback(f"Investigating {objects[potential_candidate].id}")
                investigated.add(objects[potential_candidate].id)
                move = self.inspect_object(positions[potential_candidate])
                move_id_tuple = (move, objects[potential_candidate].id)
                print("USING POTENTIAL CANDIDATE")

            if move_id_tuple is None:
                self.send_feedback("!!!! NO MORE TO EXPLORE")
                raise Exception("no more to explore")

    def get_objects_indices_for_start(self, objects):
        try:
            indices = self.get_indices_of_type(objects, "white_cylinder")
            white_index1 = indices[0]
            white_index2 = indices[1]
        except IndexError:
            return None
        except Exception:
            import traceback

            traceback.print_exc()
        return white_index1, white_index2

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

    async def prepare_to_enter(self):
        robot_position = (await self.tx_pose())[0]

        def filter_and_sort(objects, positions):
            distances = np.linalg.norm(positions - robot_position, axis=1)
            argsort = np.argsort(distances)
            return argsort

        def is_done(objects, positions):
            res = self.get_objects_indices_for_start(objects)
            if res is None:
                return None
            white_index1, white_index2 = res
            return (
                objects[white_index1],
                positions[white_index1],
                objects[white_index2],
                positions[white_index2],
            )

        white, white_position, red, red_position = await self.explore_closest_until(
            is_done,
            filter_and_sort,
        )
        self.objects_passed.add(white.id)
        self.objects_passed.add(red.id)
        gate = self.get_gate(white_position, red_position, robot_position)
        self.send_feedback(
            f"Going through start gate formed by {white.labeled_classification} and {red.labeled_classification}",
        )
        await self.go_thru_gate(gate, AFTER=-2)

    async def run(self, parameters):
        """
        TODO: check for new objects in background, cancel move
              somefucking how handle case where gates litteraly loop back and head the other direction
        """
        self.objects_passed = set()
        await self.change_wrench("autonomous")
        # Wait a bit for PCDAR to get setup
        await self.set_classifier_enabled.wait_for_service()
        await self.set_classifier_enabled(SetBoolRequest(data=True))
        await self.nh.sleep(3.0)
        await self.prepare_to_enter()
        await self.move.forward(7.0).go()
        while not (await self.do_next_gate()):
            pass
        self.send_feedback("Exiting last gate!! Go NaviGator")
        await self.set_classifier_enabled(SetBoolRequest(data=False))
