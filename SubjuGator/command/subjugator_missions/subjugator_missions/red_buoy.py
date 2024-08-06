#! /usr/bin/env python3

import math

from vision_stack.msg import ObjectDetections

from .sub_singleton import SubjuGatorMission

FRAME_WIDTH = 960  # pixels \/ \/ \/
FRAME_HEIGHT = 608
FRAME_AREA = FRAME_HEIGHT * FRAME_WIDTH

IDEAL_CENTER_X = 480
IDEAL_CENTER_Y = 304
CENTER_ERROR_RADIUS = 20

IDEAL_PERCENT_AREA = 30  # percent
PERCENT_ERROR = 5

SIDES = 8  # defines the shape that is drawn as the sub circles around the buoy

SPEED_LIMIT_YAW = 0.2
SPEED_LIMIT = 0.2

# Known values of the position of the buoy
AT_DISTANCE = 1.1  # meters
KNOWN_WIDTH = 240.7  # pixels
KNOWN_HEIGHT = 337.0  # pixels

FACTOR_OF_YAW = 0.000966284117592

FOV = 1.2  # radians or 50.42 degrees
V_FOV = 0.57  # radians or 32.7 degrees

DISTANCE_FACTOR = 182  # factor for the function c/x that relates width of the bounding box to the distance


class RedBuoyCirculation(SubjuGatorMission):
    async def run(self, args):
        # Subscribe to object detection message from detections
        self.detections_sub = self.nh.subscribe(
            "/yolo_detections/1/objectDetection_last_2/analysis",
            ObjectDetections,
        )

        await self.detections_sub.setup()

        # await self.analyze_yaw()

        # Await till we get a detection of the red buoy / search for buoy by yawing left and right
        await self.find_buoy()

        # Align sub with red buoy in the center
        await self.align_to_buoy()

        # Approach buoy until area is within the ideal percent area
        await self.approach_buoy()

        # Begin circling the buoy clockwise
        await self.circle_buoy()

        print("Done!")

    async def find_buoy(self):
        print("Finding Buoy")
        reassurance_score = 0
        scan_angle = 0  # degrees
        scan_angle_increment = 15
        while True and reassurance_score < 5:
            detections = await self.detections_sub.get_next_message()
            detections = detections.detections

            # Check for buoy
            found_buoy = False
            if detections and len(detections) > 0:

                # Check if buoy was one of the detections
                for detection in detections:
                    if detection.class_name == "Red buoy":
                        reassurance_score += 1
                        found_buoy = True

                        # TODO: Handle case where there might be two buoy detections

                        if reassurance_score >= 5:
                            # Store center and size of buoy
                            self.found_center_x = detection.center_x
                            self.found_center_y = detection.center_y
                            self.found_width = detection.width
                        break

            if found_buoy:
                scan_angle = (
                    0  # REset scan angle now that the sub is pointing towards the buoy
                )
                continue
            else:
                # Reset last scan angle
                await self.go(
                    self.move().yaw_right_deg(-scan_angle).zero_roll_and_pitch(),
                    speed=SPEED_LIMIT_YAW,
                )

                scan_angle = (
                    (abs(scan_angle) + scan_angle_increment)
                    if scan_angle < 0
                    else -(abs(scan_angle) + scan_angle_increment)
                )

                # TODO: handle if scan angle reaches 180 degrees

                # Scan other side
                await self.go(
                    self.move().yaw_right_deg(scan_angle).zero_roll_and_pitch(),
                    speed=SPEED_LIMIT_YAW,
                )

    async def analyze_yaw(self):
        did_start = False
        start_diff = 0
        start_vertcial_diff = 0

        while True:
            detections = await self.detections_sub.get_next_message()
            detections = detections.detections

            # Extract buoy detection
            if detections and len(detections) > 0:

                # Check if buoy was one of the detections
                for detection in detections:

                    if detection.class_name == "Red buoy" and not did_start:

                        # Log the starting distance from center of the buoy
                        start_diff = FRAME_WIDTH / 2 - detection.center_x
                        start_vertcial_diff = FRAME_HEIGHT / 2 - detection.center_y
                        did_start = True

                        # Print out the difference
                        print(start_diff, detection.center_x - FRAME_WIDTH / 2)
                        print("Predicted yaw offset:", (start_diff / FRAME_WIDTH) * FOV)

                        # Print out the vertical difference
                        print(
                            start_vertcial_diff,
                            detection.center_y - FRAME_HEIGHT / 2,
                        )
                        print(
                            "Predicted vertcial offsert: ",
                            math.tan((detection.height / FRAME_HEIGHT) * V_FOV)
                            * (detection.center_y - FRAME_HEIGHT / 2)
                            * 0.009,
                        )

                        print("Width of buoy: ", detection.width)

    async def align_to_buoy(self):
        did_start = False
        start_diff = 0
        start_vertcial_diff = 0

        while True:
            detections = await self.detections_sub.get_next_message()
            detections = detections.detections

            # Extract buoy detection
            if detections and len(detections) > 0:

                # Check if buoy was one of the detections
                for detection in detections:

                    if detection.class_name == "Red buoy" and not did_start:

                        # Align to buoy
                        start_diff = FRAME_WIDTH / 2 - detection.center_x
                        start_vertcial_diff = FRAME_HEIGHT / 2 - detection.center_y
                        did_start = True
                        self.found_width = detection.width

                        # Move once
                        await self.go(
                            self.move()
                            .yaw_left((start_diff / FRAME_WIDTH) * FOV)
                            .zero_roll_and_pitch(),
                            speed=SPEED_LIMIT,
                        )

                        vertical_shift_down = (
                            math.tan((1 - detection.height / FRAME_HEIGHT) * V_FOV)
                            * (detection.center_y - FRAME_HEIGHT / 2)
                            * 0.004
                        )

                        await self.go(
                            self.move().down(vertical_shift_down).zero_roll_and_pitch(),
                            speed=SPEED_LIMIT,
                        )

                        # Print out the difference
                        print(start_diff, detection.center_x - FRAME_WIDTH / 2)
                        print(
                            "Predicted yaw offset:",
                            (start_diff / FRAME_WIDTH) * FOV,
                        )

                        # Print out the vertical difference
                        print(
                            start_vertcial_diff,
                            detection.center_y - FRAME_HEIGHT / 2,
                        )
                        print("Predicted vertcial offsert: ", vertical_shift_down)

            if did_start:
                break
            else:
                await self.find_buoy()

    async def align_self_with_buoy(self):
        print("Aligning to Buoy")
        center_x = self.found_center_x
        center_y = self.found_center_y
        width = self.found_width

        x_move = 0.1
        y_move = 0.1

        while True:
            if (
                center_x < IDEAL_CENTER_X + CENTER_ERROR_RADIUS
                and center_x > IDEAL_CENTER_X - CENTER_ERROR_RADIUS
                and center_y < IDEAL_CENTER_Y + CENTER_ERROR_RADIUS
                and center_y > IDEAL_CENTER_Y - CENTER_ERROR_RADIUS
            ):
                self.found_width = width
                print("here 1")
                break
            else:
                if not (
                    center_x < IDEAL_CENTER_X + CENTER_ERROR_RADIUS
                    and center_x > IDEAL_CENTER_X - CENTER_ERROR_RADIUS
                ):
                    print("X: ", center_x, IDEAL_CENTER_X)
                    await self.go(
                        self.move().right(x_move).zero_roll_and_pitch(),
                        speed=SPEED_LIMIT,
                    )
                    print("Move right in meters: ", x_move)
                else:
                    print("x is lined up")
                if not (
                    center_y < IDEAL_CENTER_Y + CENTER_ERROR_RADIUS
                    and center_y > IDEAL_CENTER_Y - CENTER_ERROR_RADIUS
                ):
                    print("Y: ", center_y, IDEAL_CENTER_Y)
                    await self.go(
                        self.move().down(y_move).zero_roll_and_pitch(),
                        speed=SPEED_LIMIT,
                    )
                    print("Move down in meters: ", y_move)
                else:
                    print("y is lined up")

                detections = await self.detections_sub.get_next_message()
                detections = detections.detections

                # Check for buoy
                if len(detections) > 0:
                    # Check if buoy was one of the detections
                    for detection in detections:
                        if detection.class_name == "Red buoy":
                            self.found_center_x = detection.center_x
                            self.found_center_y = detection.center_y
                            self.found_width = detection.width
                            x_move = (
                                abs(x_move)
                                / abs(detection.center_x - center_x)
                                * (detection.center_x - IDEAL_CENTER_X)
                            )
                            y_move = (
                                abs(y_move)
                                / abs(detection.center_y - center_y)
                                * (detection.center_y - IDEAL_CENTER_Y)
                                * (FRAME_HEIGHT / FRAME_WIDTH)
                                * 0.4
                            )
                            print("Calculated step size: x,y", x_move, y_move)
                            center_x = detection.center_x
                            center_y = detection.center_y
                            width = detection.width
                            break

        print("Finished aligning self")

    async def approach_buoy(self):
        print("Approaching the Buoy: ", DISTANCE_FACTOR / self.found_width)
        percent_width = (self.found_width / FRAME_WIDTH) * 100
        distance_move = 0.1
        move_time = 0

        while True:
            if move_time == 2:
                break

            if (
                percent_width < IDEAL_PERCENT_AREA + PERCENT_ERROR
                and percent_width > IDEAL_PERCENT_AREA - PERCENT_ERROR
            ):
                break
            else:
                print(percent_width, IDEAL_PERCENT_AREA, "Moving: ", distance_move)
                if percent_width > IDEAL_PERCENT_AREA + PERCENT_ERROR:
                    await self.go(
                        self.move().backward(abs(distance_move)).zero_roll_and_pitch(),
                        speed=SPEED_LIMIT,
                    )
                elif percent_width < IDEAL_PERCENT_AREA - PERCENT_ERROR:
                    await self.go(
                        self.move().forward(abs(distance_move)).zero_roll_and_pitch(),
                        speed=SPEED_LIMIT,
                    )

                detections = await self.detections_sub.get_next_message()
                detections = detections.detections

                # Check for buoy
                if len(detections) > 0:
                    # Check if buoy was one of the detections
                    for detection in detections:
                        if detection.class_name == "Red buoy":
                            self.found_width = detection.width

                            distance_move = (
                                (KNOWN_WIDTH / detection.width) * AT_DISTANCE
                            ) * 0.5 - 0.2

                            percent_width = ((detection.width) / FRAME_WIDTH) * 100
                            self.found_width = detection.width

                            print(detection.width)
                            break

            move_time += 1

    async def circle_buoy(self):

        await self.align_to_buoy()

        yaw_angle = 180 * (SIDES - 2) / SIDES - 90
        distance_from_buoy = DISTANCE_FACTOR / self.found_width
        distance_per_side = 2 * (
            math.tan(math.radians(yaw_angle / 2)) * distance_from_buoy
        )

        print("Circling the Buoy", SIDES, yaw_angle, distance_per_side)

        print(yaw_angle)
        for i in range(SIDES + 1):

            await self.go(
                self.move()
                .left(
                    (
                        distance_per_side
                        if i != 0 and i != SIDES
                        else distance_per_side / 2
                    ),
                )
                .zero_roll_and_pitch(),
                speed=SPEED_LIMIT,
            )

            if i == SIDES:
                break

            await self.go(
                self.move().yaw_right_deg(yaw_angle).zero_roll_and_pitch(),
                speed=SPEED_LIMIT_YAW,
            )

            # await self.align_to_buoy()

            # await self.approach_buoy()
