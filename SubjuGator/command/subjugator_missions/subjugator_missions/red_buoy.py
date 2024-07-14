#! /usr/bin/env python3

from vision_stack.msg import ObjectDetections

from .sub_singleton import SubjuGatorMission

FRAME_WIDTH = 960  # pixels \/ \/ \/
FRAME_HEIGHT = 608
FRAME_AREA = FRAME_HEIGHT * FRAME_WIDTH

IDEAL_CENTER_X = 480
IDEAL_CENTER_Y = 304
CENTER_ERROR_RADIUS = 5

IDEAL_PERCENT_AREA = 40  # percent
PERCENT_ERROR = 5

SIDES = 8  # defines the shape that is drawn as the sub circles around the buoy

SPEED_LIMIT_YAW = 0.2
SPEED_LIMIT = 0.2


class RedBuoyCirculation(SubjuGatorMission):
    async def run(self, args):
        # Subscribe to object detection message from detections
        self.detections_sub = self.nh.subscribe(
            "/yolo_detections/1/objectDetection_last_2/analysis",
            ObjectDetections,
        )

        # Await till we get a detection of the red buoy / search for buoy by yawing left and right
        await self.find_buoy()

        # Align sub with red buoy in the center
        await self.align_self_with_buoy()

        # Approach buoy until area is within the ideal percent area
        await self.approach_buoy()

        # Begin circling the buoy clockwise
        await self.circle_buoy()

        print("Done!")

    async def find_buoy(self):
        reassurance_score = 0
        scan_angle = 0  # degrees
        scan_angle_increment = 15
        while True and reassurance_score < 5:
            detections = self.detections_sub.get_next_message().result().detections

            # Check for buoy
            found_buoy = False
            if detections.count() > 0:

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
                            self.found_area = detection.width * detection.height
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

                scan_angle = -(abs(scan_angle + scan_angle_increment))

                # TODO: handle if scan angle reaches 180 degrees

                # Scan other side
                await self.go(
                    self.move().yaw_right_deg(scan_angle).zero_roll_and_pitch(),
                    speed=SPEED_LIMIT_YAW,
                )

    async def align_self_with_buoy(self):
        center_x = self.found_center_x
        center_y = self.found_center_y
        area = self.found_area

        while True:
            if (
                center_x < IDEAL_CENTER_X + CENTER_ERROR_RADIUS
                and center_x > IDEAL_CENTER_X - CENTER_ERROR_RADIUS
                and center_y < IDEAL_CENTER_Y + CENTER_ERROR_RADIUS
                and center_y > IDEAL_CENTER_Y - CENTER_ERROR_RADIUS
            ):
                self.percent_area = area
                break
            else:
                if not (
                    center_x < IDEAL_CENTER_X + CENTER_ERROR_RADIUS
                    and center_x > IDEAL_CENTER_X - CENTER_ERROR_RADIUS
                ):
                    await self.go(
                        self.move()
                        .right(0.1 if center_x < IDEAL_CENTER_X else -0.1)
                        .zero_roll_and_pitch(),
                        speed=SPEED_LIMIT,
                    )
                if not (
                    center_y < IDEAL_CENTER_Y + CENTER_ERROR_RADIUS
                    and center_y > IDEAL_CENTER_Y - CENTER_ERROR_RADIUS
                ):
                    await self.go(
                        self.move()
                        .up(0.1 if center_y > IDEAL_CENTER_Y else -0.1)
                        .zero_roll_and_pitch(),
                        speed=SPEED_LIMIT,
                    )

                detections = self.detections_sub.get_next_message().result().detections

                # Check for buoy
                if detections.count() > 0:
                    # Check if buoy was one of the detections
                    for detection in detections:
                        if detection.class_name == "Red buoy":
                            self.found_center_x = detection.center_x
                            self.found_center_y = detection.center_y
                            self.found_area = detection.width * detection.height
                            center_x = detection.center_x
                            center_y = detection.center_y
                            area = detection.width * detection.height
                            break

    async def approach_buoy(self):
        percent_area = (self.found_area / FRAME_AREA) * 100

        while True:
            if (
                percent_area < IDEAL_PERCENT_AREA + PERCENT_ERROR
                and percent_area > IDEAL_PERCENT_AREA - PERCENT_ERROR
            ):
                break
            else:
                if percent_area > IDEAL_PERCENT_AREA + PERCENT_ERROR:
                    await self.go(
                        self.move().backward(0.05).zero_roll_and_pitch(),
                        speed=SPEED_LIMIT,
                    )
                elif percent_area < IDEAL_PERCENT_AREA - PERCENT_ERROR:
                    await self.go(
                        self.move().forward(0.05).zero_roll_and_pitch(),
                        speed=SPEED_LIMIT,
                    )

    async def circle_buoy(self):

        distance_per_side = 8 / SIDES

        for _ in range(SIDES):

            await self.go(
                self.move().left(distance_per_side).zero_roll_and_pitch(),
                speed=SPEED_LIMIT,
            )

            await self.align_self_with_buoy()

            await self.approach_buoy()
