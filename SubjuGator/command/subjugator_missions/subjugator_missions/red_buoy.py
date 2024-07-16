#! /usr/bin/env python3

from vision_stack.msg import ObjectDetections

from .sub_singleton import SubjuGatorMission

FRAME_WIDTH = 960  # pixels \/ \/ \/
FRAME_HEIGHT = 608
FRAME_AREA = FRAME_HEIGHT * FRAME_WIDTH

IDEAL_CENTER_X = 480
IDEAL_CENTER_Y = 304
CENTER_ERROR_RADIUS = 20

IDEAL_PERCENT_AREA = 20  # percent
PERCENT_ERROR = 5

SIDES = 8  # defines the shape that is drawn as the sub circles around the buoy

SPEED_LIMIT_YAW = 0.2
SPEED_LIMIT = 0.2

# Known values of the position of the buoy
AT_DISTANCE = 1.1  # meters
KNOWN_WIDTH = 240.7  # pixels
KNOWN_HEIGHT = 337.0  # pixels


class RedBuoyCirculation(SubjuGatorMission):
    async def run(self, args):
        # Subscribe to object detection message from detections
        self.detections_sub = self.nh.subscribe(
            "/yolo_detections/1/objectDetection_last_2/analysis",
            ObjectDetections,
        )
        await self.detections_sub.setup()

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

    async def align_self_with_buoy(self):
        print("Aligning to Buoy")
        center_x = self.found_center_x
        center_y = self.found_center_y
        area = self.found_area

        x_move = 0.1
        y_move = 0.1

        while True:
            if (
                center_x < IDEAL_CENTER_X + CENTER_ERROR_RADIUS
                and center_x > IDEAL_CENTER_X - CENTER_ERROR_RADIUS
                and center_y < IDEAL_CENTER_Y + CENTER_ERROR_RADIUS
                and center_y > IDEAL_CENTER_Y - CENTER_ERROR_RADIUS
            ):
                self.found_area = area
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
                            self.found_area = detection.width * detection.height
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
                            area = detection.width * detection.height
                            break

        print("Finished aligning self")

    async def approach_buoy(self):
        print("Approaching the Buoy")
        percent_area = (self.found_area / FRAME_AREA) * 100
        distance_move = 0.1

        while True:
            if (
                percent_area < IDEAL_PERCENT_AREA + PERCENT_ERROR
                and percent_area > IDEAL_PERCENT_AREA - PERCENT_ERROR
            ):
                break
            else:
                print(percent_area, IDEAL_PERCENT_AREA, "Moving: ", distance_move)
                if percent_area > IDEAL_PERCENT_AREA + PERCENT_ERROR:
                    await self.go(
                        self.move().backward(abs(distance_move)).zero_roll_and_pitch(),
                        speed=SPEED_LIMIT,
                    )
                elif percent_area < IDEAL_PERCENT_AREA - PERCENT_ERROR:
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
                            self.found_area = detection.width * detection.height

                            distance_move = (
                                (KNOWN_WIDTH / detection.width) * AT_DISTANCE
                            ) * 0.5 - 0.5

                            percent_area = (
                                (detection.width * detection.height) / FRAME_AREA
                            ) * 100

                            print(detection.width)
                            break

    async def circle_buoy(self):
        print("Circling the Buoy")
        distance_per_side = 0.5
        yaw_angle = 180 * (SIDES - 2) / SIDES - 90
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

            await self.align_self_with_buoy()

            await self.approach_buoy()
