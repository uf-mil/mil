#!/usr/bin/env python3
import asyncio

import txros
import uvloop
from sensor_msgs.msg import CompressedImage, Image


class FancyPrint:
    GOOD = "\033[32m"
    BAD = "\033[31m"
    NORMAL = "\033[0m"
    BOLD = "\033[1m"

    @classmethod
    def okay(self, text):
        print(self.GOOD + text + self.NORMAL)

    @classmethod
    def error(self, text):
        print(self.BAD + text + self.NORMAL)


def add_camera_feeds(nh, cam_name, image_type="image_raw"):
    """Returns subscribers to the raw and compressed `image_type`"""

    raw = f"/{cam_name}/{image_type}"
    compressed = f"/{cam_name}/{image_type}/compressed"
    return (
        nh.subscribe(raw, Image).get_next_message(),
        nh.subscribe(compressed, CompressedImage).get_next_message(),
    )


async def main():
    nh = await txros.NodeHandle.from_argv("self_checker")
    # Add deferreds to this dict to be yieleded on and checked later
    topics = {}

    # General Subs
    from nav_msgs.msg import Odometry

    topics["odom"] = nh.subscribe("odom", Odometry).get_next_message()

    from sensor_msgs.msg import Joy

    topics["joy"] = nh.subscribe("joy", Joy).get_next_message()

    # Perception Subs
    topics["right_images"], topics["right_compressed"] = add_camera_feeds(
        nh, "camera/starboard"
    )
    topics["front_right_images"], topics["front_right_compressed"] = add_camera_feeds(
        nh, "camera/front/right"
    )
    topics["front_left_images"], topics["front_left_compressed"] = add_camera_feeds(
        nh, "camera/front/left"
    )

    from sensor_msgs.msg import PointCloud2

    topics["velodyne"] = nh.subscribe(
        "/velodyne_points", PointCloud2
    ).get_next_message()

    # Thrusters
    from roboteq_msgs.msg import Feedback

    topics["BL_motor"] = nh.subscribe("/BL_motor/Feedback", Feedback).get_next_message()
    topics["BR_motor"] = nh.subscribe("/BR_motor/Feedback", Feedback).get_next_message()
    topics["FL_motor"] = nh.subscribe("/FL_motor/Feedback", Feedback).get_next_message()
    topics["FR_motor"] = nh.subscribe("/FR_motor/Feedback", Feedback).get_next_message()

    for name, sub in topics.items():
        try:
            # Bold the name so it's distinct
            fancy_name = FancyPrint.BOLD + name + FancyPrint.NORMAL
            print(f" - - - - Testing for {fancy_name}")

            # 2 second timeout should be good
            result = await txros.util.wrap_timeout(sub, 2)
            if result is None:
                FancyPrint.error(f"[ FAIL ] Response was None from {fancy_name}")
            else:
                FancyPrint.okay(f"[ PASS ] Response found from {fancy_name}")

        except BaseException:
            FancyPrint.error(f"[ FAIL ] No response from {fancy_name}")


if __name__ == "__main__":
    uvloop.install()
    asyncio.run(main())
