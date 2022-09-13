#!/usr/bin/python3
import asyncio
import subprocess
import sys

import txros
import uvloop
from geometry_msgs.msg import PoseStamped
from mil_misc_tools import text_effects
from mil_msgs.msg import DepthStamped, RangeStamped, VelocityMeasurements
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, Imu, Joy, MagneticField
from std_msgs.msg import Header
from sub8_msgs.msg import ThrusterStatus
from tf.msg import tfMessage

MESSAGE_TIMEOUT = 1  # s


class TemplateChecker:

    """Template for how each checker class should look.
    This provides interface functions for the main function to use. You don't have to
        implement them all in each checker.
    """

    def __init__(self, nh, name):
        self.nh = nh
        self.name = name

        self.p = text_effects.Printer()

    async def tx_init(self):
        # Do txros initiation here
        yield
        pass

    def do_check(self):
        # Do actual checking here, either pass or fail
        pass

    def pass_check(self, status="", name=None):
        # Message that prints when the check passes
        if name is None:
            name = self.name
        pass_text = self.p.text("[ ").set_green.bold("PASS").text(" ]")
        pass_text += self.p.space().bold(name)
        if status != "":
            pass_text += self.p.text(f": {status}")

        print(pass_text)

    def warn_check(self, reason="", name=None):
        # Message that prints when the check has a warning
        if name is None:
            name = self.name
        warn_text = self.p.text("[ ").set_yellow.bold("WARN").text(" ]")
        warn_text += self.p.space().bold(name)
        if reason != "":
            warn_text += self.p.text(f": {reason}")

        print(warn_text)

    def fail_check(self, reason="", name=None):
        # Message that prints when the check fails
        if name is None:
            name = self.name
        fail_text = self.p.text("[ ").set_red.bold("FAIL").text(" ]")
        fail_text += self.p.space().bold(name)
        if reason != "":
            fail_text += self.p.text(f": {reason}")

        print(fail_text)

    def err_msg(self, error, line_number):
        # If there is an error on runtime, this runs
        fail_text = self.p.text("[ ").set_red.bold(" ERR").text(" ]")
        fail_text += self.p.space().bold(self.name).text(f": ({line_number})")
        fail_text += self.p.space().text(f"{error}")

        print(fail_text)


class ThrusterChecker(TemplateChecker):
    async def tx_init(self):
        self.thruster_topic = self.nh.subscribe(
            "thrusters/thruster_status", ThrusterStatus
        )
        await self.thruster_topic.setup()

        thrusters = None
        try:
            thrusters = await self.nh.get_param("/thruster_layout/thrusters")
        except txros.XMLRPCException as e:
            raise OSError(e)

        self.found_thrusters = {}
        for thruster_name in thrusters.keys():
            self.found_thrusters[thruster_name] = False
        print(self.p.bold("  >>>>   ").set_blue.bold("Thruster Check"))

    async def do_check(self):
        try:
            passed = await txros.util.wrap_timeout(
                self.get_all_thrusters(), MESSAGE_TIMEOUT
            )
        except asyncio.TimeoutError:
            lost_thrusters = [x[0] for x in self.found_thrusters.items() if not x[1]]
            err_msg = ""
            if not any(self.found_thrusters.values()):
                self.fail_check("no messages found.")
            elif self.found_thrusters.values().count(False) > 1:
                err_msg += f"more than one failed thruster: {lost_thrusters}"
                self.fail_check(err_msg)
            elif self.found_thrusters.values().count(False) == 1:
                err_msg += "one thruster is out ({}), things should still work.".format(
                    lost_thrusters
                )
                self.warn_check(err_msg)
            else:
                self.warn_check("unknown timeout reason.")

            return False

        if passed:
            self.pass_check("all thrusters up.")
        else:
            self.warn_check("unknown failure.")

    async def get_all_thrusters(self):
        while True:
            msg = await self.thruster_topic.get_next_message()
            self.found_thrusters[msg.name] = True

            if all(self.found_thrusters.values()):
                return True


class CameraChecker(TemplateChecker):
    async def tx_init(self):
        self.front_cam_product_id = "1e10:3300"  # should be changed if cameras change
        self.right = self.nh.subscribe("/camera/front/right/image_rect_color", Image)
        self.right_info = self.nh.subscribe(
            "/camera/front/right/camera_info", CameraInfo
        )
        self.left = self.nh.subscribe("/camera/front/left/image_rect_color", Image)
        self.left_info = self.nh.subscribe("/camera/front/left/camera_info", CameraInfo)
        self.down = self.nh.subscribe(
            "/camera/down/left/image_rect_color", Image
        )  # TODO
        self.down_info = self.nh.subscribe(
            "/camera/down/left/camera_info", CameraInfo
        )  # TODO

        self.subs = [
            ("Right Image", self.right.get_next_message()),
            ("Right Info", self.right_info.get_next_message()),
            ("Left Image", self.left.get_next_message()),
            ("Left Info", self.left_info.get_next_message()),
            ("Down Image", self.down.get_next_message()),
            ("Down Info", self.down_info.get_next_message()),
        ]
        await asyncio.gather(
            self.right.setup(),
            self.right_info.setup(),
            self.left.setup(),
            self.left_info.setup(),
            self.down.setup(),
            self.down_info.setup(),
        )

        print(self.p.bold("\n  >>>>   ").set_blue.bold("Camera Check"))
        await self.nh.sleep(0.1)  # Try to get all the images

    async def do_check(self):
        # Check if front cameras are actually on usb bus
        command = f"lsusb -d {self.front_cam_product_id}"
        err_str = "{} front camera{} not connected to usb port"
        try:
            count_front_cam_usb = subprocess.check_output(
                ["/bin/sh", "-c", command]
            ).count("Point Grey")
            if count_front_cam_usb < 2:
                self.fail_check(err_str.format("One", ""))
        except subprocess.CalledProcessError:
            self.fail_check(err_str.format("Both", "s"))

        for name, df in self.subs:
            try:
                await txros.util.wrap_timeout(df, MESSAGE_TIMEOUT)
            except txros.util.TimeoutError:
                self.fail_check("no messages found.", name)
                continue

            self.pass_check("message found.", name)


class StateEstChecker(TemplateChecker):
    async def tx_init(self):
        self.odom = self.nh.subscribe("/odom", Odometry)
        self.tf = self.nh.subscribe("/tf", tfMessage)
        self.dvl = self.nh.subscribe("/dvl", VelocityMeasurements)
        self.depth = self.nh.subscribe("/depth", DepthStamped)
        self.height = self.nh.subscribe("/dvl/range", RangeStamped)
        self.imu = self.nh.subscribe("/imu/data_raw", Imu)
        self.mag = self.nh.subscribe("/imu/mag", MagneticField)

        self.subs = [
            ("Odom", self.odom.get_next_message()),
            ("TF", self.tf.get_next_message()),
            ("DVL", self.dvl.get_next_message()),
            ("Height", self.height.get_next_message()),
            ("Depth", self.depth.get_next_message()),
            ("IMU", self.imu.get_next_message()),
            ("Mag", self.mag.get_next_message()),
        ]
        await asyncio.gather(
            self.odom.setup(),
            self.tf.setup(),
            self.dvl.setup(),
            self.depth.setup(),
            self.height.setup(),
            self.imu.setup(),
            self.mag.setup(),
        )

        print(self.p.bold("\n  >>>>   ").set_blue.bold("State Estimation Check"))
        await self.nh.sleep(0.1)  # Try to get all the subs

    async def do_check(self):
        for name, df in self.subs:
            try:
                res = await txros.util.wrap_timeout(df, MESSAGE_TIMEOUT)
            except asyncio.TimeoutError:
                self.fail_check("no messages found.", name)
                continue

            if name == "DVL":
                len_measurements = len(res.velocity_measurements)
                if len_measurements == 4:
                    self.pass_check("all four beams being published.", name)
                elif len_measurements == 0:
                    self.fail_check("no beams being published", name)
                else:
                    self.warn_check("not all beams are being published", name)
            else:
                self.pass_check("message found.", name)


class ShoreControlChecker(TemplateChecker):
    async def tx_init(self):
        p = (
            self.p.bold("\n  >>>>")
            .text("   Press return when ")
            .negative("shore_control.launch")
            .text(" is running.\n")
        )
        txros.util.nonblocking_raw_input(str(p))

        self.network = self.nh.subscribe("/network", Header)
        self.spacenav = self.nh.subscribe("/spacenav/joy", Joy)
        self.posegoal = self.nh.subscribe("/posegoal", PoseStamped)
        await asyncio.gather(
            self.network.setup(),
            self.spacenav.setup(),
            self.posegoal.setup(),
        )

        self.subs = [
            ("Network", self.network.get_next_message()),
            ("Spacenav", self.spacenav.get_next_message()),
            ("Pose Goal", self.posegoal.get_next_message()),
        ]

        print(self.p.bold("  >>>>   ").set_blue.bold("Shore Control Check"))
        await self.nh.sleep(0.1)  # Try to get all the subs

    async def do_check(self):
        for name, df in self.subs:
            try:
                await txros.util.wrap_timeout(df, MESSAGE_TIMEOUT)
            except asyncio.TimeoutError:
                self.fail_check("no messages found.", name)
                continue

            self.pass_check("message found.", name)


async def main():
    nh = await txros.NodeHandle.from_argv("startup_checker")
    check_order = [
        ThrusterChecker(nh, "Thrusters"),
        CameraChecker(nh, "Cameras"),
        StateEstChecker(nh, "State Estimation"),
        ShoreControlChecker(nh, "Shore Control"),
    ]

    p = text_effects.Printer()
    txros.util.nonblocking_raw_input(
        str(
            p.bold("\n  >>>>")
            .text("   Press return when ")
            .negative("sub8.launch")
            .text(" is running.")
        )
    )
    print(p.newline().set_blue.bold("-------- Running self checks...").newline())

    for check in check_order:
        try:
            await check.tx_init()
            await nh.sleep(0.1)
            await check.do_check()
        except Exception as e:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            check.err_msg(e, exc_tb.tb_lineno)

        del check

    print(p.newline().set_blue.bold("-------- Finished!").newline())


if __name__ == "__main__":
    uvloop.install()
    asyncio.run(main())
