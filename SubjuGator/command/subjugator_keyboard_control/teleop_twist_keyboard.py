#! /usr/bin/env python3


import sys
import threading
from select import select

import roslib
import rospy
from c3_trajectory_generator.srv import SetDisabled, SetDisabledRequest
from geometry_msgs.msg import Accel, Point, Pose, Twist, TwistStamped, Vector3
from mil_msgs.msg import PoseTwist, PoseTwistStamped
from nav_msgs.msg import Odometry
from ros_alarms import AlarmListener

roslib.load_manifest("teleop_twist_keyboard")


if sys.platform == "win32":
    import msvcrt
else:
    import termios
    import tty


TwistMsg = Twist

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    "i": (1, 0, 0, 0),
    "o": (1, 0, 0, -1),
    "j": (0, 0, 0, 1),
    "l": (0, 0, 0, -1),
    "u": (1, 0, 0, 1),
    ",": (-1, 0, 0, 0),
    ".": (-1, 0, 0, 1),
    "m": (-1, 0, 0, -1),
    "O": (1, -1, 0, 0),
    "I": (1, 0, 0, 0),
    "J": (0, 1, 0, 0),
    "L": (0, -1, 0, 0),
    "U": (1, 1, 0, 0),
    "<": (-1, 0, 0, 0),
    ">": (-1, -1, 0, 0),
    "M": (-1, 1, 0, 0),
    "t": (0, 0, 1, 0),
    "b": (0, 0, -1, 0),
}

speedBindings = {
    "q": (1.1, 1.1),
    "z": (0.9, 0.9),
    "w": (1.1, 1),
    "x": (0.9, 1),
    "e": (1, 1.1),
    "c": (1, 0.9),
}


class PublishThread(threading.Thread):
    def __init__(self, rate):
        super().__init__()
        self.publisher = rospy.Publisher("/trajectory", PoseTwistStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber(
            "/odom",
            Odometry,
            self.odom_callback,
            queue_size=1,
        )
        self.traj_gen_disable = rospy.ServiceProxy(
            "/c3_trajectory_generator/set_disabled",
            SetDisabled,
        )
        self.kill_listener = AlarmListener(
            "kill",
            callback_funct=self.update_alarm_status,
        )
        self.kill_raised = self.kill_listener.is_raised()
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_z = 0.0
        self.odom_orient = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.set_traj_generator(disable=True)
        self.start()

    def set_traj_generator(self, *, disable: bool):
        self.traj_gen_disable(SetDisabledRequest(disabled=disable))

    def update_alarm_status(self, alarm):
        self.kill_raised = alarm.raised

    def odom_callback(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_z = msg.pose.pose.position.z
        self.odom_orient = msg.pose.pose.orientation

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print(
                    "Waiting for subscriber to connect to {}".format(
                        self.publisher.name,
                    ),
                )
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn, *, no_raise=False):
        self.condition.acquire()
        if self.kill_raised and not no_raise:
            self.condition.release()
            raise RuntimeError('"kill" alarm is raised!')
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0, no_raise=True)
        pub_thread.set_traj_generator(disable=False)
        self.join()

    def run(self):
        twist_msg = TwistMsg()

        if stamped:
            twist = twist_msg.twist
            twist_msg.header.stamp = rospy.Time.now()
            twist_msg.header.frame_id = twist_frame
        else:
            twist = twist_msg
        while not self.done:
            if stamped:
                twist_msg.header.stamp = rospy.Time.now()
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            trajectory_msg = PoseTwistStamped()
            trajectory_msg.header.stamp = rospy.Time.now()
            trajectory_msg.header.frame_id = twist_frame

            pose_twist_msg = PoseTwist()
            pose_twist_msg.pose = Pose()
            pose_twist_msg.pose.position = Point(
                self.odom_x + (self.x * self.speed),
                self.odom_y + (self.y * self.speed),
                min(-0.5, self.odom_z + (self.z * self.speed)),
            )
            pose_twist_msg.pose.orientation = self.odom_orient
            pose_twist_msg.twist = twist
            pose_twist_msg.acceleration = Accel(
                Vector3(self.x, self.y, self.z),
                Vector3(0, 0, 0),
            )

            trajectory_msg.posetwist = pose_twist_msg
            self.publisher.publish(trajectory_msg)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0


def getKey(settings, timeout):
    if sys.platform == "win32":
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if rlist else ""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == "win32":
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == "win32":
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return f"currently:\tspeed {speed}\tturn {turn} "


if __name__ == "__main__":
    settings = saveTerminalSettings()

    rospy.init_node("teleop_twist_keyboard")

    speed = rospy.get_param("~speed", 0.1)
    turn = rospy.get_param("~turn", 1.0)
    speed_limit = rospy.get_param("~speed_limit", 1000)
    turn_limit = rospy.get_param("~turn_limit", 1000)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.5)
    stamped = rospy.get_param("~stamped", False)
    twist_frame = rospy.get_param("~frame_id", "")
    if stamped:
        TwistMsg = TwistStamped

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)

        print(msg)
        print(vels(speed, turn))
        while 1:
            key = getKey(settings, key_timeout)
            if key in moveBindings:
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings:
                speed = min(speed_limit, speed * speedBindings[key][0])
                turn = min(turn_limit, turn * speedBindings[key][1])
                if speed == speed_limit:
                    print("Linear speed limit reached!")
                if turn == turn_limit:
                    print("Angular speed limit reached!")
                print(vels(speed, turn))
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == "" and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if key == "\x03":
                    break

            pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)
