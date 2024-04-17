import datetime

import rclpy


def rospy_to_datetime(time: rclpy.Time) -> datetime.datetime:
    return datetime.datetime.utcfromtimestamp(time.to_sec())


def datetime_to_rospy(dt: datetime.datetime) -> rclpy.Time:
    return rclpy.Time.from_sec(dt.replace(tzinfo=datetime.timezone.utc).timestamp())
