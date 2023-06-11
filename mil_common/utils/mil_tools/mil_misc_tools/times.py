import datetime

import rospy


def rospy_to_datetime(time: rospy.Time) -> datetime.datetime:
    return datetime.datetime.utcfromtimestamp(time.to_sec())


def datetime_to_rospy(dt: datetime.datetime) -> rospy.Time:
    return rospy.Time.from_sec(dt.replace(tzinfo=datetime.timezone.utc).timestamp())
