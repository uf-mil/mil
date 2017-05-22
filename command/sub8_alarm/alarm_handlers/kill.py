import rospy
from ros_alarms import HandlerBase
from mil_msgs.srv import BaggerCommands


class Kill(HandlerBase):
    alarm_name = 'kill'
    initally_raised = True

    def __init__(self):
        self._killed = False
        self._last_mission_killed = False
        self.bagging_server = rospy.ServiceProxy('/online_bagger/dump', BaggerCommands)

    def raised(self, alarm):
        self._killed = True
        self.bagger_dump()

    def cleared(self, alarm):
        self._killed = False

    def bagger_dump(self):
        """Call online_bagger/dump service"""
        camera = '/camera/front/left/camera_info /camera/front/left/image_raw '
        navigation = '/wrench /wrench_actual /c3_trajectory_generator/trajectory_v \
                      /c3_trajectory_generator/waypoint /trajectory '
        controller = '/pd_out /rise_6dof/parameter_descriptions /rise_6dof/parameter_updates '
        blueview = '/blueview_driver/ranges /blueview_driver/image_color'
        kill_topics = camera + navigation + controller + blueview
        try:
            self.bagging_server(bag_name='kill_bag', bag_time=60, topics=kill_topics)
        except rospy.ServiceException as e:
            print "/online_bagger service failed: %s" % e

    def meta_predicate(self, meta_alarm, sub_alarms):
        ignore = []

        # Stay killed until user clears
        if self._killed:
            return True

        # Battery too low
        if sub_alarms["bus-voltage"].raised and sub_alarms["bus-voltage"].severity == 5:
            return True
        ignore.append("bus-voltage")

        if sub_alarms["odom-kill"].raised and sub_alarms["odom-kill"].severity == 5:
            return True
        ignore.append("odom-kill")

        # If we lose network but don't want to go autonomous
        if sub_alarms["network-loss"].raised and not rospy.get_param("autonomous", False):
            return True
        ignore.append("network-loss")

        # Severity level of 5 means too many thrusters out
        if sub_alarms["thruster-out"].raised and sub_alarms["thruster-out"].severity == 5:
            return True
        ignore.append("thruster-out")

        # If a mission wants us to kill, go ahead and kill
        if sub_alarms["mission-kill"].raised:
            self._last_mission_killed = True
            return True
        elif self._last_mission_killed:
            self._last_mission_killed = False

            # If we weren't killed by another source, clear the kill
            if not self._killed:
                return False
        ignore.append("mission-kill")

        # Raised if any alarms besides the two above are raised
        return any([alarm.raised for name, alarm in sub_alarms.items()
                    if name not in ignore])
