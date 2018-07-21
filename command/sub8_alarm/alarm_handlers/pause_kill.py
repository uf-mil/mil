import rospy
from ros_alarms import AlarmBroadcaster, HandlerBase


# Constantly monitor GPIO port to check if the kill plug is still connected
class PauseKill(HandlerBase):
    alarm_name = "pause-kill"

    def __init__(self):
        gpio = rospy.get_param("/pause-kill/gpio", 507)

        # Use sysfs to monitor gpio
        self.gpio_file = '/sys/class/gpio/gpio{}/value'.format(gpio)

        self.ab = AlarmBroadcaster(self.alarm_name, node_name='pause-kill')
        self._killed = False

        # The correct way would be use 'poll' syscall but meh
        rospy.Timer(rospy.Duration(0.01), self._check)

    def _check(self, *args):
        try:
            '''
                The following must be completed to enable GPIO sysfs
                echo "507" > /sys/class/gpio/export
                echo "in" > /sys/class/gpio/gpio507/direction
                chmod 777 /sys/class/gpio/gpio507/value
            '''
            file_open = open(self.gpio_file, 'r')
        except IOError:
            rospy.logwarn_throttle(60, 'Is Kill Plug GPIO enabled via sysfs?')
            return
        except Exception:
            rospy.logwarn_throttle(
                60, 'There was an error in opening GPIO for Kill Plug')
            return
        res = file_open.read(1)
        file_open.close()
        if not self._killed and not int(res):
            self._killed = True
            self.ab.raise_alarm(problem_description='KILL PULLED', severity=5)
        elif self._killed and int(res):
            self._killed = False
            self.ab.clear_alarm()

    def raised(self, alarm):
        self._killed = True

    def cleared(self, alarm):
        self._killed = False
