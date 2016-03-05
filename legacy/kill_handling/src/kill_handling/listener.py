import rospy

from kill_handling.msg import KillsStamped
from kill_handling.msg import Kill


class KillListener(object):
    def _killmsg_callback(self, msg):
        self._kills = msg.kills

        self._check_killed()

    # Standard method to check if killed and call callbacks if appropriate
    def _check_killed(self):
        killed = self.get_killed()
        if killed and not self._previously_killed:
            self._killed_callback()
        elif not killed and self._previously_killed:
            self._unkilled_callback()
        self._previously_killed = killed

    # Constructor with killed callback and unkilled callback
    def __init__(self, killed_callback=lambda: None, unkilled_callback=lambda: None):
        self._killed_callback = killed_callback
        self._unkilled_callback = unkilled_callback

        self._kills = None
        self._sub = rospy.Subscriber('/kill', KillsStamped, self._killmsg_callback)
        self._previously_killed = False

        rospy.Timer(rospy.Duration(1), self._timer_cb)

    # User function to see what is causing the kill
    def get_kills(self):
        if self._kills is not None:
            return [kill.description for kill in self._kills if kill.active]
        else:
            return []

    # Iterate through _kills to find any active kills
    #      If _kills has not been initialized (kill master has not published any kill msgs yet) return true
    def get_killed(self):
        # Check if the kill_master is publishing
        if self._sub.get_num_connections() == 0:
            # Kill master isn't publishing
            k = Kill()
            k.id = 'This kill listener'
            k.active = True
            k.description = 'Kill master is not publishing'
            self._kills = [k]
            return True

        if self._kills is not None:
            return len(self.get_kills()) > 0
        else:
            return True

    def get_all_kills(self):
        return self._kills

    def _timer_cb(self, event):
        self._check_killed()
