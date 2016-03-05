from txros import variable

from std_msgs.msg import Header
from kill_handling.msg import Kill, KillsStamped


def kills_to_killed(kills):
    return len(kills) > 0


class KillListenerTxROS(object):
    def __init__(self, nh):
        self.kills_variable = variable.Variable([
            Kill(header=Header(stamp=nh.get_time()), id='null', active=True,
                 description='default kill until a kill update message is received')
        ])

        self.killed_variable = variable.Variable(kills_to_killed(self.kills_variable.value))
        self.kills_variable.changed.watch(lambda kills: self.killed_variable.set(kills_to_killed(kills)))

        def _killmsg_callback(msg):
            self.kills_variable.set([k for k in msg.kills if k.active])
        self._sub = nh.subscribe('/kill', KillsStamped, _killmsg_callback)
