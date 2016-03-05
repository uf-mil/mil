from txros import util

from std_msgs.msg import Header

from kill_handling.srv import SetKill, SetKillRequest
from kill_handling.msg import Kill


class KillBroadcasterTxROS(object):
    def __init__(self, nh, id, description):
        self._nh = nh
        self.id = id
        self.description = description

        self.set_kill = nh.get_service_client('/set_kill', SetKill)

    @util.cancellableInlineCallbacks
    def send(self, active):
        try:
            yield self.set_kill(SetKillRequest(
                kill=Kill(
                    header=Header(
                        stamp=self._nh.get_time(),
                    ),
                    id=self.id,
                    active=active,
                    description=self.description,
                ),
                clear=False,
            ))
        except:
            import traceback
            traceback.print_exc()

    @util.cancellableInlineCallbacks
    def clear(self):
        yield self.set_kill(SetKillRequest(
            kill=Kill(
                header=Header(
                    stamp=self._nh.get_time(),
                ),
                id=self.id,
            ),
            clear=True,
        ))
