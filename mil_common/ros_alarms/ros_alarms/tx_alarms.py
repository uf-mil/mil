import asyncio
import json
import traceback

import axros
from ros_alarms.msg import Alarm
from ros_alarms.srv import AlarmGet, AlarmGetRequest, AlarmSet, AlarmSetRequest

"""
Alarms implementation for axros (https://github.com/axros/axros)
"""


async def _check_for_alarm(nh: axros.NodeHandle, alarm_name: str, nowarn=False):
    if (
        not nowarn
        and (await nh.has_param("/known_alarms"))
        and alarm_name not in (await nh.get_param("/known_alarms"))
    ):
        msg = "'{}' is not in the list of known alarms (as defined in the /known_alarms rosparam)"
        print(msg.format(alarm_name))


def _check_for_valid_name(alarm_name, nowarn=False):
    if nowarn:
        return

    assert (
        alarm_name.isalnum() or "_" in alarm_name or "-" in alarm_name
    ), f"Alarm name '{alarm_name}' is not valid!"


class TxAlarmBroadcaster:
    @classmethod
    async def init(cls, nh: axros.NodeHandle, name: str, node_name=None, nowarn=False):
        _check_for_valid_name(name, nowarn)
        await _check_for_alarm(nh, name, nowarn)

        node_name = nh.get_name() if node_name is None else node_name

        return cls(nh, name, node_name)

    def __init__(self, nh, name, node_name):
        """Don't invoke this function directly, use the `init` function above"""
        self._nh = nh
        self._alarm_name = name

        self._node_name = node_name
        self._alarm_set = self._nh.get_service_client("/alarm/set", AlarmSet)

        print(f"Created alarm broadcaster for alarm {name}")

    def _generate_request(
        self, raised, problem_description="", parameters={}, severity=0
    ):
        request = AlarmSetRequest()
        request.alarm.alarm_name = self._alarm_name
        request.alarm.node_name = self._node_name

        request.alarm.raised = raised
        request.alarm.problem_description = problem_description
        request.alarm.parameters = json.dumps(parameters)
        request.alarm.severity = severity

        return request

    def raise_alarm(self, **kwargs):
        """Raises this alarm"""
        return self._alarm_set(self._generate_request(True, **kwargs))

    def clear_alarm(self, **kwargs):
        """Clears this alarm"""
        return self._alarm_set(self._generate_request(False, **kwargs))


class TxAlarmListener:
    @classmethod
    async def init(cls, nh, name, callback_funct=None, nowarn=False, **kwargs):
        _check_for_valid_name(name, nowarn)
        await _check_for_alarm(nh, name, nowarn)

        alarm_client = nh.get_service_client("/alarm/get", AlarmGet)
        try:
            await axros.util.wrap_timeout(alarm_client.wait_for_service(), 1)
        except asyncio.TimeoutError:
            print("No alarm sever found! Alarm behaviours will be unpredictable.")

        return cls(nh, name, alarm_client, callback_funct)

    def __init__(self, nh, name, alarm_client, callback_funct, **kwargs):
        """Don't invoke this function directly, use the `init` function above"""
        self._nh = nh
        self._alarm_name = name
        self._alarm_get = alarm_client
        self._last_alarm = None

        # Data used to trigger callbacks
        self._raised_cbs = []  # [(severity_for_cb1, cb1), (severity_for_cb2, cb2), ...]
        self._cleared_cbs = []
        self.update_sub = self._nh.subscribe(
            "/alarm/updates", Alarm, self._alarm_update
        )

        if callback_funct is not None:
            self.add_callback(callback_funct, **kwargs)

    async def setup(self):
        await self.update_sub.setup()

    async def shutdown(self):
        await self.update_sub.shutdown()

    async def is_raised(self):
        """Returns whether this alarm is raised or not"""
        resp = await self._alarm_get(AlarmGetRequest(alarm_name=self._alarm_name))
        return resp.alarm.raised

    async def is_cleared(self):
        """Returns whether this alarm is raised or not"""
        resp = await self._alarm_get(AlarmGetRequest(alarm_name=self._alarm_name))
        return not resp.alarm.raised

    async def get_alarm(self):
        """Returns the alarm message
        Also worth noting, the alarm this returns has it's `parameter` field
            converted to a dictionary
        """
        resp = await self._alarm_get(AlarmGetRequest(alarm_name=self._alarm_name))

        params = resp.alarm.parameters
        resp.alarm.parameters = params if params == "" else json.loads(params)
        return resp.alarm

    def _severity_cb_check(self, severity):
        if isinstance(severity, tuple) or isinstance(severity, list):
            return severity[0] <= self._last_alarm.severity <= severity[1]

        # Not a tuple, just an int. The severities should match
        return self._last_alarm.severity == severity

    def add_callback(
        self,
        funct,
        call_when_raised=True,
        call_when_cleared=True,
        severity_required=(0, 5),
    ):
        """Deals with adding function callbacks
        The user can specify if the function should be run on a raise or clear of this alarm.

        Each callback can have a severity level associated with it such that different callbacks can
            be triggered for different levels of severity.
        """

        if call_when_raised:
            self._raised_cbs.append((severity_required, funct))

            if (
                self._last_alarm is not None
                and self._last_alarm.raised
                and self._severity_cb_check(severity_required)
            ):
                funct(self._nh, self._last_alarm)

        if call_when_cleared:
            self._cleared_cbs.append(((0, 5), funct))  # Clear callbacks always run

            if self._last_alarm is not None and not self._last_alarm.raised:
                funct(self._nh, self._last_alarm)

    def clear_callbacks(self):
        """Clears all callbacks"""
        self._raised_cbs = []
        self._cleared_cbs = []

    def _alarm_update(self, alarm):
        if alarm.alarm_name == self._alarm_name:
            self._last_alarm = alarm

            # Run the callbacks if severity conditions are met
            cb_list = self._raised_cbs if alarm.raised else self._cleared_cbs
            for severity, cb in cb_list:
                # If the cb severity is not valid for this alarm's severity, skip it
                if not self._severity_cb_check(severity):
                    continue

                # Try to run the callback, absorbing any errors
                try:
                    try:
                        alarm.parameters = json.loads(alarm.parameters)
                    except Exception:
                        pass

                    cb(self._nh, alarm)
                except Exception as e:
                    err_msg = "A callback function for the alarm: {} threw an error!\n{}\nException:{}"
                    print(err_msg.format(self._alarm_name, traceback.format_exc(), e))


class TxHeartbeatMonitor(TxAlarmBroadcaster):
    @classmethod
    async def init(
        cls,
        nh: axros.NodeHandle,
        alarm_name: str,
        topic_name: str,
        msg_class,
        prd=0.2,
        predicate=None,
        **kwargs,
    ):
        """Used to trigger an alarm if a message on the topic `topic_name` isn't published
            at least every `prd` seconds. This alarm is self clearing.

        An alarm won't be triggered if no messages are initially received
        """
        ab = await TxAlarmBroadcaster.init(nh, alarm_name, **kwargs)
        predicate = predicate if predicate is not None else lambda *args: True
        prd = axros.util.genpy.Duration(prd)
        obj = cls(nh, ab, topic_name, msg_class, predicate, prd)
        await obj.sub.setup()
        return obj

    def __init__(self, nh: axros.NodeHandle, ab, topic_name, msg_class, predicate, prd):
        """Don't invoke this function directly, use the `init` function above"""
        self._nh = nh
        self._predicate = predicate
        self._prd = prd
        self._ab = ab

        self._last_msg_time = None
        self._dropped = False

        self.sub = self._nh.subscribe(topic_name, msg_class)

    def set_predicate(self, funct):
        self._predicate = funct

    async def start_monitor(self, sample_prd: int = -1):
        """Starts monitoring the topic
        This separate function allows you to start and stop the heartbeat monitor
        """
        if sample_prd == -1:
            sample_prd = self._prd.to_sec() / 2

        # Wait for that first message
        await self.sub.get_next_message()

        while True:
            await self._nh.sleep(sample_prd)

            last_time = self.sub.get_last_message_time()
            last_msg = self.sub.get_last_message()
            if (self._nh.get_time() - last_time > self._prd) and (
                await self._predicate(self._nh, last_msg)
            ):
                if not self._dropped:
                    await self._ab.raise_alarm()
                    self._dropped = True

            elif self._dropped:
                await self._ab.clear_alarm()
                self._dropped = False
