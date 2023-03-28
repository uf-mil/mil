from __future__ import annotations

import json
import traceback
from typing import Callable

import rospy
from ros_alarms_msg.msg import Alarm as AlarmMsg
from ros_alarms_msg.srv import (
    AlarmGet,
    AlarmGetRequest,
    AlarmGetResponse,
    AlarmSet,
    AlarmSetRequest,
)


def parse_json_str(json_str: str) -> dict | str:
    parameters = ""
    try:
        parameters = "" if json_str == "" else json.loads(json_str)
    except ValueError:
        # User passed in a non JSON string
        parameters = {}
        parameters["data"] = json_str
    finally:
        return parameters


def _check_for_alarm(alarm_name, nowarn=False):
    if (
        not nowarn
        and rospy.has_param("/known_alarms")
        and alarm_name not in rospy.get_param("/known_alarms")
    ):
        msg = "'{}' is not in the list of known alarms (as defined in the /known_alarms rosparam)"
        rospy.logwarn(msg.format(alarm_name))


def _check_for_valid_name(alarm_name, nowarn=False):
    if nowarn:
        return

    assert (
        alarm_name.isalnum() or "_" in alarm_name or "-" in alarm_name
    ), f"Alarm name '{alarm_name}' is not valid!"


def _make_callback_error_string(alarm_name, backtrace=""):
    """Creates a string with the name of the alarm, the stacktrace, and an exception"""
    err_msg = "A callback for the alarm: {} threw an error!\n{}"
    return err_msg.format(alarm_name, backtrace)


def wait_for_service(
    service,
    warn_time=1.0,
    warn_msg="Waiting for service..",
    timeout=None,
):
    """
    A fancy extension of wait for service that will warn with a message if it is taking a while.

    @param warn_time: float in seconds, how long to wait before logging warn_msg
    @param warn_msg: msg logged with rospy.logwarn if warn_time passes without service connected
    @param timeout: overall timeout. If None, does nothing. If a float, will raise exception
                    if many TOTAL seconds has passed without connecting

    Copied from https://github.com/uf-mil/mil_common/blob/master/utils/mil_tools/mil_ros_tools/init_helpers.py
    to avoid dependency
    """
    try:
        service.wait_for_service(warn_time)
    except rospy.ROSException:
        if timeout is not None:
            timeout = timeout - warn_time
        rospy.logwarn(warn_msg)
        service.wait_for_service(timeout)


class Alarm:
    """
    Pythonic representation of a ROS alarm.

    This class should not be constructed by clients (excluding internal ``ros_alarms``
    code, of course). This class is primarily used by the alarm server node to
    manage data about a series of alarms. However, this class may be passed
    into callbacks registered to particular alarms.

    .. container:: operations

        .. describe:: str(x)

            Pretty prints the contents of the alarm. Equivalent to ``repr(x)``.

        .. describe:: repr(x)

            Pretty prints the contents of the alarm. Equivalent to ``str(x)``.

    Attributes:
        alarm_name (str): The name of this specific alarm.
        raised (bool): Whether the alarm has been raised.
        node_name (str): The name of the node attached to the alarm. Defaults to
            ``unknown``.
        problem_description (str): A description of the problem related to the alarm.
            Defaults to an empty string.
        parameters (dict): The JSON parameters associated with the alarm. Defaults
            to an empty dict.
        severity (int): The severity associated with a raised alarm. Defaults to ``0``.
        stamp (rospy.Time): The time of the most recent update. Defaults to now if
            ROS has been initialized, otherwise zero.
        raised_cbs (List[Tuple[int, Callable]): A list of callbacks to execute
            when the alarm is raised. Each callback should be associated with
            the severity required to execute the callback through a tuple.
        cleared_cbs (List[Tuple[int, Callable]): A list of callbacks to execute
            when the alarm is cleared. Each callback should be associated with
            the severity required to execute the callback through a tuple.
    """

    def __init__(
        self,
        alarm_name: str,
        raised: bool,
        node_name: str = "unknown",
        problem_description: str = "",
        parameters: dict = {},
        severity: int = 0,
    ):
        self.alarm_name = alarm_name
        self.raised = raised
        self.node_name = node_name
        self.problem_description = problem_description
        self.parameters = parameters
        self.severity = severity

        try:
            self.stamp = rospy.Time.now()
        except rospy.ROSInitException:
            self.stamp = rospy.Time(0)

        # Callbacks to run if the alarm is cleared or raised formatted as follows:
        #   [(severity_required, cb1), (severity_required, cb2), ...]
        self.raised_cbs = []
        self.cleared_cbs = []

    def __repr__(self):
        msg = self.as_msg()
        msg.parameters = parse_json_str(msg.parameters)
        return str(msg)

    __str__ = __repr__

    @classmethod
    def blank(cls, name: str) -> Alarm:
        """
        Generate a general blank alarm that is cleared with a low severity.

        Args:
            name (str): The name for the blank alarm.

        Returns:
            ros_alarms.Alarm: The constructed alarm.
        """
        return cls(name, raised=False, severity=0)

    @classmethod
    def from_msg(cls, msg: AlarmMsg) -> Alarm:
        """
        Generate an alarm object from an Alarm message.

        Args:
            msg (AlarmMsg): The message to generate the object from.

        Returns:
            Alarm: The constructed alarm.
        """
        node_name = "unknown" if msg.node_name == "" else msg.node_name
        parameters = parse_json_str(msg.parameters)

        return cls(
            msg.alarm_name,
            msg.raised,
            node_name,
            msg.problem_description,
            parameters,
            msg.severity,
        )

    def _severity_cb_check(self, severity):
        if isinstance(severity, (tuple, list)):
            return severity[0] <= self.severity <= severity[1]

        # Not a tuple, just an int. The severities should match
        return self.severity == severity

    def add_callback(
        self,
        funct: Callable,
        call_when_raised: bool = True,
        call_when_cleared: bool = True,
        severity_required: tuple[int, int] = (0, 5),
    ):
        """
        Adds a callback function to the alarm.

        If the callback is set to be called when the alarm has been raised, and
        the alarm is already raised, then the callback is immediately called.
        Similarly, if the callback is set to run when the alarm is cleared, and
        the alarm is already cleared, then the callback is immediately ran. In
        both cases, the callback is registered like any other callback.

        Exceptions in callbacks are swallowed and are printed out through a
        rospy logging statement.

        Args:
            funct (Callable): The callback to add.
            call_when_raised (bool): Whether to call the callback when the alarm
                is raised. Defaults to ``True``.
            call_when_cleared (bool): Whether to call the callback when the alarm
                is cleared. Defaults to ``True``.
            severity_required (Tuple[int, int]): The severity required to run the
                callback. The tuple represents the minimum and maximum severities
                under which to execute the callback. Defaults to ``(0, 5)``.
        """
        if call_when_raised:
            self.raised_cbs.append((severity_required, funct))
            if self.raised and self._severity_cb_check(severity_required):
                # Try to run the callback, absorbing any errors
                try:
                    funct(self)
                except Exception:
                    rospy.logwarn(
                        _make_callback_error_string(
                            self.alarm_name,
                            traceback.format_exc(),
                        ),
                    )

        if call_when_cleared:
            self.cleared_cbs.append(((0, 5), funct))
            if not self.raised and self._severity_cb_check(severity_required):
                # Try to run the callback, absorbing any errors
                try:
                    funct(self)
                except Exception:
                    rospy.logwarn(
                        _make_callback_error_string(
                            self.alarm_name,
                            traceback.format_exc(),
                        ),
                    )

    def update(self, srv: Alarm):
        """
        Updates this alarm with a new AlarmSet request. Also will call any
        required callbacks.

        Args:
            srv (ros_alarms.Alarm): The request to set the alarm.
        """
        self.stamp = rospy.Time.now()

        node_name = "unknown" if srv.node_name == "" else srv.node_name
        parameters = parse_json_str(srv.parameters)

        # Update all possible parameters
        self.raised = srv.raised
        self.node_name = node_name
        self.problem_description = srv.problem_description
        self.parameters = parameters
        self.severity = srv.severity

        rospy.loginfo(
            "Updating alarm: {}, {}.".format(
                self.alarm_name,
                "raised" if self.raised else "cleared",
            ),
        )
        # Run the callbacks for that alarm
        cb_list = self.raised_cbs if srv.raised else self.cleared_cbs
        for severity, cb in cb_list:
            # If the cb severity is not valid for this alarm's severity, skip it
            if srv.raised and not self._severity_cb_check(severity):
                continue

            # Try to run the callback, absorbing any errors
            try:
                cb(self)
            except Exception:
                rospy.logwarn(
                    _make_callback_error_string(
                        self.alarm_name,
                        traceback.format_exc(),
                    ),
                )

    def as_msg(self) -> AlarmMsg:
        """
        Get this alarm as an Alarm message.

        Returns:
            AlarmMsg: The constructed message.
        """
        alarm = AlarmMsg()
        alarm.alarm_name = self.alarm_name
        alarm.raised = self.raised
        alarm.node_name = self.node_name
        alarm.problem_description = self.problem_description
        alarm.parameters = json.dumps(self.parameters)
        alarm.severity = self.severity
        return alarm

    def as_srv_resp(self) -> AlarmGetResponse:
        """
        Get this alarm as an AlarmGet response.

        Returns:
            AlarmGetResponse: The constructed service response.
        """
        resp = AlarmGetResponse()
        resp.header.stamp = self.stamp
        resp.alarm = self.as_msg()
        return resp


class AlarmBroadcaster:
    """
    Broadcasts information about an alarm, and allows for the raising and clearing
    of the alarm.
    """

    def __init__(self, name: str, node_name: str | None = None, nowarn: bool = False):
        """
        Args:
            name (str): The name of the related alarm.
            node_name (Optional[str]): The name of the node alarm.
            nowarn (bool): Whether to disable warning for the class' operations.
        """
        self._alarm_name = name
        _check_for_valid_name(self._alarm_name, nowarn)
        _check_for_alarm(self._alarm_name, nowarn)

        self._node_name = rospy.get_name() if node_name is None else node_name

        self._alarm_set = rospy.ServiceProxy("/alarm/set", AlarmSet)

    def wait_for_server(self, timeout=None):
        """
        Wait for node to connect to alarm server. Waits timeout seconds (or forever
            if ``timeout`` is ``None``) to connect then fetches the current alarm
            and calls callbacks as needed.

        .. warning::

            User should always call this method before calling other methods.

        Args:
            timeout (Optional[float]): The amount of seconds to wait before timing
                out.
        """
        if timeout is not None and timeout < 1.5:
            self._alarm_set.wait_for_service(timeout=timeout)
        else:
            wait_for_service(
                self._alarm_set,
                warn_time=1.0,
                warn_msg="Waiting for alarm server..",
                timeout=timeout,
            )
        rospy.logdebug("alarm server connected")

    def _generate_request(
        self,
        raised,
        node_name=None,
        problem_description="",
        parameters={},
        severity=0,
    ):
        request = AlarmSetRequest()
        request.alarm.alarm_name = self._alarm_name

        request.alarm.node_name = (
            node_name if node_name is not None else self._node_name
        )
        request.alarm.raised = raised
        request.alarm.problem_description = problem_description
        request.alarm.parameters = json.dumps(parameters)
        request.alarm.severity = severity

        return request

    def raise_alarm(self, **kwargs):
        """
        Raises the alarm.

        Args:
            kwargs: The associated keyword arguments, as described below.

        Keyword Arguments:
            node_name (str): String that holds the name of the node making the
                request.
            problem_description (str): String with a description of the problem
                (defaults to empty string).
            parameters (dict): JSON dumpable dictionary with optional parameters
                that describe the alarm.
            severity (int): Integer in [0, 5] that indicates the severity of
                the alarm. (5 is most severe)

        Returns:
            bool: Whether the alarm was successfully raised.
        """
        try:
            return self._alarm_set(self._generate_request(True, **kwargs)).succeed
        except rospy.service.ServiceException:
            rospy.logerr("No alarm sever found! Can't raise alarm.")
            return False

    def clear_alarm(self, **kwargs):
        """
        Clears the alarm.

        Args:
            kwargs: The associated keyword arguments, as described below.

        Keyword Arguments:
            node_name (str): String that holds the name of the node making the
                request.
            problem_description (str): String with a description of the problem
                (defaults to empty string).
            parameters (dict): JSON dumpable dictionary with optional parameters
                that describe the alarm.
            severity (int): Integer in [0, 5] that indicates the severity of
                the alarm. (5 is most severe)

        Returns:
            bool: Whether the alarm was successfully cleared.
        """
        try:
            return self._alarm_set(self._generate_request(False, **kwargs)).succeed
        except rospy.service.ServiceException:
            rospy.logerr("No alarm sever found! Can't clear alarm.")
            return False


class AlarmListener:
    """
    Listens to an alarm.
    """

    def __init__(
        self,
        name: str,
        callback_funct: Callable | None = None,
        nowarn: bool = False,
        **kwargs,
    ):
        """
        Args:
            name (str): The alarm name.
            callback_funct (Optional[Callable]): The callback function.
            nowarn (bool): Whether to enable warnings about the class. Defaults to ``False``.
        """
        self._alarm_name = name
        self._last_alarm = None
        _check_for_valid_name(self._alarm_name, nowarn)
        _check_for_alarm(self._alarm_name, nowarn)

        self._alarm_get = rospy.ServiceProxy("/alarm/get", AlarmGet)

        # Data used to trigger callbacks
        self._raised_cbs = []  # [(severity_for_cb1, cb1), (severity_for_cb2, cb2), ...]
        self._cleared_cbs = []
        rospy.Subscriber("/alarm/updates", AlarmMsg, self._alarm_update)

        if callback_funct is not None:
            self.add_callback(callback_funct, **kwargs)

    def wait_for_server(self, timeout: float | None = None):
        """
        Wait for node to connect to alarm server. Waits timeout seconds (or forever
            if ``timeout`` is ``None``) to connect then fetches the current alarm
            and calls callbacks as needed.

        .. warning::

            User should always call this method before calling other methods.

        Args:
            timeout (Optional[float]): The amount of seconds to wait before timing
                out.
        """
        if timeout is not None and timeout < 1.5:
            self._alarm_get.wait_for_service(timeout=timeout)
        else:
            wait_for_service(
                self._alarm_get,
                warn_time=1.0,
                warn_msg="Waiting for alarm server..",
                timeout=timeout,
            )
        rospy.logdebug("alarm server connected")
        self.get_alarm()  # Now that we have service, update callbacks

    def is_raised(self, fetch: bool = True):
        """
        Returns whether this alarm is raised or not.

        Args:
            fetch (bool): Whether to fetch the alarm from the server. If ``False``,
                then uses the most recently cached alarm. Defaults to ``True``.

        Returns:
            bool: Whether the alarm is raised.
        """
        alarm = self.get_alarm(fetch=fetch)
        if alarm is None:
            return False
        return alarm.raised

    def is_cleared(self, fetch: bool = True) -> bool:
        """
        Returns whether this alarm is cleared or not.

        Args:
            fetch (bool): Whether to fetch the alarm from the server. If ``False``,
                then uses the most recently cached alarm. Defaults to ``True``.

        Returns:
            bool: Whether the alarm is cleared.
        """
        return not self.is_raised(fetch=fetch)

    def get_alarm(self, fetch: bool = True) -> AlarmGetResponse | None:
        """
        Returns the alarm message.

        Also worth noting, the alarm this method returns has it's ``parameters`` field
            converted to a dictionary.

        Args:
            fetch (bool): Whether to fetch the alarm from the server. If ``False``,
                then uses the most recently cached alarm. Defaults to ``True``.

        Returns:
            AlarmGetResponse: The response message, with its updated ``parameters`` field.
        """
        if not fetch:
            return self._last_alarm
        try:
            resp = self._alarm_get(AlarmGetRequest(alarm_name=self._alarm_name))
        except rospy.service.ServiceException:
            rospy.logerr("No alarm sever found!")
            return self._last_alarm
        self._alarm_update(resp.alarm)
        return self._last_alarm

    def _severity_cb_check(self, severity):
        # In case _last alarm hasn't been declared yet
        if self._last_alarm is None:
            return False

        if isinstance(severity, (tuple, list)):
            return severity[0] <= self._last_alarm.severity <= severity[1]

        # Not a tuple or list, just an int. The severities should match
        return self._last_alarm.severity == severity

    def add_callback(
        self,
        funct: Callable,
        call_when_raised: bool = True,
        call_when_cleared: bool = True,
        severity_required: tuple[int, int] = (0, 5),
    ):
        """
        Adds a callback function to the alarm.

        Args:
            funct (Callable): The callback to add.
            call_when_raised (bool): Whether to call the callback when the alarm
                is raised. Defaults to ``True``.
            call_when_cleared (bool): Whether to call the callback when the alarm
                is cleared. Defaults to ``True``.
            severity_required (Tuple[int, int]): The severity required to run the
                callback. The tuple represents the minimum and maximum severities
                under which to execute the callback. Defaults to ``(0, 5)``.
        """
        alarm = self._last_alarm
        if call_when_raised:
            self._raised_cbs.append((severity_required, funct))
            if (
                alarm is not None
                and alarm.raised
                and self._severity_cb_check(severity_required)
            ):
                # Try to run the callback, absorbing any errors
                try:
                    alarm.parameters = parse_json_str(alarm.parameters)
                    funct(alarm)
                except Exception:
                    rospy.logwarn(
                        _make_callback_error_string(
                            self._alarm_name,
                            traceback.format_exc(),
                        ),
                    )

        if call_when_cleared:
            self._cleared_cbs.append(((0, 5), funct))  # Clear callbacks always run
            if alarm is not None and not alarm.raised:
                # Try to run the callback, absorbing any errors
                try:
                    alarm.parameters = parse_json_str(alarm.parameters)
                    funct(alarm)
                except Exception:
                    rospy.logwarn(
                        _make_callback_error_string(
                            self._alarm_name,
                            traceback.format_exc(),
                        ),
                    )

    def clear_callbacks(self):
        """
        Clears all callbacks.
        """
        self._raised_cbs = []
        self._cleared_cbs = []

    def _alarm_update(self, alarm: AlarmMsg):
        if alarm.alarm_name != self._alarm_name:
            return
        alarm.parameters = parse_json_str(alarm.parameters)
        self._last_alarm = alarm
        cb_list = self._raised_cbs if alarm.raised else self._cleared_cbs
        for severity, cb in cb_list:
            # If the cb severity is not valid for this alarm's severity, skip it
            if not self._severity_cb_check(severity):
                continue

            # Try to run the callback, absorbing any errors
            try:
                cb(alarm)
            except Exception:
                rospy.logerr(
                    _make_callback_error_string(
                        self._alarm_name,
                        traceback.format_exc(),
                    ),
                )


class HeartbeatMonitor(AlarmBroadcaster):
    """
    Used to trigger an alarm if a message on the topic ``topic_name`` isn't published
    at least every ``prd`` seconds.

    An alarm won't be triggered if no messages are initially received.

    All of this class' methods and attributes are internal.
    """

    def __init__(
        self,
        alarm_name: str,
        topic_name: str,
        msg_class,
        prd: float = 0.2,
        predicate: Callable | None = None,
        nowarn: bool = False,
        **kwargs,
    ):
        self._predicate = predicate if predicate is not None else lambda *args: True
        self._last_msg_time = None
        self._prd = rospy.Duration(prd)
        self._dropped = False

        super().__init__(alarm_name, nowarn=nowarn, **kwargs)
        rospy.Subscriber(topic_name, msg_class, self._got_msg)

        rospy.Timer(rospy.Duration(prd / 2), self._check_for_message)

    def _got_msg(self, msg):
        # If the predicate passes, store the message time
        if self._predicate(msg):
            self._last_msg_time = rospy.Time.now()

            # If it's dropped, clear the alarm and reset the dropped status
            if self._dropped:
                self.clear_alarm()
                self._dropped = False

    def _check_for_message(self, *args):
        if self._last_msg_time is None:
            return

        if rospy.Time.now() - self._last_msg_time > self._prd and not self._dropped:
            self.raise_alarm()
            self._dropped = True
