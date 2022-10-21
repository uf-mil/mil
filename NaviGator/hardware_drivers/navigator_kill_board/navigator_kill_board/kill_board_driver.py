#!/usr/bin/env python3
from __future__ import annotations

import copy
import threading

import numpy as np
import rospy
import serial
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mil_tools import thread_lock
from mil_usb_to_can import CANDeviceHandle, ReceivePacket
from navigator_alarm_handlers import NetworkLoss

# from navigator_kill_board import KillMessage, RequestMessage, constants
from ros_alarms import AlarmBroadcaster, AlarmListener
from sensor_msgs.msg import Joy
from std_msgs.msg import Header, String

from .constants import constants
from .packets import KillMessage, RequestMessage

lock = threading.Lock()


class KillInterface(CANDeviceHandle):
    """
    Driver to interface with NaviGator's kill handling board, which disconnects power to actuators
    if any of 4 emergency buttons is pressed, a software kill command is sent, or the network heartbeat
    stops. This driver enables the software kill option via ros_alarms and outputs diagnostics
    data about the board to ROS. The driver can handle the board's asynchronous updates of kill statuses
    and will also periodicly request updates in case the async is not working (it often doesn't).
    """

    ALARM = "hw-kill"  # Alarm to raise when hardware kill is detected
    YELLOW_WRENCHES = [
        "rc",
        "/wrench/rc",
        "keyboard",
        "/wrench/keyboard",
    ]  # Wrenches which activate YELLOW LED
    GREEN_WRENCHES = [
        "autonomous",
        "/wrench/autonomous",
    ]  # Wrenches which activate GREEN LED

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.diagnostics_pub = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=3
        )
        self.connected = True
        rospy.loginfo("Board connected!")
        self.board_status = {}
        for kill in constants["KILLS"]:
            self.board_status[kill] = False
        self.kills: list[str] = list(self.board_status.keys())
        self.network_killed = False
        self.software_killed = False
        self.expected_responses = []
        self.network_msg = None
        self.wrench = ""
        self._hw_killed = False
        self._last_hw_kill_paramaters = self.board_status
        self.last_request = None
        self.request_index = -1

        self.hw_kill_broadcaster = AlarmBroadcaster("hw-kill")
        self.hw_kill_broadcaster.wait_for_server()
        self._heartbeat_timer = rospy.Timer(rospy.Duration(0.7), self.send_heartbeat)

        self._hw_kill_listener = AlarmListener("hw-kill", self.hw_kill_alarm_cb)
        self._kill_listener = AlarmListener("kill", self.kill_alarm_cb)
        self.kill_broadcaster = AlarmBroadcaster("kill")
        self.kill_broadcaster.wait_for_server()
        self._network_kill_listener = AlarmListener(
            "network-loss", self.network_kill_alarm_cb
        )
        self._hw_kill_listener.wait_for_server()
        self._kill_listener.wait_for_server()
        self._network_kill_listener.wait_for_server()
        rospy.Subscriber("/wrench/selected", String, self.wrench_cb)
        self.network_kill = NetworkLoss()

    def run(self):
        """
        Main loop for driver, at a fixed rate updates alarms and diagnostics output with new
        kill board output.
        """
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.last_request is None:
                self.request_next()
            # self.receive()
            self.update_ros()
            rate.sleep()

    def update_ros(self):
        self.update_hw_kill()
        self.publish_diagnostics()
        if self.ctrl_msg_received is True:
            self.publish_joy()
            self.ctrl_msg_received = False

    def handle_byte(self, msg):
        """
        React to a byte received from the board. This could by an async update of a kill status or
        a known response to a recent request
        """
        print("data received")
        if self.last_request is not None:
            if msg == constants["RESPONSE_FALSE"]:
                if self.board_status[self.last_request] is True:
                    rospy.logdebug(f"SYNC FALSE for {self.last_request}")
                self.board_status[self.last_request] = False
                self.last_request = None
                return
            if msg == constants["RESPONSE_TRUE"]:
                if self.board_status[self.last_request] is False:
                    rospy.logdebug(f"SYNC TRUE for {self.last_request}")
                self.board_status[self.last_request] = True
                self.last_request = None
                return
        # If an async update was received, update internal state
        for kill in self.board_status:
            if msg == constants[kill]["FALSE"]:
                if self.board_status[kill] is True:
                    rospy.logdebug(f"ASYNC FALSE for {self.last_request}")
                self.board_status[kill] = False
                return
            if msg == constants[kill]["TRUE"]:
                if self.board_status[kill] is False:
                    rospy.logdebug(f"ASYNC TRUE FOR {kill}")
                self.board_status[kill] = True
                return
        # If a response to another request, like ping or computer kill/clear is received
        for index, byte in enumerate(self.expected_responses):
            if msg == byte:
                del self.expected_responses[index]
                return
        # GH-861: Figure out why this happens so much
        # Log a warning if an unexpected byte was received
        # rospy.logwarn(
        #    "Received an unexpected byte {}, remaining expected_responses={}".format(
        #        hex(ord(msg)), len(self.expected_responses)
        #    )
        # )

    def on_data(self, data: bytes):
        msg = ReceivePacket.from_bytes(data)
        self.handle_byte(msg)

    @thread_lock(lock)
    def receive(self):
        """
        Receive update bytes sent from the board without requests being sent, updating internal
        state, raising alarms, etc in response to board updates. Clears the in line buffer.
        """
        while not rospy.is_shutdown():
            msg = self.ser.read(1)
            self.handle_byte(msg)

    def request_next(self):
        """
        Manually request status of the next kill, looping back to the first
        once all have been responsded to.
        """
        self.request_index += 1
        if self.request_index == len(self.kills):
            self.request_index = 0
        self.last_request = self.kills[self.request_index]
        self.request(constants["REQUEST"]["KILL_STATE_REQUEST"])

    def wrench_cb(self, msg):
        """
        Updates wrench (autnomous vs teleop) diagnostic light if necessary
        on wrench changes
        """
        wrench = msg.data
        if wrench != self.wrench:
            if wrench in self.YELLOW_WRENCHES:
                self.request(
                    constants["REQUEST"]["IS_RC"],
                    constants["REQUEST"]["IS_RC"],
                )
            elif wrench in self.GREEN_WRENCHES:
                self.request(
                    constants["REQUEST"]["IS_AUTONOMOUS"],
                    constants["REQUEST"]["IS_AUTONOMOUS"],
                )
            self.wrench = wrench

    def network_kill_alarm_cb(self, alarm):
        """
        Pings kill board on every network heartbeat message. Pretends to be the rf-based heartbeat because
        real one does not work :(
        """
        if alarm.raised:
            self.network_killed = True
            if self.software_killed:
                return
            self.kill_broadcaster.raise_alarm()
            self.request(
                constants["REQUEST"]["KILL_COMMAND"], constants["REQUEST"]["CLEAR_KILL"]
            )
        else:
            self.network_killed = False
            if self.software_killed:
                return
            self.kill_broadcaster.clear_alarm()
            self.request(
                constants["REQUEST"]["KILL_COMMAND"], constants["REQUEST"]["CLEAR_KILL"]
            )
        # self.request(constants["PING"]["REQUEST"], constants["PING"]["RESPONSE"])

    def publish_diagnostics(self, err=None):
        """
        Publishes current kill board state in ROS diagnostics package, making it easy to use in GUIs and other ROS tools
        """
        msg = DiagnosticArray()
        msg.header.stamp = rospy.Time.now()
        status = DiagnosticStatus()
        status.name = "kill_board"
        status.hardware_id = self.port
        if not self.connected:
            status.level = DiagnosticStatus.ERROR
            status.message = "Cannot connect to kill board. Retrying every second."
            status.values.append(KeyValue("Exception", str(err)))
        else:
            status.level = DiagnosticStatus.OK
            for key, value in self.board_status.items():
                status.values.append(KeyValue(key, str(value)))
        msg.status.append(status)
        self.diagnostics_pub.publish(msg)

    def update_hw_kill(self):
        """
        Raise/Clear hw-kill ROS Alarm is necessary (any kills on board are engaged)
        """
        killed = self.board_status["HW_KILL"]
        if (killed and not self._hw_killed) or (
            killed and self.board_status != self._last_hw_kill_paramaters
        ):
            self._hw_killed = True
            self.hw_kill_broadcaster.raise_alarm(parameters=self.board_status)
            self._last_hw_kill_paramaters = copy.copy(self.board_status)
        if not killed and self._hw_killed:
            self._hw_killed = False
            self.hw_kill_broadcaster.clear_alarm()

    @thread_lock(lock)
    def request(self, write_str, expected_response=None):
        """
        Deals with requesting data and checking if the response matches some `recv_str`.
        Returns True or False depending on the response.
        With no `recv_str` passed in the raw result will be returned.
        """
        bytes = None
        if write_str == constants["REQUEST"]["KILL_STATE_REQUEST"]:
            bytes = RequestMessage.create_message().to_bytes()
        else:
            bytes = KillMessage.create_message(write_str).to_bytes()
        self.send_data(bytes, can_id=int.from_bytes(constants["CAN_ID"], "big"))
        if expected_response is not None:
            for byte in expected_response:
                self.expected_responses.append(byte)

    def kill_alarm_cb(self, alarm):
        """
        Informs kill board about software kills through ROS Alarms
        """
        if alarm.raised:
            self.software_killed = True
            if self.network_killed:
                return
            self.request(
                constants["REQUEST"]["KILL_COMMAND"],
                constants["REQUEST"]["KILL_COMMAND"],
            )
        else:
            self.software_killed = False
            if self.network_killed:
                self.kill_broadcaster.raise_alarm()
                return
            self.request(
                constants["REQUEST"]["CLEAR_KILL"], constants["REQUEST"]["CLEAR_KILL"]
            )

    def hw_kill_alarm_cb(self, alarm):
        self._hw_killed = alarm.raised

    def send_heartbeat(self, _: TimerEvent) -> None:
        """
        Send a special heartbeat packet. Called by a recurring timer set upon
        initialization.
        """
        # self.send_data(constants["REQUEST"]["HEARTBEAT"], can_id=b"\x12")
        self.request(constants["REQUEST"]["HEARTBEAT"])


if __name__ == "__main__":
    rospy.init_node("kill_board_driver")
    driver = KillInterface()
    driver.run()
