#!/usr/bin/env python
import numpy as np
import json
import rospy
import rospkg
import scipy.interpolate
import threading
import argparse
from std_msgs.msg import Header, Float64
from sub8_msgs.msg import Thrust, ThrusterStatus
from mil_ros_tools import wait_for_param, thread_lock
from sub8_msgs.srv import ThrusterInfo, ThrusterInfoResponse, FailThruster, FailThrusterResponse
from sub8_thruster_comm import thruster_comm_factory
from ros_alarms import AlarmBroadcaster, AlarmListener
lock = threading.Lock()


class BusVoltageEstimator(object):

    '''
    Class that estimates sub8's thruster bus voltage.
    As of May 2017, this is just a simple rolling average with a constant width sliding
    window. However add_reading and get_estimate methods are left for when smarter
    filtering is needed
    '''
    class VoltageReading(object):

        def __init__(self, voltage, time):
            self.v = voltage
            self.t = time

    def __init__(self, window_duration):
        '''
        window_duration - float (amount of seconds for which to keep a reading in the buffer)
        '''
        self.window_duration = rospy.Duration(window_duration)
        self.last_update_time = None
        self.buffer = []

    def add_reading(self, voltage, time):
        ''' Adds voltage readings to buffer '''
        self.buffer.append(self.VoltageReading(voltage, time))
        self.last_update_time = time
        self.prune_buffer()

    def prune_buffer(self):
        ''' Removes readings older than the window_duration from buffer '''
        for reading in self.buffer:
            age = rospy.Time.now() - reading.t
            if age > self.window_duration:
                self.buffer.remove(reading)

    def get_voltage_estimate(self):
        ''' Returns average voltage in buffer '''
        voltages = []
        for r in self.buffer:
            voltages.append(r.v)
        if not self.buffer:
            return None
        else:
            return np.mean(voltages)

    def get_last_update_time(self):
        ''' Returns time of most recent reading in buffer '''
        if self.last_update_time is None:
            self.last_update_time = rospy.Time.now()
        return self.last_update_time


class ThrusterDriver(object):
    _dropped_timeout = 1.0  # s
    _window_duration = 30.0  # s

    def __init__(self, config_path, ports, thruster_definitions):
        '''Thruster driver, an object for commanding all of the sub's thrusters
            - Gather configuration data and make it available to other nodes
            - Instantiate ThrusterPorts, (Either simulated or real), for communicating with thrusters
            - Track a thrust_dict, which maps thruster names to the appropriate port
            - Given a command message, route that command to the appropriate port/thruster
            - Send a thruster status message describing the status of the particular thruster
        '''
        self.thruster_heartbeats = {}
        self.failed_thrusters = []

        # Bus voltage
        self.bus_voltage_estimator = BusVoltageEstimator(self._window_duration)
        self.bus_voltage_pub = rospy.Publisher('bus_voltage', Float64, queue_size=1)
        self.bus_timer = rospy.Timer(rospy.Duration(0.1), self.publish_bus_voltage)
        self.warn_voltage = rospy.get_param("/battery/warn_voltage", 44.5)
        self.kill_voltage = rospy.get_param("/battery/kill_voltage", 44.0)
        self.bus_voltage_alarm = AlarmBroadcaster("bus-voltage")

        self.make_fake = rospy.get_param('simulate', False)
        if self.make_fake:
            rospy.logwarn("Running fake thrusters for simulation, based on parameter '/simulate'")

        # Individual thruster configuration data
        newtons, thruster_input = self.load_effort_to_thrust_map(config_path)
        self.interpolate = scipy.interpolate.interp1d(newtons, thruster_input)

        self.thrust_service = rospy.Service('thrusters/thruster_range', ThrusterInfo, self.get_thruster_info)
        self.status_pub = rospy.Publisher('thrusters/thruster_status', ThrusterStatus, queue_size=8)

        # Port and thruster layout
        self.thruster_out_alarm = AlarmBroadcaster("thruster-out")
        AlarmListener("thruster-out", self.check_alarm_status, call_when_raised=False)
        self.port_dict = self.load_thruster_layout(ports, thruster_definitions)
        self.drop_check = rospy.Timer(rospy.Duration(0.5), self.check_for_drops)

        # The bread and bones
        self.thrust_sub = rospy.Subscriber('thrusters/thrust', Thrust, self.thrust_cb, queue_size=1)

        # This is essentially only for testing
        self.fail_thruster_server = rospy.Service('fail_thruster', FailThruster, self.fail_thruster)

    def load_effort_to_thrust_map(self, path):
        '''Load the effort to thrust mapping:
            - Map force inputs from Newtons to [-1, 1] required by the thruster
        '''
        try:
            _file = file(path)
        except IOError as e:
            rospy.logerr("Could not find thruster configuration file at {}".format(path))
            raise(e)

        json_data = json.load(_file)
        newtons = json_data['calibration_data']['newtons']
        thruster_input = json_data['calibration_data']['thruster_input']
        return newtons, thruster_input

    @thread_lock(lock)
    def load_thruster_layout(self, ports, thruster_definitions):
        '''Load and handle the thruster layout'''
        port_dict = {}

        # These alarms require this service to be available before things will work
        rospy.wait_for_service("update_thruster_layout")
        self.thruster_out_alarm.clear_alarm(parameters={'clear_all': True})

        for port_info in ports:
            thruster_port = thruster_comm_factory(port_info, thruster_definitions, fake=self.make_fake)

            # Add the thrusters to the thruster dict
            for thruster_name in port_info['thruster_names']:
                if thruster_name in thruster_port.missing_thrusters:
                    rospy.logerr("{} IS MISSING!".format(thruster_name))
                    self.alert_thruster_loss(thruster_name, "Motor ID was not found on it's port.")
                else:
                    rospy.loginfo("{} registered".format(thruster_name))

                self.thruster_heartbeats[thruster_name] = None
                port_dict[thruster_name] = thruster_port

        return port_dict

    def get_thruster_info(self, srv):
        '''
        Get the thruster info for a particular thruster ID
        Right now, this is only the min and max thrust data
        '''
        # Unused right now
        # query_id = srv.thruster_id

        min_thrust = min(self.interpolate.x)
        max_thrust = max(self.interpolate.x)
        thruster_info = ThrusterInfoResponse(
            min_force=min_thrust,
            max_force=max_thrust
        )
        return thruster_info

    def publish_bus_voltage(self, *args):
        ''' Publishes bus voltage estimate and raises bus_voltage alarm if needed '''
        since_voltage = rospy.Time.now() - self.bus_voltage_estimator.get_last_update_time()
        if (since_voltage) > rospy.Duration(0.5):
            self.stop()  # for safety

        bus_voltage = self.bus_voltage_estimator.get_voltage_estimate()
        if bus_voltage is not None:
            msg = Float64(bus_voltage)
            self.bus_voltage_pub.publish(msg)
            self.check_bus_voltage(bus_voltage)  # also checks the severity of the bus voltage

    def check_bus_voltage(self, voltage):
        ''' Raises bus_voltage alarm with a corresponding severity given a bus voltage '''
        # Timing bug occurs here occasionally so I (David) wil disable enforcing bus_voltage kills
        # until I submit a PR fixing the issue
        return

        severity = None
        if voltage < self.warn_voltage:
            severity = 3
        if voltage < self.kill_voltage:
            severity = 5

        if severity is not None:
            self.bus_voltage_alarm.raise_alarm(
                problem_description='Bus voltage has fallen to {}'.format(voltage),
                parameters={
                    'bus_voltage': voltage,
                },
                severity=severity
            )

    def check_alarm_status(self, alarm):
        # If someone else cleared this alarm, we need to make sure to raise it again
        if not alarm.raised and len(self.failed_thrusters) != 0 and not alarm.parameters.get("clear_all", False):
            self.alert_thruster_loss(self.failed_thrusters[0], "Timed out")

    def check_for_drops(self, *args):
        for name, time in self.thruster_heartbeats.items():
            if time is None:
                # Thruster wasn't registered on startup
                continue

            elif rospy.Time.now() - time > rospy.Duration(self._dropped_timeout):
                rospy.logwarn("TIMEOUT, No recent response from: {}.".format(name))
                if name not in self.failed_thrusters:
                    self.alert_thruster_loss(name, "Timed out")

                # Check if the thruster is back up
                self.command_thruster(name, 0)

            elif name in self.failed_thrusters:
                rospy.logwarn("Thruster {} has come back online".format(name))
                self.alert_thruster_unloss(name)

    def alert_thruster_unloss(self, thruster_name):
        if thruster_name in self.failed_thrusters:
            self.failed_thrusters.remove(thruster_name)

        if len(self.failed_thrusters) == 0:
            self.thruster_out_alarm.clear_alarm(parameters={"clear_all"})
        else:
            severity = 3 if len(self.failed_thrusters) <= rospy.get_param("thruster_loss_limit", 2) else 5
            rospy.logerr(self.failed_thrusters)
            self.thruster_out_alarm.raise_alarm(
                parameters={
                    'thruster_names': self.failed_thrusters,
                },
                severity=severity
            )

    def alert_thruster_loss(self, thruster_name, last_update):
        if thruster_name not in self.failed_thrusters:
            self.failed_thrusters.append(thruster_name)

        # Severity rises to 5 if too many thrusters are out
        severity = 3 if len(self.failed_thrusters) <= rospy.get_param("thruster_loss_limit", 2) else 5
        rospy.logerr(self.failed_thrusters)
        self.thruster_out_alarm.raise_alarm(
            problem_description='Thruster {} has failed'.format(thruster_name),
            parameters={
                'thruster_names': self.failed_thrusters,
                'last_update': last_update
            },
            severity=severity
        )

    def fail_thruster(self, srv):
        self.alert_thruster_loss(srv.thruster_name, None)
        return FailThrusterResponse()

    @thread_lock(lock)
    def command_thruster(self, name, force):
        '''Issue a a force command (in Newtons) to a named thruster
            Example names are BLR, FLH, etc.
        '''
        target_port = self.port_dict[name]
        margin_factor = 1.0  # Not sure why this would not be anything but one - David + Jason
        clipped_force = np.clip(
            force,
            margin_factor * min(self.interpolate.x),
            margin_factor * max(self.interpolate.x)
        )
        normalized_force = self.interpolate(clipped_force)

        if name in self.failed_thrusters:
            normalized_force = 0

        # We immediately get thruster_status back
        thruster_status = target_port.command_thruster(name, normalized_force)

        # Don't try to do anything if the thruster status is bad
        if thruster_status is None:
            return

        message_contents = [
            'rpm',
            'bus_voltage',
            'bus_current',
            'temperature',
            'fault',
            'response_node_id',
        ]

        message_keyword_args = {key: thruster_status[key] for key in message_contents}
        self.thruster_heartbeats[name] = rospy.Time.now()
        self.status_pub.publish(
            ThrusterStatus(
                header=Header(stamp=rospy.Time.now()),
                name=name,
                **message_keyword_args
            )
        )

        # TODO: TEST
        self.bus_voltage_estimator.add_reading(message_keyword_args['bus_voltage'],
                                               rospy.Time.now())
        return

        # Undervolt/overvolt faults are unreliable
        if message_keyword_args['fault'] > 2:
            fault_codes = {
                (1 << 0): 'UNDERVOLT',
                (1 << 1): 'OVERRVOLT',
                (1 << 2): 'OVERCURRENT',
                (1 << 3): 'OVERTEMP',
                (1 << 4): 'STALL',
                (1 << 5): 'STALL_WARN',
            }
            fault = int(message_keyword_args['fault'])
            faults = []
            for code, fault_name in fault_codes.items():
                if code & fault != 0:
                    faults.append(fault_name)
            rospy.logwarn("Thruster: {} has entered fault with status {}".format(name, message_keyword_args))
            rospy.logwarn("Fault causes are: {}".format(faults))
            self.alert_thruster_loss(name, message_keyword_args)

    def thrust_cb(self, msg):
        '''Callback for receiving thrust commands
        These messages contain a list of instructions, one for each thruster
        '''
        for thrust_cmd in list(msg.thruster_commands):
            self.command_thruster(thrust_cmd.name, thrust_cmd.thrust)

    def stop(self):
        ''' Commands 0 thrust to all thrusters '''
        for name in self.port_dict.keys():
            if name not in self.failed_thrusters:
                self.command_thruster(name, 0.0)


if __name__ == '__main__':
    PKG = 'sub8_videoray_m5_thruster'
    usage_msg = "Interface to Sub8's VideoRay M5 thrusters"
    desc_msg = "Specify a path to the configuration.json file containing the thrust calibration data"
    parser = argparse.ArgumentParser(usage=usage_msg, description=desc_msg)
    parser.add_argument('--calibration_path', dest='calib_path',
                        help='Designate the absolute path of the calibration json file')
    args = parser.parse_args(rospy.myargv()[1:])
    config_path = args.calib_path

    rospy.init_node('videoray_m5_thruster_driver')

    layout_parameter = '/thruster_layout'
    rospy.loginfo("Thruster Driver waiting for parameter, {}".format(layout_parameter))
    thruster_layout = wait_for_param(layout_parameter)
    if thruster_layout is None:
        raise rospy

    thruster_driver = ThrusterDriver(config_path, thruster_layout['thruster_ports'],
                                     thruster_layout['thrusters'])
    rospy.spin()
