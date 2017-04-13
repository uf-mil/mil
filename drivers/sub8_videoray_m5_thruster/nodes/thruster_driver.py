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


class ThrusterDriver(object):
    _dropped_timeout = 1 # s

    def __init__(self, config_path, bus_layout):
        '''Thruster driver, an object for commanding all of the sub's thrusters
            - Gather configuration data and make it available to other nodes
            - Instantiate ThrusterPorts, (Either simulated or real), for communicating with thrusters
            - Track a thrust_dict, which maps thruster names to the appropriate port
            - Given a command message, route that command to the appropriate port/thruster
            - Send a thruster status message describing the status of the particular thruster
        '''
        self.thruster_out_alarm = AlarmBroadcaster("thruster-out")
        AlarmListener("thruster-out", self.check_alarm_status, call_when_raised=False)
        self.bus_voltage_alarm = AlarmBroadcaster("bus-voltage")
        
        self.thruster_heartbeats = {}
        self.failed_thrusters = []

        self.make_fake = rospy.get_param('simulate', False)
        if self.make_fake:
            rospy.logwarn("Running fake thrusters for simulation, based on parameter '/simulate'")

        # Individual thruster configuration data
        newtons, thruster_input = self.load_config(config_path)
        self.interpolate = scipy.interpolate.interp1d(newtons, thruster_input)

        self.thrust_service = rospy.Service('thrusters/thruster_range', ThrusterInfo, self.get_thruster_info)
        self.status_pub = rospy.Publisher('thrusters/thruster_status', ThrusterStatus, queue_size=8)

        # Bus configuration
        self.port_dict = self.load_bus_layout(bus_layout)
        self.bus_voltage = None
        self.last_bus_voltage_time = rospy.Time.now()
        self.bus_voltage_pub = rospy.Publisher('bus_voltage', Float64, queue_size=1)
        self.bus_timer = rospy.Timer(rospy.Duration(0.1), self.publish_bus_voltage)
        self.drop_check = rospy.Timer(rospy.Duration(0.5), self.check_for_drops)
    
        # The bread and bones
        self.thrust_sub = rospy.Subscriber('thrusters/thrust', Thrust, self.thrust_cb, queue_size=1)

        # This is essentially only for testing
        self.fail_thruster_server = rospy.Service('fail_thruster', FailThruster, self.fail_thruster)

    def check_alarm_status(self, alarm):
        # If someone else cleared this alarm, we need to make sure to raise it again
        if not alarm.raised and len(self.failed_thrusters) != 0 and not alarm.parameters.get("clear_all", False):
            self.alert_thruster_loss(self.failed_thrusters[0], "Timed out")

    def publish_bus_voltage(self, *args):
        if (rospy.Time.now() - self.last_bus_voltage_time) > rospy.Duration(0.5):
            self.stop()
        
        if self.bus_voltage is not None:
            msg = Float64(self.bus_voltage)
            self.bus_voltage_pub.publish(msg)
            if self.bus_voltage < rospy.get_param("/battery/warn_voltage", 44.5):
                # This alert checks the severity of the battery voltage level as well 
                self.alert_bus_voltage(self.bus_voltage)

    def check_for_drops(self, *args):
        for name, time in self.thruster_heartbeats.items():
            if time is None:
                # Thruster wasn't registered on startup
                continue

            elif time < rospy.Time.now().to_sec() - self._dropped_timeout:
                rospy.logwarn("TIMEOUT, No recent response from: {}.".format(name))
                if name not in self.failed_thrusters:
                    self.alert_thruster_loss(name, "Timed out")

                # Check if the thruster is back up
                self.command_thruster(name, 0)

            elif name in self.failed_thrusters:
                rospy.logwarn("Thruster {} has come back online".format(name))
                self.alert_thruster_unloss(name)

    def update_bus_voltage(self, voltage):
        if self.bus_voltage is None:
            self.bus_voltage = voltage
            return

        if 40.0 < voltage < 50.0:
            self.last_bus_voltage_time = rospy.Time.now()
            self.bus_voltage = (0.1 * voltage) + (0.9 * self.bus_voltage)

    def load_config(self, path):
        '''Load the configuration data:
            - Map force inputs from Newtons to [-1, 1] required by the thruster
        '''
        try:
            _file = file(path)
        except IOError, e:
            rospy.logerr("Could not find thruster configuration file at {}".format(path))
            raise(e)

        json_data = json.load(_file)
        newtons = json_data['calibration_data']['newtons']
        thruster_input = json_data['calibration_data']['thruster_input']
        return newtons, thruster_input

    def get_thruster_info(self, srv):
        '''Get the thruster info for a particular thruster ID
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

    @thread_lock(lock)
    def load_bus_layout(self, layout):
        '''Load and handle the thruster bus layout'''
        port_dict = {}

        # These alarms require this service to be available before things will work
        rospy.wait_for_service("update_thruster_layout")
        self.thruster_out_alarm.clear_alarm(parameters={'clear_all': True})
        
        for port in layout:
            thruster_port = thruster_comm_factory(port, fake=self.make_fake)

            # Add the thrusters to the thruster dict
            for thruster_name, thruster_info in port['thrusters'].items():
                if thruster_name in thruster_port.missing_thrusters:
                    rospy.logerr("{} IS MISSING!".format(thruster_name))
                    self.alert_thruster_loss(thruster_name, "Motor ID was not found on it's port.")
                else:
                    rospy.loginfo("{} registered".format(thruster_name))

                self.thruster_heartbeats[thruster_name] = None 
                port_dict[thruster_name] = thruster_port
            
        return port_dict

    def stop(self):
        for name in self.port_dict.keys():
            if name not in self.failed_thrusters:
                self.command_thruster(name, 0.0)

    @thread_lock(lock)
    def command_thruster(self, name, force):
        '''Issue a a force command (in Newtons) to a named thruster
            Example names are BLR, FLH, etc.
        '''
        target_port = self.port_dict[name]
        margin_factor = 0.8
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
        self.thruster_heartbeats[name] = rospy.Time.now().to_sec() 
        self.status_pub.publish(
            ThrusterStatus(
                header=Header(stamp=rospy.Time.now()),
                name=name,
                **message_keyword_args
            )
        )

        # TODO: TEST
        self.update_bus_voltage(message_keyword_args['bus_voltage'])
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

    def alert_bus_voltage(self, voltage):
        severity = 3 
        # Kill if the voltage goes too low
        if self.bus_voltage < rospy.get_param("/battery/kill_voltage", 44.0):
            severity = 5
        
        self.bus_voltage_alarm.raise_alarm(
            problem_description='Bus voltage has fallen to {}'.format(voltage),
            parameters={
                'bus_voltage': voltage,
            },
            severity=severity
        )

    def fail_thruster(self, srv):
        self.alert_thruster_loss(srv.thruster_name, None)
        return FailThrusterResponse()


if __name__ == '__main__':
    PKG = 'sub8_videoray_m5_thruster'
    usage_msg = "Interface to Sub8's VideoRay M5 thrusters"
    desc_msg = "Specify a path to the configuration.json file containing the thrust calibration data"
    parser = argparse.ArgumentParser(usage=usage_msg, description=desc_msg)
    parser.add_argument('--configuration_path', dest='config_path',
                        default=rospkg.RosPack().get_path(PKG) + '/config/calibration.json',
                        help='Designate the absolute path of the calibration/configuration json file')
    args = parser.parse_args(rospy.myargv()[1:])
    config_path = args.config_path

    rospy.init_node('videoray_m5_thruster_driver')

    layout_parameter = '/busses'
    rospy.loginfo("Thruster Driver waiting for parameter, {}".format(layout_parameter))
    busses = wait_for_param(layout_parameter)
    if busses is None:
        raise(rospy.exceptions.ROSException("Failed to find parameter '{}'".format(layout_parameter)))

    thruster_driver = ThrusterDriver(config_path, busses)
    rospy.spin()
