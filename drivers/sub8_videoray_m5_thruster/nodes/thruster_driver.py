#!/usr/bin/env python
import numpy as np
import json
import rospy
import scipy.interpolate
import threading
import argparse
from std_msgs.msg import Header
from sub8_msgs.msg import Thrust, ThrusterStatus
from sub8_ros_tools import wait_for_param, thread_lock
from sub8_msgs.srv import ThrusterInfo, ThrusterInfoResponse, FailThruster, FailThrusterResponse
from sub8_thruster_comm import thruster_comm_factory
from sub8_alarm import AlarmBroadcaster
lock = threading.Lock()


class ThrusterDriver(object):
    def __init__(self, config_path, bus_layout):
        '''Thruster driver, an object for commanding all of the sub's thrusters
            - Gather configuration data and make it available to other nodes
            - Instantiate ThrusterPorts, (Either simulated or real), for communicating with thrusters
            - Track a thrust_dict, which maps thruster names to the appropriate port
            - Given a command message, route that command to the appropriate port/thruster
            - Send a thruster status message describing the status of the particular thruster
        '''
        self.alarm_broadcaster = AlarmBroadcaster()
        self.thruster_out_alarm = self.alarm_broadcaster.add_alarm(
            name='thruster_out',
            action_required=True,
            severity=2
        )
        self.failed_thrusters = []

        self.make_fake = rospy.get_param('simulate', False)
        if self.make_fake:
            rospy.logwarn("Running fake thrusters for simulation, based on parameter '/simulate'")

        # Individual thruster configuration data
        newtons, thruster_input = self.load_config(config_path)
        self.interpolate = scipy.interpolate.interp1d(newtons, thruster_input)

        # Bus configuration
        self.port_dict = self.load_bus_layout(bus_layout)

        self.thrust_service = rospy.Service('thrusters/thruster_range', ThrusterInfo, self.get_thruster_info)
        self.thrust_sub = rospy.Subscriber('thrusters/thrust', Thrust, self.thrust_cb, queue_size=1)
        self.status_pub = rospy.Publisher('thrusters/thruster_status', ThrusterStatus, queue_size=8)

        # This is essentially only for testing
        self.fail_thruster_server = rospy.Service('fail_thruster', FailThruster, self.fail_thruster)

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
        for port in layout:
            thruster_port = thruster_comm_factory(port, fake=self.make_fake)

            # Add the thrusters to the thruster dict
            for thruster_name, thruster_info in port['thrusters'].items():
                port_dict[thruster_name] = thruster_port

        return port_dict

    @thread_lock(lock)
    def command_thruster(self, name, force):
        '''Issue a a force command (in Newtons) to a named thruster
            Example names are BLR, FLL, etc

            TODO:
                Make this still get a thruster status when the thruster is failed
                (We could figure out if it has stopped being failed!)
        '''
        if name in self.failed_thrusters:
            rospy.logwarn("Attempted to command failed thruster {}".format(name))
            return

        target_port = self.port_dict[name]
        clipped_force = np.clip(force, min(self.interpolate.x), max(self.interpolate.x))
        normalized_force = self.interpolate(clipped_force)

        # We immediately get thruster_status back
        thruster_status = target_port.command_thruster(name, normalized_force)
        message_contents = [
            'rpm',
            'bus_voltage',
            'bus_current',
            'temperature',
            'fault',
            'response_node_id',
        ]

        message_keyword_args = {key: thruster_status[key] for key in message_contents}
        if message_keyword_args['fault'] >= 2:
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

        self.status_pub.publish(
            ThrusterStatus(
                header=Header(stamp=rospy.Time.now()),
                name=name,
                **message_keyword_args
            )
        )

    def thrust_cb(self, msg):
        '''Callback for recieving thrust commands
        These messages contain a list of instructions, one for each thruster
        '''
        for thrust_cmd in list(msg.thruster_commands):
            self.command_thruster(thrust_cmd.name, thrust_cmd.thrust)

    def alert_thruster_loss(self, thruster_name, fault_info):
        self.thruster_out_alarm.raise_alarm(
            problem_description='Thruster {} has failed'.format(thruster_name),
            parameters={
                'thruster_name': thruster_name,
                'fault_info': fault_info
            }
        )
        self.failed_thrusters.append(thruster_name)

    def fail_thruster(self, srv):
        self.alert_thruster_loss(srv.thruster_name, None)
        return FailThrusterResponse()


if __name__ == '__main__':
    usage_msg = "Interface to Sub8's VideoRay M5 thrusters"
    desc_msg = "Specify a path to the configuration.json file containing the thrust calibration data"
    parser = argparse.ArgumentParser(usage=usage_msg, description=desc_msg)
    parser.add_argument('--configuration_path', dest='config_path',
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
