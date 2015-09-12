#!/usr/bin/env python
import numpy as np
import json
import rospy
import scipy.interpolate
import threading
import argparse
from sub8_msgs.msg import Thrust, ThrusterCmd
from sub8_ros_tools import wait_for_param, thread_lock
from sub8_msgs.srv import ThrusterInfo, ThrusterInfoResponse
from sub8_thruster_comm import thruster_comm_factory


lock = threading.Lock()
class ThrusterDriver(object):
    def __init__(self, config_path, bus_layout):
        '''Thruster driver, an object for commanding all of the sub's thrusters
            - Gather configuration data and make it available to other nodes
            - Instantiate ThrusterPorts, (Either simulated or real), for communicating with thrusters
            - Track a thrust_dict, which maps thruster names to the appropriate port
            - Given a command message, route that command to the appropriate port/thruster

        TODO:
            - Publish thruster status messages

        '''
        self.make_fake = rospy.get_param('simulate', False)
        if self.make_fake:
            rospy.logwarn("Running fake thrusters for simulation, based on parameter '/simulate'")

        # Individual thruster configuration data
        newtons, thruster_input = self.load_config(config_path)
        self.interpolate = scipy.interpolate.interp1d(newtons, thruster_input)

        # Bus configuration
        self.port_dict = self.load_bus_layout(bus_layout)

        thrust_service = rospy.Service('thruster_range', ThrusterInfo, self.get_thruster_info)
        self.thrust_sub = rospy.Subscriber('/thrust', Thrust, self.thrust_cb, queue_size=1)

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
        query_id = srv.thruster_id

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
        '''
        target_port = self.port_dict[name]
        clipped_force = np.clip(force, min(self.interpolate.x), max(self.interpolate.x))
        normalized_force = self.interpolate(clipped_force)

        # We immediately get thruster_status back
        thruster_status = target_port.command_thruster(name, force)

    def thrust_cb(self, msg):
        '''Callback for recieving thrust commands
        These messages contain a list of instructions, one for each thruster
        '''
        for thrust_cmd in msg.thruster_commands:
            self.command_thruster(thrust_cmd.name, thrust_cmd.thrust)


if __name__ == '__main__':
    '''
        --> Simulation capabilities (Return thruster ranges)
    '''
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