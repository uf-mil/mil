#!/usr/bin/env python
import rospy
from sub8_thruster_comm import ThrusterModel


class FakeThrusterPort(object):

    def __init__(self, port_info, thruster_definitions):
        '''Fake the behavior of a thruster'''
        self.port_name = port_info['port']
        self.thruster_dict = {}
        self.thruster_info = {}
        self.status_dict = {}
        self.online_thruster_names = []
        self.missing_thrusters = []
        for thruster_name in port_info['thruster_names']:
            self.load_thruster_config(
                thruster_name, thruster_definitions[thruster_name])

    def load_thruster_config(self, thruster_name, thruster_info):
        self.thruster_dict[thruster_name] = thruster_info['node_id']
        self.thruster_info[thruster_name] = ThrusterModel(thruster_info)
        self.online_thruster_names.append(thruster_name)
        self.status_dict[thruster_name] = {
            'fault': 0,
            'rpm': 0,
            'temp': 30,
            'bus_v': 48,
        }

    def get_offline_thruster_names(self):
        return self.missing_thrusters

    def set_registers_from_dict(*args, **kwargs):
        pass

    def reboot_thruster(*args, **kwargs):
        pass

    def read_status(self, thruster_name):
        '''
            TODO: Add simulated thruster-out
        '''
        # The data that the thruster sends back
        response_keys = [
            'sync',
            'response_node_id',
            'flag',
            'CSR_address',
            'length',
            'header_checksum',
            'device_type',
            'rpm',
            'bus_v',
            'bus_i',
            'temp',
            'fault',
            'payload_checksum',
            'command_tx_count',
            'status_rx_count',
            'command_latency_avg'
        ]
        response_dict = {key: self.status_dict[
            thruster_name].get(key, 0) for key in response_keys}
        return response_dict

    def get_declared_thruster_names(self):
        ''' Gets the names of all the ports that were declared on this port '''
        return self.thruster_info.keys()

    def command_thruster(self, thruster_name, normalized_thrust):
        '''
        Fake thruster command
        normalized_thrust should be between 0 and 1
        '''
        assert thruster_name in self.thruster_dict.keys(
        ), "{} must be associated with this port".format(thruster_name)
        rospy.loginfo('Commanding {}: {}'.format(
            thruster_name, normalized_thrust))
        thruster_status = self.read_status(thruster_name)
        return thruster_status


if __name__ == '__main__':
    '''
    Module test code
    TODO: unit-test this if not already done
    '''
    import rospkg
    import rosparam
    import numpy.random as npr  # haha
    sub8_thruster_mapper = rospkg.RosPack().get_path('sub8_thruster_mapper')
    thruster_layout = rosparam.load_file(
        sub8_thruster_mapper + '/config/thruster_layout.yaml')[0][0]
    print thruster_layout

    port_info = npr.choice(thruster_layout['thruster_ports'])
    print "port_info", port_info
    thruster_definitions = thruster_layout['thrusters']

    thruster_name = npr.choice(port_info['thruster_names'])
    motor_id = thruster_definitions[thruster_name]['node_id']

    print 'Test fake thruster comm over port {}, node_id {}, thruster name {}'.format(
        port_info['port'],
        motor_id,
        thruster_name
    )

    tp = FakeThrusterPort(port_info, thruster_definitions)
    print tp.command_thruster(thruster_name, 0.04)
