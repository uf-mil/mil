#!/usr/bin/env python
import rospy


class FakeThrusterPort(object):
    def __init__(self, port_info, thruster_definitions):
        '''Fake the behavior of a thruster'''
        self.port_name = port_info['port']
        self.thruster_dict = {}
        self.status_dict = {}
        self.missing_thrusters = []
        for thruster_name in port_info['thruster_names']:
            self.load_thruster_config(thruster_name, thruster_definitions[thruster_name])

    def load_thruster_config(self, thruster_name, thruster_info):
        self.thruster_dict[thruster_name] = thruster_info['motor_id']
        self.status_dict[thruster_name] = {
            'fault': 0,
            'rpm': 0,
            'temperature': 30,
            'bus_voltage': 48,
        }

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
            'bus_voltage',
            'bus_current',
            'temperature',
            'fault',
            'payload_checksum',
        ]
        response_dict = {key: self.status_dict[thruster_name].get(key, 0) for key in response_keys}
        return response_dict

    def command_thruster(self, thruster_name, normalized_thrust):
        '''
        Fake thruster command
        normalized_thrust should be between 0 and 1
        '''
        assert thruster_name in self.thruster_dict.keys(), "{} must be associated with this port".format(thruster_name)
        rospy.loginfo('Commanding {}: {}'.format(thruster_name, normalized_thrust))
        motor_id = self.thruster_dict[thruster_name]
        thruster_status = self.read_status(thruster_name)
        return thruster_status


if __name__ == '__main__':
    '''
    Module test code
    TODO: unit-test this if not already done
    '''
    import rospkg
    import rosparam
    import numpy.random as npr # haha
    sub8_thruster_mapper = rospkg.RosPack().get_path('sub8_thruster_mapper')
    thruster_layout = rosparam.load_file(sub8_thruster_mapper + '/config/thruster_layout.yaml')[0][0]
    print thruster_layout

    port_info = npr.choice(thruster_layout['thruster_ports'])
    print "port_info", port_info
    thruster_definitions = thruster_layout['thrusters']

    thruster_name = npr.choice(port_info['thruster_names'])
    motor_id = thruster_definitions[thruster_name]['motor_id']

    print'Test fake thruster comm over port {}, node_id {}, thruster name {}'.format(
        port_info['port'],
        motor_id,
        thruster_name
    )

    tp = FakeThrusterPort(port_info, thruster_definitions)
    print tp.command_thruster(thruster_name, 0.04)

