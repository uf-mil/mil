#!/usr/bin/env python
import rospy


class FakeThrusterPort(object):
    def __init__(self, port_info):
        '''Fake the behavior of a thruster'''
        self.port_name = port_info['port']
        self.thruster_dict = {}
        self.status_dict = {}
        self.missing_thrusters = []
        for thruster_name, thruster_info in port_info['thrusters'].items():
            self.load_thruster_config(thruster_name, thruster_info)

    def load_thruster_config(self, thruster_name, thruster_info):
        self.thruster_dict[thruster_name] = thruster_info['node_id']
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
        '''Fake thruster command
        normalized_thrust should be between 0 and 1'''
        assert thruster_name in self.thruster_dict.keys(), "{} must be associated with this port".format(thruster_name)
        rospy.loginfo('Commanding {}: {}'.format(thruster_name, normalized_thrust))
        motor_id = self.thruster_dict[thruster_name]
        thruster_status = self.read_status(thruster_name)
        return thruster_status


if __name__ == '__main__':
    '''Module test code'''
    port = '/dev/serial/by-id/usb-FTDI_USB-RS485_Cable_FTX1O9GJ-if00-port0'
    node_id = 17
    thruster_name = 'BRL'
    print 'Testing fake communication over port {}, node_id {}, thruster name {}'.format(port, node_id, thruster_name)

    port_info = {
        'port': port,
        'thrusters': {
            'BRV': {
                'node_id': 16,
            },
            thruster_name: {
                'node_id': node_id,
            }
        }
    }

    ftp = FakeThrusterPort(port_info)
    ftp.command_thruster('BRV', 0.2)
