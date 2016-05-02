#!/usr/bin/env python
import unittest
import numpy as np
from sub8_thruster_comm import thruster_comm_factory, FakeThrusterPort


class TestThrusterComm(unittest.TestCase):
    def setUp(self):
        port = '/dev/fake_port'
        node_id = 17
        thruster_name = 'BRL'

        self.port_info = {
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

    def test_thruster_comm_factory_fake(self):
        '''Test that the thruster factory returns a proper simulated FakeThrusterPort'''
        # This should succeed
        thrust_comm = thruster_comm_factory(self.port_info, fake=True)
        self.assertIsNotNone(thrust_comm)

    def test_fake_thruster_status(self):
        '''Test that the fake thruster status published is what we expect'''
        thrust_comm = FakeThrusterPort(self.port_info)
        fake_status = thrust_comm.command_thruster('BRV', 0.2)
        self.assertEqual(fake_status['bus_voltage'], 48)

    def test_thruster_comm_factory_real_fail(self):
        '''Test that the comm factory fails to create a ThrusterPort on a port that does not exist'''
        # This should fail
        with self.assertRaises(IOError):
            thrust_comm = thruster_comm_factory(self.port_info, fake=False)
            self.assertIsNotNone(thrust_comm)


if __name__ == '__main__':
    unittest.main()
