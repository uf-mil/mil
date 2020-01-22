#!/usr/bin/env python
import unittest
from sub8_thruster_comm import thruster_comm_factory, FakeThrusterPort, Sub8SerialException
import rospkg
import rosparam

rospack = rospkg.RosPack()


class TestThrusterComm(unittest.TestCase):

    def setUp(self):
        sub8_thruster_mapper = rospack.get_path('sub8_thruster_mapper')
        self.thruster_layout = rosparam.load_file(sub8_thruster_mapper + '/config/thruster_layout.yaml')[0][0]

    def test_thruster_comm_factory_fake(self):
        '''Test that the thruster factory returns a proper simulated FakeThrusterPort'''
        # This should succeed
        thrust_comm = thruster_comm_factory(self.thruster_layout['thruster_ports'][0],
                                            self.thruster_layout['thrusters'], fake=True)
        self.assertIsNotNone(thrust_comm)

    def test_fake_thruster_status(self):
        '''Test that the fake thruster status published is what we expect'''
        port_info = self.thruster_layout['thruster_ports'][0]
        thruster_defs = self.thruster_layout['thrusters']
        thrust_comm = FakeThrusterPort(port_info, thruster_defs)
        fake_status = thrust_comm.command_thruster(port_info['thruster_names'][0], 0.2)
        self.assertEqual(fake_status['bus_v'], 48)

    def test_thruster_comm_factory_real_fail(self):
        '''Test that the comm factory fails to create a ThrusterPort on a port that does not exist'''
        # This should fail
        port_info = self.thruster_layout['thruster_ports'][0]
        thruster_defs = self.thruster_layout['thrusters']
        port_info['port'] = 'bad_name'
        with self.assertRaises(Sub8SerialException):
            thrust_comm = thruster_comm_factory(port_info, thruster_defs, fake=False)
            self.assertIsNotNone(thrust_comm)

if __name__ == '__main__':
    unittest.main()
