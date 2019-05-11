#!/usr/bin/env python
import time
import argparse
import sub8_thruster_comm as thruster_comm
import rospy

'''
TODO tools
    - List thrusters and spin them
    - Determine whether thrusters are in fault
        - Un-fault thrusters

- Reorganize thruster_comm so that it only knows the port
'''


def test_thruster():
    usage_msg = "Dev tool for testing thrusters"
    desc_msg = "Author: Rev. Jacob Panikulam"
    parser = argparse.ArgumentParser(usage=usage_msg, description=desc_msg)
    parser.add_argument(
        '--node_id', dest='node_id',
        help='Thruster node id'
    )
    parser.add_argument(
        '--port', dest='port',
        help='Check /dev/serial/by-id'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    port_info = {
        'port': args.port,
        'thrusters': {
            'my_thruster': {
                'node_id': args.node_id,
            }
        }
    }

    tc = thruster_comm.ThrusterPort(port_info)
    qq = [
        'undervoltage_trigger',
        'overvoltage_trigger',
        'overcurrent_trigger',
        'temp_trigger',
        'stall_count_max',
        'fault_control',
        'fault',
        'save_settings',
        'undervoltage_err_cnt',
        'overvoltage_err_cnt',
        'overcurrent_err_cnt',
        'temp_err_cnt',
        'stall_err_cnt',
    ]
    for q in qq:
        r = tc.to_register(args.node_id, q)
        print q, r
        time.sleep(0.5)


if __name__ == '__main__':
    test_thruster()
