#!/usr/bin/python
import rospy
from board import USBtoCANBoard
import importlib


class USBtoCANDriver(object):
    '''
    '''
    def __init__(self):
        port = rospy.get_param('~port', '/dev/tty0')
        baud = rospy.get_param('~baudrate', 9600)
        simulation = rospy.get_param('/is_simulation', True)
        if simulation:
            simulated_devices = rospy.get_param('~simulated_devices')
            devices = {}
            for device_id, module_name in simulated_devices.iteritems():
                device_id = int(device_id)
                module_name, cls = module_name.rsplit('.', 1)
                module = importlib.import_module(module_name)
                devices[device_id] = getattr(module, cls)()
            self.board = USBtoCANBoard(port=port, baud=baud, simulated=simulation, devices=devices)
        else:
            self.board = USBtoCANBoard(port=port, baud=baud, simulated=simulation)
        devices = rospy.get_param('~device_handles')
        self.handles = []
        for device_id, module_name in devices.iteritems():
            device_id = int(device_id)
            module_name, cls = module_name.rsplit('.', 1)
            module = importlib.import_module(module_name)
            self.handles.append(getattr(module, cls)(self.board, device_id))


if __name__ == '__main__':
    rospy.init_node('usb_to_can_driver')
    driver = USBtoCANDriver()
    rospy.spin()
