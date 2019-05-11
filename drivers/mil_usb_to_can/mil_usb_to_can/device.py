#!/usr/bin/python
import rospy
import struct
import random
import string
from application_packet import ApplicationPacket
from rospy_tutorials.srv import AddTwoInts


class CANDeviceHandle(object):
    '''
    Helper class to allow developers to write handles for communication with a particular CAN device.
    See ExampleCANDeviceHandle for an example of implementing one of these.
    '''
    def __init__(self, driver, device_id):
        '''
        Creates a CANDeviceHandle.
        @param driver: a USBtoCANBoard object that will be used for communication with the USB to CAN board
        @param device_id: the CAN id of the device this class will handle
        '''
        self._driver = driver
        self._device_id = device_id

    def on_data(self, data):
        '''
        Called when data is received from the device this handle is registered for
        '''
        pass

    def send_data(self, data, can_id=0):
        '''
        Sends data to the device
        @param data: the data payload to send to the device (string/bytes object)
        '''
        return self._driver.send_data(data, can_id=can_id)


class ExampleEchoDeviceHandle(CANDeviceHandle):
    '''
    An example implementation of a CANDeviceHandle which will handle
    a device that echos back any data sent to it.
    '''
    def __init__(self, *args, **kwargs):
        super(ExampleEchoDeviceHandle, self).__init__(*args, **kwargs)
        self.last_sent = None
        self.send_new_string()

    def on_data(self, data):
        if self.last_sent is None:
            print 'Received {} but have not yet sent anthing'.format(data)
        elif data != self.last_sent[0]:
            print 'ERROR! Reveived {} but last sent {}'.format(data, self.last_sent)
        else:
            print 'SUCCESSFULLY echoed {} in {}seconds'.format(
                self.last_sent[0],
                (rospy.Time.now() - self.last_sent[1]).to_sec())
            rospy.sleep(0.15)
            self.send_new_string()

    def send_new_string(self):
        # Example string to test with
        test = ''.join([random.choice(string.ascii_letters) for i in range(4)])
        self.last_sent = (test, rospy.Time.now())
        print 'SENDING {}'.format(test)
        self.send_data(test)


class ExampleAdderDeviceHandle(CANDeviceHandle):
    '''
    An example implementation of a CANDeviceHandle which will handle
    a device that echos back any data sent to it.
    '''
    def __init__(self, *args, **kwargs):
        super(ExampleAdderDeviceHandle, self).__init__(*args, **kwargs)
        self.correct_response = 37
        self.response_received = None
        self._srv = rospy.Service('add_two_ints', AddTwoInts, self.on_service_req)

    def on_service_req(self, req):
        payload = struct.pack('hh', req.a, req.b)
        self.correct_response = req.a + req.b
        self.response_received = None
        self.send_data(ApplicationPacket(37, payload).to_bytes())
        start = rospy.Time.now()
        while self.response_received is None:
            if rospy.Time.now() - start > rospy.Duration(1):
                return -1
        res = ApplicationPacket.from_bytes(self.response_received, expected_identifier=37)
        my_sum = struct.unpack('i', res.payload)
        return my_sum

    def on_data(self, data):
        self.response_received = data
