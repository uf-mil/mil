#!/usr/bin/python
import rospy


class CANDeviceHandle(object):
    def __init__(self, driver, device_id):
        self._driver = driver
        self._device_id = device_id

    def request_data(self, length):
        return self._driver.request_data(self._device_id, length)

    def send_data(self, data):
        return self._driver.send_data(self._device_id, data)


class ExampleCANDeviceHandle(CANDeviceHandle):
    def __init__(self, *args, **kwargs):
        super(ExampleCANDeviceHandle, self).__init__(*args, **kwargs)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_cb)

    def timer_cb(self, *args):
        test = 'HELLOWORLD'
        self.send_data(test)
        res = self.request_data(len(test))
        assert res == test
        rospy.loginfo('Succesfully echoed {}'.format(test))
