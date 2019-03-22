#!/usr/bin/python
from mil_misc_tools.serial_tools import SimulatedSerial
from utils import Packet, CommandPacket


class SimulatedCANDevice(object):
    '''
    Simulates a CAN device, with functions to be overrided
    to handle data requests and sends from motherboard
    '''
    def handle_request(self, length):
        return '\x00' * length

    def handle_send(self, data):
        print 'Got send ', data


class ExampleSimulatedCANDevice(SimulatedCANDevice):
    def __init__(self):
        self.echo_buffer = ''

    def handle_send(self, data):
        self.echo_buffer += data

    def handle_request(self, length):
        ret, self.echo_buffer = self.echo_buffer[0:length], self.echo_buffer[length:]
        return ret


class SimulatedUSBtoCAN(SimulatedSerial):
    def __init__(self, devices={0: SimulatedCANDevice()}, *args, **kwargs):
        self.devices = devices
        super(SimulatedUSBtoCAN, self).__init__()

    def write(self, data):
        p = CommandPacket.from_bytes(data)
        filter_id = p.filter_id
        if filter_id not in self.devices:
            raise Exception('simulated version of {} not connected'.format(filter_id))
        if p.is_receive:
            payload = self.devices[filter_id].handle_request(p.length)
            self.buffer += Packet(payload).to_bytes()
        else:
            self.devices[filter_id].handle_send(p.data)
        return len(data)
