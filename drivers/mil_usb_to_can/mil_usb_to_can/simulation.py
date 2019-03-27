#!/usr/bin/python
from mil_misc_tools.serial_tools import SimulatedSerial
from utils import Packet, CommandPacket


class SimulatedCANDevice(object):
    '''
    Simulates a CAN device, with functions to be overrided
    to handle data requests and sends from motherboard
    '''
    def handle_request(self, length):
        '''
        Called when the motherboard requests data from this device
        @param length: number of bytes to return
        @return: should return a bytes/string object with the specificed length
        '''
        # For this default device, simply echo a bunch of zeros
        return '\x00' * length

    def handle_send(self, data):
        '''
        Called when the motherboard sends data to this device
        @param data: the data payload as a string/bytes object
        '''
        # Simply print the received payload
        print 'Got send ', data


class ExampleSimulatedCANDevice(SimulatedCANDevice):
    '''
    Example implementation of a SimulatedCANDevice.
    On sends, stores the transmited data in a buffer.
    When data is requested, it echos this data back.
    '''
    def __init__(self):
        self.echo_buffer = ''

    def handle_send(self, data):
        self.echo_buffer += data

    def handle_request(self, length):
        ret, self.echo_buffer = self.echo_buffer[0:length], self.echo_buffer[length:]
        return ret


class SimulatedUSBtoCAN(SimulatedSerial):
    '''
    Simulates the USB to CAN board. Is supplied with a dictionary of simualted
    CAN devices to simulate the behavior of the whole CAN network.
    @param devices: dictionary {device_id: SimulatedCANDevice} mapping CAN IDs
                    to SimulatedCANDevice instances that will be used for that ID
    '''
    def __init__(self, devices={0: SimulatedCANDevice()}, *args, **kwargs):
        self.devices = devices
        super(SimulatedUSBtoCAN, self).__init__()

    def write(self, data):
        # Parse incomming data as a command packet from motherboard
        p = CommandPacket.from_bytes(data)
        filter_id = p.filter_id
        # If no simulated device for this id, raise exception
        if filter_id not in self.devices:
            raise Exception('simulated version of {} not connected'.format(filter_id))
        # Call appropriate handle for simulated device
        if p.is_receive:
            payload = self.devices[filter_id].handle_request(p.length)
            self.buffer += Packet(payload).to_bytes()
        else:
            self.devices[filter_id].handle_send(p.data)
        return len(data)
