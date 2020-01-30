#!/usr/bin/env python
import numpy as np
import copy
import rospy
import rospkg
import rosparam
import threading
import argparse
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header, Float64
from sub8_msgs.msg import Thrust, ThrusterStatus
from mil_ros_tools import wait_for_param, thread_lock, numpy_to_point
from sub8_msgs.srv import ThrusterInfo, ThrusterInfoResponse, FailThruster, UnfailThruster
from sub8_thruster_comm import thruster_comm_factory
from ros_alarms import AlarmBroadcaster, AlarmListener
lock = threading.Lock()


class BusVoltageMonitor(object):

    '''
    Class that estimates sub8's thruster bus voltage.
    As of May 2017, this is just a simple rolling average with a constant width sliding
    window. However add_reading and get_estimate methods are left for when smarter
    filtering is needed
    '''
    VMAX = 50  # volts
    VMIN = 0  # volts

    class VoltageReading(object):
        def __init__(self, voltage, time):
            self.v = voltage
            self.t = time

    def __init__(self, window_duration):
        '''
        window_duration - float (amount of seconds for which to keep a reading in the buffer)
        '''
        self.bus_voltage_alarm = AlarmBroadcaster("bus-voltage")
        self.bus_voltage_pub = rospy.Publisher('bus_voltage', Float64, queue_size=1)
        self.warn_voltage = rospy.get_param("/battery/warn_voltage", 44.5)
        self.kill_voltage = rospy.get_param("/battery/kill_voltage", 44.0)
        self.last_estimate_time = rospy.Time.now()
        self.WINDOW_DURATION = rospy.Duration(window_duration)
        self.ESTIMATION_PERIOD = rospy.Duration(0.2)
        self.cached_severity = 0
        self.buffer = []

    def add_reading(self, voltage, time):
        ''' Adds voltage readings to buffer '''
        voltage = float(voltage)

        # Only add if it makes sense (the M5's will give nonsense feedback at times)
        if voltage >= self.VMIN and voltage <= self.VMAX:
            self.buffer.append(self.VoltageReading(voltage, time))
            self.prune_buffer()

        # check bus voltage if enough time has passed
        if rospy.Time.now() - self.last_estimate_time > self.ESTIMATION_PERIOD:
            self.check_bus_voltage()

    def prune_buffer(self):
        ''' Removes readings older than the window_duration from buffer '''
        for reading in self.buffer:
            age = rospy.Time.now() - reading.t
            if age > self.WINDOW_DURATION:
                self.buffer.remove(reading)

    def get_voltage_estimate(self):
        ''' Returns average voltage in buffer '''
        voltages = []
        if len(self.buffer) == 0:
            return None
        for r in self.buffer:
            voltages.append(r.v)
        return np.mean(voltages)

    def check_bus_voltage(self):
        ''' Publishes bus_voltage estimate and raises alarm if necessary '''
        bus_voltage = self.get_voltage_estimate()
        if bus_voltage is None:
            return

        self.bus_voltage_pub.publish(Float64(bus_voltage))

        severity = None
        if bus_voltage < self.warn_voltage:
            severity = 3
        if bus_voltage < self.kill_voltage:
            severity = 5

        if severity is not None and self.cached_severity != severity:
            self.bus_voltage_alarm.raise_alarm(
                problem_description='Bus voltage has fallen to {}'.format(bus_voltage),
                parameters={'bus_voltage': bus_voltage},
                severity=severity
            )
            self.cached_severity = severity


class ThrusterDriver(object):
    _dropped_timeout = 1.0  # s
    _window_duration = 30.0  # s
    _NODE_NAME = rospy.get_name()

    def __init__(self, ports_layout, thruster_definitions):
        '''Thruster driver, an object for commanding all of the sub's thrusters
            - Gather configuration data and make it available to other nodes
            - Instantiate ThrusterPorts, (Either simulated or real), for communicating with thrusters
            - Track a thrust_dict, which maps thruster names to the appropriate port
            - Given a command message, route that command to the appropriate port/thruster
            - Send a thruster status message describing the status of the particular thruster
        '''
        self.failed_thrusters = set()       # This is only determined by comms
        self.deactivated_thrusters = set()  # These will not come back online even if comms are good (user managed)

        # Alarms
        self.thruster_out_alarm = AlarmBroadcaster("thruster-out")
        AlarmListener("thruster-out", self.check_alarm_status, call_when_raised=False)  # Prevent outside interference

        # Create ThrusterPort objects in a dict indexed by port name
        self.load_thruster_ports(ports_layout, thruster_definitions)

        # Feedback on thrusters (thruster mapper blocks until it can use this service)
        self.thruster_info_service = rospy.Service('thrusters/thruster_info', ThrusterInfo, self.get_thruster_info)
        self.status_publishers = {name: rospy.Publisher('thrusters/status/' + name, ThrusterStatus, queue_size=10)
                                  for name in self.thruster_to_port_map.keys()}

        # These alarms require this service to be available before things will work
        rospy.wait_for_service("update_thruster_layout")
        self.update_thruster_out_alarm()

        # Bus voltage
        self.bus_voltage_monitor = BusVoltageMonitor(self._window_duration)

        # Command thrusters
        self.thrust_sub = rospy.Subscriber('thrusters/thrust', Thrust, self.thrust_cb, queue_size=1)

        # To programmatically deactivate thrusters
        self.fail_thruster_server = rospy.Service('fail_thruster', FailThruster, self.fail_thruster)
        self.unfail_thruster_server = rospy.Service('unfail_thruster', UnfailThruster, self.unfail_thruster)

    @thread_lock(lock)
    def load_thruster_ports(self, ports_layout, thruster_definitions):
        ''' Loads a dictionary ThrusterPort objects '''
        self.ports = {}                      # ThrusterPort objects
        self.thruster_to_port_map = {}       # node_id to ThrusterPort
        rospack = rospkg.RosPack()

        self.make_fake = rospy.get_param('simulate', False)
        if self.make_fake:
            rospy.logwarn("Running fake thrusters for simulation, based on parameter '/simulate'")

        # Instantiate thruster comms port
        for port_info in ports_layout:
            port_name = port_info['port']
            self.ports[port_name] = thruster_comm_factory(port_info, thruster_definitions, fake=self.make_fake)

            # Add the thrusters to the thruster dict and configure if present
            for thruster_name in port_info['thruster_names']:
                self.thruster_to_port_map[thruster_name] = port_info['port']

                if thruster_name not in self.ports[port_name].online_thruster_names:
                    rospy.logerr("ThrusterDriver: {} IS MISSING!".format(thruster_name))
                else:
                    rospy.loginfo("ThrusterDriver: {} registered".format(thruster_name))

                    # Set firmware settings
                    port = self.ports[port_name]
                    node_id = thruster_definitions[thruster_name]['node_id']
                    config_path = (rospack.get_path('sub8_videoray_m5_thruster') + '/config/firmware_settings/' +
                                   thruster_name + '.yaml')
                    rospy.loginfo('Configuring {} with settings specified in {}.'.format(thruster_name,
                                  config_path))
                    port.set_registers_from_dict(node_id=node_id,
                                                 reg_dict=rosparam.load_file(config_path)[0][0])
                    port.reboot_thruster(node_id)  # Necessary for some settings to take effect

    def get_thruster_info(self, srv):
        ''' Get the thruster info for a particular thruster name '''
        query_name = srv.thruster_name
        info = self.ports[self.thruster_to_port_map[query_name]].thruster_info[query_name]

        thruster_info = ThrusterInfoResponse(
            node_id=info.node_id,
            min_force=info.thrust_bounds[0],
            max_force=info.thrust_bounds[1],
            position=numpy_to_point(info.position),
            direction=Vector3(*info.direction)
        )
        return thruster_info

    def check_alarm_status(self, alarm):
        # If someone else cleared this alarm, we need to make sure to raise it again
        if not alarm.raised and alarm.node_name != self._NODE_NAME:
            self.update_thruster_out_alarm()

    def update_thruster_out_alarm(self):
        '''
        Raises or clears the thruster out alarm
        Updates the 'offline_thruster_names' parameter accordingly
        Sets the severity to the number of failed thrusters (clipped at 5)
        '''
        offline_names = list(self.failed_thrusters)
        if len(self.failed_thrusters) > 0:
            self.thruster_out_alarm.raise_alarm(
                node_name=self._NODE_NAME,
                parameters={'offline_thruster_names': offline_names},
                severity=int(np.clip(len(self.failed_thrusters), 1, 5)))
        else:
            self.thruster_out_alarm.clear_alarm(
                node_name=self._NODE_NAME,
                parameters={'offline_thruster_names': offline_names})

    @thread_lock(lock)
    def command_thruster(self, name, thrust):
        '''
        Issue a a force command (in Newtons) to a named thruster
            Example names are BLR, FLH, etc.
        Raises RuntimeError if a thrust value outside of the configured thrust bounds is commanded
        Raises UnavailableThrusterException if a thruster that is offline is commanded a non-zero thrust
        '''
        port_name = self.thruster_to_port_map[name]
        target_port = self.ports[port_name]
        thruster_model = target_port.thruster_info[name]

        if thrust < thruster_model.thrust_bounds[0] or thrust > thruster_model.thrust_bounds[1]:
            rospy.logwarn('Tried to command thrust ({}) outside of physical thrust bounds ({})'.format(
                thrust, thruster_model.thrust_bounds))

        if name in self.failed_thrusters:
            if not np.isclose(thrust, 0):
                rospy.logwarn('ThrusterDriver: commanding non-zero thrust to offline thruster (' + name + ')')

        effort = target_port.thruster_info[name].get_effort_from_thrust(thrust)

        # We immediately get thruster_status back
        thruster_status = target_port.command_thruster(name, effort)

        # Keep track of thrusters going online or offline
        offline_on_port = target_port.get_offline_thruster_names()
        for offline in offline_on_port:
            if offline not in self.failed_thrusters:
                self.failed_thrusters.add(offline)        # Thruster went offline
        for failed in copy.deepcopy(self.failed_thrusters):
            if (failed in target_port.get_declared_thruster_names() and
                    failed not in offline_on_port and
                    failed not in self.deactivated_thrusters):
                self.failed_thrusters.remove(failed)  # Thruster came online

        # Don't try to do anything if the thruster status is bad
        if thruster_status is None:
            return

        message_contents = [
            'rpm',
            'bus_v',
            'bus_i',
            'temp',
            'fault',
            'command_tx_count',
            'status_rx_count',
            'command_latency_avg'
        ]

        message_keyword_args = {key: thruster_status[key] for key in message_contents}
        power = thruster_status['bus_v'] * thruster_status['bus_i']
        self.status_publishers[name].publish(
            ThrusterStatus(
                header=Header(stamp=rospy.Time.now()),
                name=name,
                node_id=thruster_model.node_id,
                power=power,
                effort=effort,
                thrust=thrust,
                **message_keyword_args
            )
        )

        # Will publish bus_voltage and raise alarm if necessary
        self.bus_voltage_monitor.add_reading(message_keyword_args['bus_v'], rospy.Time.now())

        # Undervolt/overvolt faults are unreliable (might not still be true - David)
        if message_keyword_args['fault'] > 2:
            fault_codes = {
                (1 << 0): 'UNDERVOLT',
                (1 << 1): 'OVERRVOLT',
                (1 << 2): 'OVERCURRENT',
                (1 << 3): 'OVERTEMP',
                (1 << 4): 'STALL',
                (1 << 5): 'STALL_WARN',
            }
            fault = int(message_keyword_args['fault'])
            faults = []
            for code, fault_name in fault_codes.items():
                if code & fault != 0:
                    faults.append(fault_name)
            rospy.logwarn("Thruster: {} has entered fault with status {}".format(name, message_keyword_args))
            rospy.logwarn("Fault causes are: {}".format(faults))
        return

    def thrust_cb(self, msg):
        '''
        Callback for receiving thrust commands
        These messages contain a list of instructions, one for each thruster
        If there are any updates to the list of failed thrusters, it will raise and alarm
        '''
        failed_before = {x for x in self.failed_thrusters}

        for thrust_cmd in list(msg.thruster_commands):
            self.command_thruster(thrust_cmd.name, thrust_cmd.thrust)

        # Raise or clear 'thruster-out' alarm
        if not self.failed_thrusters == failed_before:
            rospy.logdebug('Failed thrusters:', self.failed_thrusters)
            self.update_thruster_out_alarm()

    def stop(self):
        ''' Commands 0 thrust to all thrusters '''
        for port in self.ports.values():
            for thruster_name in port.online_thruster_names.copy():
                self.command_thruster(thruster_name, 0.0)

    def fail_thruster(self, srv):
        ''' Makes a thruster unavailable for thrust allocation '''
        # So that thrust is not allocated to the thruster
        self.failed_thrusters.add(srv.thruster_name)

        # So that it won't come back online even if comms are good
        self.deactivated_thrusters.add(srv.thruster_name)

        # So that thruster_mapper updates the B-matrix
        self.update_thruster_out_alarm()
        return {}

    def unfail_thruster(self, srv):
        ''' Undoes effect of self.fail_thruster '''
        self.failed_thrusters.remove(srv.thruster_name)
        self.deactivated_thrusters.remove(srv.thruster_name)
        self.update_thruster_out_alarm()
        return {}


if __name__ == '__main__':
    PKG = 'sub8_videoray_m5_thruster'
    usage_msg = "Interface to Sub8's VideoRay M5 thrusters"
    desc_msg = "Specify a path to the configuration.json file containing the thrust calibration data"
    parser = argparse.ArgumentParser(usage=usage_msg, description=desc_msg)
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('videoray_m5_thruster_driver')

    layout_parameter = '/thruster_layout'
    rospy.loginfo("Thruster Driver waiting for parameter, {}".format(layout_parameter))
    thruster_layout = wait_for_param(layout_parameter)
    if thruster_layout is None:
        raise IOError('/thruster_layout rosparam needs to be set before launching the thruster driver')

    thruster_driver = ThrusterDriver(thruster_layout['thruster_ports'], thruster_layout['thrusters'])
    rospy.spin()
