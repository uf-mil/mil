#!/usr/bin/env python

if __name__ == '__main__':
    import sys
    import rospy
    import rospkg
    import rosparam
    from sub8_thruster_comm import thruster_comm_factory
    from mil_misc_tools.text_effects import *
    from mil_misc_tools.terminal_input import get_ch
    
    rospack = rospkg.RosPack()
    
    def ports_from_layout(layout):
            '''Load and handle the thruster bus layout'''
            port_dict = {}
    
            for port in layout:
                thruster_port = thruster_comm_factory(port, fake=True)
    
                # Add the thrusters to the thruster dict
                for thruster_name, thruster_info in port['thrusters'].items():
                    port_dict[thruster_name] = thruster_port
    
            return port_dict

    rospy.init_node('thruster_spin_test')

    sub8_thruster_mapper = rospack.get_path('sub8_thruster_mapper')
    busses = rosparam.load_file(sub8_thruster_mapper + '/config/busses.yaml')[0][0]
    thruster_ports = ports_from_layout(busses)
    print len(thruster_ports)
    print thruster_ports
    names_from_motor_id = {0: 'FLH', 1: 'FLV', 2: 'FRH', 3: 'FRV',
                           4: 'BLH', 5: 'BLV', 6: 'BRH', 7: 'BRV'}
    active_thrusters = set()

    fprint = FprintFactory(title='thruster_spin_test', time=rospy.Time.now, auto_bold=False).fprint
    usage_msg = \
    '''
    Welcome to the thruster_spin_test tool.
    Instructions:
    * press up or down to control the thrust commanded
    * press the thruster id to spin that specific thruster (1-indexed, 0 means all)
    * if a thruster is already spinning, press its id again to stop it
    '''
    fprint(usage_msg)

    thrust = 0.5  # Normalized thrust in [0, 1]
    key = None

    def command_thrusters(timer_event):
        for motor_id in active_thrusters:
            name = names_from_motor_id[motor_id]
            thruster_ports[name].send_thrust_msg(motor_id, thrust)

    timer = rospy.Timer(period=rospy.Duration(0.01), callback=command_thrusters)

    while not rospy.is_shutdown():
       rospy.Rate(20) # just enough to be interesting
       key = get_ch()

       # quitting
       if key == 'q':
           sys.exit()

       # modify thrust
       if key == '\x1b':
           if get_ch() == '[':
               if get_ch() == 'A':  # UP key
                   thrust = thrust + 0.1 if thrust + 0.1 < 1.0 else 1.0
               if get_ch() == 'B':  # DOWN key
                   thrust = thrust - 0.1 if thrust + 0.1 < 1.0 else 0.0

       # toggle active thrusters
       if ord(key) in range(ord('0'), ord('9')):
           motor_id = int(key) - 1
           if motor_id is -1:
               if active_thrusters == {0, 1, 2, 3, 4, 5, 6, 7}:
                   active_thrusters = set()
               elif active_thrusters == set():
                   active_thrusters = {0, 1, 2, 3, 4, 5, 6, 7}
               else:
                   active_thrusters = set()
           else:
               if motor_id in active_thrusters:
                   active_thrusters.remove(motor_id)
               else:
                   active_thrusters.add(motor_id)

