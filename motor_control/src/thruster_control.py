#!/usr/bin/env python

##
##                          thruster_control node
##
## Purpose:
##  This script accepts ROS msgs of type thrusterConfig on topic /thruster_status
##  and converts the given newtons into a degree output sutiable for the arduino
##  servo class. A ramping function then moves the current value to the desired value
##  This value is ouput over serial each itteration to an arduino with the ino file in
##  the src folder of this same package, the arduino then generates the appropriate pwm
##  to send to the Victor 888 motor contolers which are attached to the thrusters
##
##  This node (thruster_control) also ouputs the current newton output of the particular
##  thruster periodically
##
## Trouble shooting:
##  If this node stops sending to the motors propely try decreasing RAMP_RATE by 0.1
##  If that doesn't work try decreasing the ros rate (sleep time)
##

import rospy
import serial
from std_msgs.msg import Float64
from motor_control.msg import thrusterNewtons, ControlThrustConfig
import time

#Define some constants
#Note: these should be replaced with ros Params
STARBOARD_THRUSTER = 2
PORT_THRUSTER = 3

#    PWM and coresponding values, 20 ms duty cycle
## Direction:       Reverse  Stop    Forward    unit
## Output value:    45       91      145        Degrees
## Pulse width:     1.04    ~1.49    2.07       ms
## Newton input:   -23.65047 0       39.24135   N
#ZERO_DEG = 91               #Zero in degrees
#MAX_DEG = 145               #full forward in degrees
#MIN_DEG = 45                #full reverse in degrees
#MAX_NEWTONS = 39.24135      #Full forward in newtons
#MIN_NEWTONS = -23.65047     #Full reverse in newtons
MAX_NEWTONS =  100.0         #Full forward Jacksons motors
MIN_NEWTONS =  -100.0        #Full reverse Jacksons
ABS_MAX_PW = 2000
ABS_MIN_PW = 1000
ZERO_PW = 1500
REV_CONV = (ZERO_PW - ABS_MIN_PW) / (0 - MIN_NEWTONS)
FWD_CONV = (ABS_MAX_PW - ZERO_PW) / MAX_NEWTONS

#control_kill = True

#These are the valuse that the thrusters
# attempt to achieve in newtons
starboard_setpoint = 0.0
port_setpoint = 0.0
#Values that the thrusters are currently at
# in newtons
port_current = 0.0
starboard_current = 0.0

#Timing Variables
PUB_RATE = rospy.Duration(0.01)
UPDATE_RATE = 100                      #Update at 100 Hz
RAMP_RATE = (MAX_NEWTONS / 2.0) / UPDATE_RATE    # (Newtons/second) / Hz ==> N, i.e. 10 / UPDATE_RATE amounts to the boat changing 10 newtons per second

# state vars
killed = False

#Pub
newton_pub = rospy.Publisher('thruster_status', thrusterNewtons, queue_size=10)
pwm_pub = rospy.Publisher('/controller/thruster_config', ControlThrustConfig, queue_size=10)
stm1_pub = rospy.Publisher('/stm32f3discovery_imu_driver/pwm1', Float64, queue_size=10)
stm2_pub = rospy.Publisher('/stm32f3discovery_imu_driver/pwm2', Float64, queue_size=10)

#       stopThrusters
# Input: none
# Output: none
# Description: Sets both thrusters to ZERO_PWM imediatly
def stopThrusters():
    global starboard_setpoint
    global port_setpoint
    global starboard_current
    global port_current

    msg = ControlThrustConfig()
    msg.controller = 'azi_drive'
    msg.id = ControlThrustConfig.STARBOARD
    msg.pulse_width = ZERO_PW / 1000000.0
    pwm_pub.publish(msg)
    msg.id = ControlThrustConfig.PORT
    pwm_pub.publish(msg)

    #Zero internal varibles
    starboard_setpoint = 0.0;
    port_setpoint = 0.0;
    starboard_current = 0.0
    port_current = 0.0

#   motorConfigCallback
# input: thrusterConfig
# Output: none
# Description: This is the call back for config msgs to the thrusters
#               It checks to ensure the correct thrusters are being addressed
#               It checks to make sure the thrust settings are within bounds
#               It then sets the desired level for the ramp function in main
#               to work towards
def motorConfigCallback(config):
    global starboard_setpoint
    global port_setpoint

    #Check to ensure we are working with an appropriate thruster
    if (config.id != STARBOARD_THRUSTER and config.id != PORT_THRUSTER):
        rospy.logwarn("Id: %i, is not a known id", config.id);
        return


    thrust = config.thrust

    #Check if we are more than 0.25 newtons out of bounds
    if thrust > MAX_NEWTONS:
        if thrust > (MAX_NEWTONS + 0.25):
            rospy.logwarn("Attempted to set thruster %i to %f which is significantly above maximum (%fN) output thrust cliped to max. achivable value", config.id, thrust, MAX_NEWTONS)
        thrust = MAX_NEWTONS;
    elif thrust < MIN_NEWTONS:
        if thrust < (MIN_NEWTONS - 0.25):
            rospy.logwarn("Attempted to set thruster %i to %f which is significantly below minimum (%fN) output thrust cliped to min. achivable value", config.id, thrust, MIN_NEWTONS)
        thrust = MIN_NEWTONS
    #Set the setpoints
    if config.id == STARBOARD_THRUSTER:
        #print(config.thrust)
        #print(thrust)
        starboard_setpoint = thrust
    elif config.id == PORT_THRUSTER:
        port_setpoint = thrust

#   convertNewtonsToPW
# Input: floating point (Newtons)
# Output: floating point (degrees)
# Description: Converts a value in newtons to a degree value suitable for the
#               arduino servo class. From testing a degree 3 polynomial
#               Interpolation is used to convert newtons in the positive
#               and negative region to Degrees
def convertNewtonsToPW(newtons):
    out = newtons
    #print("Convert %i to PWM", out)
    # These equations were determined experimentally
    # Degree 3 polynomials were used to map newton inputs to
    # the coresponding forward and reverse deg representaitons:
    #   Forward_deg = 0.0016x^3 - 0.1027x^2 + 2.812X + 96.116
    #   Reverse_deg = 0.0055x^3 + 0.224X^2 + 3.9836x + 86.679
    # Where x is equal to the inputed newtons

    # PW = (PW_RANGE/NEWTON_RANGE)*NEWTONS + PW_ZER0
    if out < 0:
        #out = 0.0055*out**3 + 0.224*out**2 + 3.9836 * out + 86.679
        out = REV_CONV * out + ZERO_PW
        #print("Reverse: %i" % out)
    elif out > 0:
        #out = 0.0016*out**3 - 0.1027*out**2 + 2.812*out + 96.116
        out = FWD_CONV * out + ZERO_PW
        #print("Forward: %i" % out)
    else:
        #out = ZERO_DEG;
        out = ZERO_PW
        #print("Zero: %i" % ZERO_PW)

    #Bounds should be taken care of in the motor callback
    #Just in case we double check the bounds after conversion
    if out > ABS_MAX_PW:
        out = ABS_MAX_PW
    elif out < ABS_MIN_PW:
        out = ABS_MIN_PW
    #print "Bounded out: %i" % out
    return out

# pubStatus
# Input: event (not actually used)
# Output: none
# Description: Periodically outputs the thruster values
#               This is the callback function to a rospytimer
#               (pub_timer) defined in main
#               The output rate is determined by PUB_RATE
def pubStatus(event):
    thruster = thrusterNewtons(STARBOARD_THRUSTER, starboard_current)
    newton_pub.publish(thruster)
    thruster = thrusterNewtons(PORT_THRUSTER, port_current)
    newton_pub.publish(thruster)

#   Set Kill
# On a kill we should stop all thrusters imideatly
def set_kill():
    global killed    #Zero internal varibles
    global starboard_setpoint
    global port_setpoint
    global starboard_current
    global port_current

    killed = True
    starboard_setpoint = 0.0;
    port_setpoint = 0.0;
    starboard_current = 0.0
    port_current = 0.0

    rospy.logwarn('Newtons to PWM Killed because: %s' % kill_listener.get_kills())

#   Clear Kill
# On an unkill we should start all thrusters from 0!
def clear_kill():
    global killed
    global starboard_setpoint
    global port_setpoint
    global starboard_current
    global port_current

    starboard_setpoint = 0.0;
    port_setpoint = 0.0;
    starboard_current = 0.0
    port_current = 0.0

    killed = False

    rospy.loginfo('Newtons to PWM Unkilled')

#def control_callback(msg):
#    global control_kill
#    if msg.data == True:
#        control_kill = False
#    if msg.data == False:
#        control_kill = True

#   thrusterCtrl
# Input: None
# Output: None
# Description: This is the main code
#               Sets up ROS enviorment an exicutes functionality
#               as described at top of this script
def thrusterCtrl():
    global port_current
    global starboard_current
    global control_kill

    #Setup ros
    rospy.init_node('thruster_control')
    #rospy.Subscriber("control_arbiter", Bool, control_callback)
    rospy.Subscriber("thruster_config", thrusterNewtons, motorConfigCallback)
    r = rospy.Rate(UPDATE_RATE)          #1000 hz(1ms Period)... I think
    pub_timer = rospy.Timer(PUB_RATE, pubStatus)

    #Initilize the motors to 0
    stopThrusters()

    #Main loop
    while not rospy.is_shutdown():

        #Simple ramping function
        #If its less than the setpoint add RAMP_RATE
        #If its greater than the setpoint subtract RAMP_RATE
        #If we overshoot set thrust equal to the setpoint
        if port_current > port_setpoint:
            port_current -= RAMP_RATE
            if port_current < port_setpoint:
                port_current = port_setpoint
        elif port_current < port_setpoint:
            port_current += RAMP_RATE
            if port_current > port_setpoint:
                port_current = port_setpoint

        if starboard_current > starboard_setpoint:
            starboard_current -= RAMP_RATE
            if starboard_current < starboard_setpoint:
                starboard_current = starboard_setpoint
        elif starboard_current < starboard_setpoint:
            starboard_current += RAMP_RATE
            if starboard_current > starboard_setpoint:
                starboard_current = starboard_setpoint

        msg = ControlThrustConfig()
        msg.controller = 'azi_drive'
        msg.id = ControlThrustConfig.PORT
        msg.pulse_width = convertNewtonsToPW(port_current) / 1000000.0
        stm1_pub.publish(msg.pulse_width)
        print(msg)

        msg.id = ControlThrustConfig.STARBOARD
        msg.pulse_width = convertNewtonsToPW(starboard_current) / 1000000.0
        stm2_pub.publish(msg.pulse_width)

        #Wait till next cycle
        r.sleep()

    #Clean up
    stopThrusters()         #Stop motors

    # Kill the system


if __name__ == '__main__':
    try:
        thrusterCtrl()
    except rospy.ROSInterruptException: pass
