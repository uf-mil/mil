# Drone Project

## Notes on hardware and software

### Current hardware
The drone from Aerotestra is currently configured to use a (very old) ardupilot Arduino-based controller.
There are several other drones in the Software Dungeon that have other controllers.
One of them (our **octocopter**) is running a newer [Pixhawk controller](https://docs.px4.io/main/en/flight_controller/mro_pixhawk.html)
that is easily compatible with the newest [ground control software](https://docs.qgroundcontrol.com/master/en/releases/release_notes.html).
Currently, we are only considering using these more current controllers.

The exact model of magnetometer/GPS unit is currently unknown.
It is a 3DRobotics unit from ~2016.


### Software

Zobby is currently the in-lab computer being used to develop the drone project.
Zobby has QGroundControl installed as an AppImage ( in `~/Downloads/appimages`)

The Pixhawk has been flashed with the latest available version of the Pixhawk firmware,
rather than the arudpilot firmware. This decision was made because it seemed
most likely to work smoothly.


### Implimentation and Configuration
The [OrangeRX T-Six](https://hobbyking.com/en_us/orangerx-t-six-2-4ghz-dsm2-compatible-6ch-transmitter-w-10-model-memory-and-3-pos-switch-mode-2.html)
transmitter is paired to the receiver on the homemade octocopter, which has the Pixhawk onboard currently.
The Pixhawk magnetometer has been calibrated roughly, although some drift still occurs.


#### Synchronizing the RF transmitter and receiver
![Controller Synchronizing](https://cdn-global-hk.hobbyking.com/media/file/330895068X111007X3.jpg)

## Comments by Sean (Founder of Aerotestra)

- He can provide help with the OS systems and the current state of the vehicle. Confirm that whatever is inside is updated, autopilot can be controlled through a telemetry interface. Drone uses the ardupilot, which Jarrod Sanders has volunteered to become familiar with and keep communicating with Sean.  
- There is no specific documentation for our vehicle because it is a customized vehicle. But he will send us the user manual. 
- Sean will send us an [ardupilot replacement](https://ardupilot.org/) he has laying around. 
- He mentioned that the [AR2.5](https://ardupilot.org/copter/docs/common-apm25-and-26-overview.html) ardupilot is a bit outdated and he recommends Pixhawk but we might get away with it if it's working properly. He mentioned that there could be corrosion problems, expanded Teflon since it is semipermeable could be used in the enclosure to reduce exposure to water. 
- He recommended to also check the bearings in the motor to check for corrosion and resistance when turning. If there is, consider replacing the motors.  
- He will send us a telemetry unit, and the ground unit. Also, he will send a selection bag with rubber gaskets for the GPS enclosure. 
- Next steps are to get [Mission Planner](https://ardupilot.org/planner/) on a laptop and interface with the vehicle. Then research how to interface with ROS. 
- He also recommended <https://github.com/rmackay9> and <https://irlock.readme.io/docs/1-sensor-firmware-software>, <https://irlock.com/products/ir-lock-sensor-precision-landing-kit> for landing.
