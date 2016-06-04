Move thruster:

roslaunch sub8_launch sub8.launch

rostopic pub /thrusters/thrust sub8_msgs/Thrust Thrust.msg **tab** **tab** **enter thruster name (RRL, RLV, etc.) and a value (0.1 is safe)**

OR

On the VideoRay tool (http://download.videoray.com/documentation/m5_thruster/html/thruster_py.html)

python thruster.py -c <<SERIAL PORT>> -i <<NODE ID>> -m 0 0.1

**safe value is -0.1 or 0.1, full scale is -1 to 1**

Serial Ports & Node IDs are located at: Sub8/gnc/sub8_thruster_mapper/launch/thruster_mapper.launch