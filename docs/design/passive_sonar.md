# Passive sonar
TODO: more details

## Testing ROS bridge
* Download [example TCP dump](http://sylphase.com/files/oof.bin)
* Run ROS bridge `rosrun mil_passive_sonar sylphase_sonar_ros_bridge _port:=10001 _ip:=127.0.0.1`
* Either stream a TCP dump or run the driver on a real robot
  * Stream TCP dump `rosrun mil_tools stream_tcp_dump  ~/Downloads/oof.bin --port=10001 --ip 127.0.0.1`
  * On Sub8 run the actual driver `(cd ~/.mil/sylphase-sonar/; ./driver/publish 10001)`
* Make sure samples are being published `rostopic echo /samples`

