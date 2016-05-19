@Scott:

# Topics of interest
```
/odom
/true_wrench

/wrench (but that is unclamped)
/thrusters/thrust


roslaunch sub8_launch duck.launch

rostopic pub wrench geometry_msgs/WrenchStamped (then press tab twice and put random stuff)
# I would suggest writing a ros node that publishes random wrenches
```

Links:

Jason's Stuff

https://github.com/jnez71/neural_control/blob/master/src/neural_control/nn_controller.py

http://www.pdx.edu/sites/www.pdx.edu.sysc/files/media_assets/SySc576_FrankLewisNNsControl.pdf