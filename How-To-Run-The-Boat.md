# **How To Run The Boat**

There are a few more steps to starting up the real boat than the simulator. 

**Everything in this page assumes you have completed the step in the [[Networking with NaviGator]]**

### 1. Verify that your computer can ping the boat and both routers by running the three commands below individually. 

    ping navigator
    ping nav_router_onboard
    ping nav_router_onshore

If you can ping all three systems you are good to go!

### 2. SSH into the boat and start the hardware launch files

First, start a screen by running 

    screen -t navigator_onboard_screen

Next run:

    roslaunch navigator_launch motor_control.launch

Finally run:

    screen -d

3. On the shore computer open a new terminal and start the necessary launch files

Run:

    roslaunch navigator_launch gnc.launch

Then navigate to the NaviGator GUI.  

## How To Use The GUI

Kill and Revive do not work at the moment, waiting for the Sub8 alarm system to be updated. 

**The boat starts in a killed state so until a control mode (rc, autonomous, or gui) is chosen the boat remains killed. So choose which controller you want and go from there.** 

The three modes are the the three bottom buttons. RC sets it to be controlled by the Xbox controller. AU sets it to be controlled by the autonomous controller.  GUI sets it to be controlled by the sliders on the right

If in 'gui' control mode the sliders on the right are active and can be used to give the boat a movement command. Only one can be used at once though. If moving forward and wanting to move back, the slider for forward must be set to zero before the slide for backward will take effect. 