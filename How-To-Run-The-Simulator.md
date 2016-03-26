# **How To Run The Simulator**

Open a new terminal and run:

    roslaunch navigator_launch simulation.launch

This file will start simulation as well as the gnc.launch file

Right now the simulator is only able to simulate components of GNC. Perception is to come!

Open a new terminal and run:

    rqt

Then navigate to the NaviGator GUI.  

## How To Use The GUI

Kill and Revive do not work at the moment, waiting for the Sub8 alarm system to be updated. 

**The boat starts in a killed state so until a control mode (rc, autonomous, or gui) is chosen the boat remains killed. So choose which controller you want and go from there.** 

The three modes are the the three bottom buttons. RC sets it to be controlled by the Xbox controller. AU sets it to be controlled by the autonomous controller.  GUI sets it to be controlled by the sliders on the right

If in 'gui' control mode the sliders on the right are active and can be used to give the boat a movement command. Only one can be used at once though. If moving forward and wanting to move back, the slider for forward must be set to zero before the slide for backward will take effect. 