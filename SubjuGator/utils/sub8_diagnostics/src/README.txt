These are basic ROS nodes that publish either to /wrench or to the /moveto/goal topic. 

IMPORTANT: If the launch file publishes to /wrench, remember to comment out the adaptive_controller.launch in sub8.launch. Otherwise, bad data will be obtained. If you are using the launch file that publishes to /moveto/goal, make sure adaptive_controller.launch is not commented out. Otherwise nothing will happen to the sub. This adaptive_controller.launch is the bridge between the /wrench topic and the entire mission system.

We can view and graph topic data by using rqt_plot. Then if you type, for example, /odom/twist/twist/linear/x and add it to the plot, you will see the linear velocity of the sub. This can be helpful in debugging.

I choose not to use the /moveto/goal topic because it does not seem to significantly change the velocity. The acceration aspect of the topic is likely not considered in the code. You can see this for yourself if you run the dynamic_move.launch or dynamic_yaw.launch. Then you can change the accelerations within the launch files, and graph the velocities again.

The nodes we will likely use for the dynamic test are the nodes in the backup directory. They are in this directoy because they were initially my backup options. These nodes publish an increasing acceleration at a certain rate (a constant jerk). The reason I did not publish a constant acceleration is because due to the drag of the system, the sub reaches steady state velocity rather quickly. Publishing a constant jerk allows for a linear increase in the velocity of the sub, which is what andres wants to see. You can see and graph this behavior by running the simulation, and then running either linear_acc.launch or angular_acc.launch
