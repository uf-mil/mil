* Alarm system - Each node publishes alarms on a single topic (a'la tf), and the system reports (SysRep) node determines the appropriate action
* Better simulation - The ability to run simulated monte-carlo tests, simulate depth images, simulate water and wind resistance accounting for sub geometry
* Automatic controller/motion planner quality assessment via unit-test (For both real world and simulation)

* Get an [imaging sonar](http://www.humminbird.com/Category/Technology/Down-Imaging/), there are some very reasonably priced imaging sonars available from Hummingbird, we need to figure out sponsorship to get ahold of one.



* [ ] Simulation
    * [x] Visualization
    * [ ] Monte-Carlos
    * [ ] Sensor simulation

* [ ] State estimation
    * [ ] SLAM
    * [ ] Forrest's new code-generation tool

* [ ] Control
    * [x] Energy-optimal thrust allocation
    * [ ] LQR
    * [ ] MPC
    * [ ] Something adaptive?
    * [ ] Automatic controller quality assessment via simulation

* [ ] Perception
    * [ ] Imaging sonar
    * [ ] SLAM
    * [ ] Real-time visual odometry

* [ ] Fault-tolerance
    * [ ] Alarm system
    * [ ] Handle loss of DVL track
    * [ ] Navigate using optical flow

* [ ] Testing/Modelling
    * [x] Unit-test requirements
    * [ ] Automated NLLS model-fitting for both sim and pool
