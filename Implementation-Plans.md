* Alarm system - Each node publishes alarms on a single topic (a'la tf), and the system reports (SysRep) node determines the appropriate action
* Better simulation - The ability to run simulated monte-carlo tests, simulate depth images, simulate water and wind resistance accounting for sub geometry
* Automatic controller/motion planner quality assessment via unit-test (For both real world and simulation)

* Get an [imaging sonar](http://www.humminbird.com/Category/Technology/Down-Imaging/), there are some very reasonably priced imaging sonars available from Hummingbird, we need to figure out sponsorship to get ahold of one.

* A node that makes "beeping sounds" when bus voltage is low, etc

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
    * [x] Alarm system
    * [ ] Handle loss of DVL track
    * [ ] Navigate using optical flow

* [ ] Testing/Modelling
    * [x] Unit-test requirements
    * [ ] Automated NLLS model-fitting for both sim and pool


# Bounty problems

If you're looking for something to solve, here's a list of bounty problems. Solving

    * [ ] Simulated imaging sonar
    * [ ] Scene-graph system for simulator
    * [ ] C++ Implementation of alarms
    * [ ] Easier `rostopic pub` tool that doesn't require you to specify message type!
    * [ ] Add visualization to the monte-carlo engine
    * [ ] Add monte-carlo optimization for controller
    * [ ] Figure out automatic differentiation
        * [ ] Or Make a jacobian-generator for the sub's dynamics!
    * [ ] Real-time visual odometry
    * [ ] Get us an imaging sonar
    * [ ] A method for calibrating imaging sonar
    * [ ] Implement alarms for existing nodes
        * [ ] Alarm for bus undervoltage
        * [ ] Alarm for thruster temperature
    * [ ] Implement simulated integration tests for all systems
    * [ ] Add an install_dependencies.sh file to the repo, and make SemaphoreCI automatically run it