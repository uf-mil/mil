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
    * [ ] Be able to automatically prove that the robot works in simulation

* [ ] Testing/Modelling
    * [x] Unit-test requirements
    * [ ] Automated NLLS model-fitting for both sim and pool
    * [ ] Automated monte-carlos for both sim and pool


# Bounty problems

If you're looking for something to solve, here's a list of bounty problems. Solving

# Giant Bounty
Prize: We will get Korean Barbeque

* [ ] Get the cameras working (Or get working cameras) -- Ralph
* [ ] Get stereo producing point-clouds -- Gabe and Ralph
* [ ] Get the sub station-holding on all new software (Excluding state-estimation)

# Big Bounty
Prize: Jake will get you ice cream

* [ ] Pure physics simulation (Without Vispy)
    - This is "easy" with the current backend design, but there is a rough edge due to the design of simulate.py
* [ ] Get simulation running on Semaphore Continuous Integration (Ask Jake about the roadblocks here)
* [ ] Get everything working on sim-time or real-time seamlessly
    * [ ] And write a short doc explaining how nodes need to be written to support this
* [ ] Real-time visual odometry -- David
* [ ] Path indicator 3D orientation estimation + caustics -- Nathan
* [ ] Depth estimation from cautics -- Matt
    * [ ] Special bonus for rendering new images without caustics, to do CV on

# Medium Bounty
Prize: Glory, Jake might get you ice cream

* [ ] Come up with a good rostest framework/process for using full simulation
    * [ ] Should we break it up into little bits? Or something
* [ ] Get us an imaging sonar
* [ ] Simulated imaging sonar
* [ ] Scene-graph system for simulator
* [ ] C++ Implementation of alarms
* [ ] Add visualization to the monte-carlo engine
* [ ] Implement simulated integration tests for all systems
* [ ] Add monte-carlo optimization for controller
* [x] Add visualization to the monte-carlo engine
* [ ] Add monte-carlo verification
    * [ ] Controller
    * [ ] Vision
    * [ ] Motion planning
    * [ ] Alarms
    * The sub should be able to handle *almost* completely random conditions (With some constraints on orientation and systems functional)
* [ ] Figure out automatic differentiation
    * [ ] Or Make a jacobian-generator for the sub's dynamics
* [ ] A method for calibrating imaging sonar (Make a huge chessboard and cut out the white parts)
* [ ] Implement alarms for existing nodes
    * [ ] Alarm for bus undervoltage
    * [ ] Alarm for thruster temperature
    * [ ] Alarm and response for DVL track lost (Should we even do anything?)
* [ ] Unit-tests for all of the tools that are not unit-tested
* [ ] Add an install_dependencies.sh file to the repo, and make SemaphoreCI automatically run it

# Small Bounty
Prize: Glory

* [ ] Easier `rostopic pub` tool that doesn't require you to specify message type!
* [ ] Easy "make_*stamped" message helper function (or wrappers around the uglier parts of Rospy, like 6 lines to send a trivial WrenchStamped)
