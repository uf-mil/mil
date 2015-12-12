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

If you're looking for something to solve, here's a list of bounty problems. Giant bounty problems are determined not by their difficulty, but how important they are to the sub right now.

# Giant Bounty
Prize: We will get Korean Barbeque

* [ ] Get the cameras working (Or get working cameras) -- Ralph
* [ ] Get stereo producing point-clouds -- Gabe and Ralph
* [ ] Get the sub station-holding on all new software (Excluding state-estimation)

# Big Bounty
Prize: Jake will get you ice cream

* [x] Pure physics simulation (Without Vispy)
    - This is "easy" with the current backend design, but there is a rough edge due to the design of simulate.py
* [x] Get simulation running on Semaphore Continuous Integration (Ask Jake about the roadblocks here)
* [x] Get everything working on sim-time or real-time seamlessly
    * [x] And write a short doc explaining how nodes need to be written to support this
* [ ] Real-time visual odometry -- David
* [ ] Path indicator 3D orientation estimation + caustics -- Nathan
* [ ] Depth estimation from caustics -- Matt
    * [ ] Special bonus for rendering new images without caustics, to do CV on
* [ ] Get us an imaging sonar
* [ ] 3D Pose estimation for the vehicle
* [ ] Task recognition + 3d pose estimation
* [ ] Generate a traversability map

# Medium Bounty
Prize: Glory, Jake might get you ice cream

* [ ] Mission scheduler
    * [ ] Handle concurrency somehow
* [ ] Come up with a good rostest framework/process for using full simulation
    * [ ] Should we break it up into little bits? Or something
* [x] C++ Implementation of alarms -- Patrick
* [x] Add visualization to the monte-carlo engine
* [ ] Implement simulated integration tests for all systems
* [ ] Add monte-carlo optimization for controller
* [x] Add visualization to the monte-carlo engine
* [ ] Add monte-carlo verification (And many monte-carlo test cases)
    * [x] Controller
    * [ ] Vision
    * [ ] Motion planning
    * [ ] Alarms
    * The sub should be able to handle *almost* completely random conditions (With some constraints on orientation and systems functional)
* [x] Figure out automatic differentiation
    * [ ] Or Make a jacobian-generator for the sub's dynamics
* [ ] Implement alarms for existing nodes
    * [ ] Alarm for bus undervoltage
    * [ ] Alarm for thruster temperature
    * [ ] Alarm and response for DVL track lost (Should we even do anything?)
* [ ] Unit-tests for all of the tools that are not yet unit-tested
* [ ] Add an install_dependencies.sh file to the repo, and make SemaphoreCI automatically run it
* [ ] A script to run on Semaphore to compare performance metrics for current build vs pull
    i.e. if build[n]'s controller and vision performed better than build[n + 1], then something may have gone wrong, and we should be cautious pulling those changes

## Simulation
* [ ] Add trajectory visualization to the simulator
    * When a trajectory message is published, draw a bunch of spheres or something
* [ ] Add "widgets" to the simulator
* [ ] Simulated shadows
* [ ] CUDA water physics for Simul8
    * [ ] Or better water resistance at all
* [x] Multiple light sources -- Tess
* [ ] Keyboard control of Sub (Requested by Zach and Patrick) -- Annie
* [ ] Scene-graph system for simulator

## Machine Learning
* [ ] Toolset for object recognition
* [ ] Fit a dynamic model for the sub (TDQ learning?)
* [ ] Automatic controller optimization (Once we have a good model for the sub)

## Perception
* [ ] Simulated cameras
* [ ] Simulated imaging sonar
* [ ] Monte-carlos for perception testing
* [ ] A method for calibrating imaging sonar (Make a huge chessboard and cut out the white parts)
* [ ] Build an apparatus of LED's for generating random patterns, to create artificial stereo disparity underwater
* [ ] Get ArrayFire working using OpenCL to do 3D pose estimation
* [ ] Diver pose and gesture recognition
* [ ] Handle moving objects in whatever SLAM process we use
    * [ ] Also handle variation in surface appearance due to sand moving
* [ ] Predict the world-frame trajectories of moving objects
    * Docking with the moving WAM-V
    * Water-stunts, like grabbing a falling ring
* [ ] Statistical noise rejection for sonar and stereo data
* [ ] Implement PCL functions in OpenCL and/or CUDA
* [ ] Implement point-cloud research using PCL (Ask Jake about this - there are many many great PCL results that do not publish source code)
* [ ] Come up with a way to use LED's or AR-tags to determine the pose of the camera relative to some landmark, e.g. the boat's docking adapter
* [ ] Add stuff to the setup.py for sub8_command to automatically set up bash aliases for kill/unkill, switch rosmaster, etc

# Small Bounty
Prize: Glory

* [ ] Easier `rostopic pub` tool that doesn't require you to specify message type!
* [ ] Easy "make_*stamped" message helper function (or wrappers around the uglier parts of Rospy, like 6 lines to send a trivial WrenchStamped)


This is not a complete list. It should be made especially clear: If you think of a really cool project you'd like to do, you absolutely have free reign to do so.


# If we raise more money

* [ ] 7-DOF underwater manipulator
* [ ] Multiple imaging sonars
* [ ] GPU Vessel (Or better solution, for getting CUDA into the sub)
