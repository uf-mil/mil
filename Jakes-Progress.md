This is an unlisted wiki page that Jake uses to track his little projects. Uh-oh, how did you find it!?


In order...


## Control
- [x] V & V node for controllers
    - [x] pyplot 3d trajectory visualization
    - [ ] Randomized orientation
        - [ ] Visualize orientation progress
    - [x] Visualize velocity vectors (Tangents are visible, don't bother)
        - [ ] maybe color magnitude?
    - [ ] Actual quality assessment
        - [ ] Time-to-stable
        - [ ] Length of trajectory
        - [x] Size of convergence ball
    - [x] Visualize many trajectories
- [x] Get sim to run faster than real-time
- [x] Submit a pull-request for the 3D Mouse
- [x] Add headless simulation so we can run on Semaphore
    - [x] OR! Figure out how to run vispy headless on Semaphore

- [x] Add controller-test montecarlos
    - [x] Submit a pull-request for controller-test montecarlos

- [x] Add integration tests for the PD Controller
- [x] Submit a pull-request for the PD controller
- [x] Add vector of gains to demo controller

- [ ] Submit PR for Controller base-class
- [x] Get adaptive controller behaving nicely (The nonlinear project worked ~okay~)
    - [x] Test w/ Monte-Carlos
    - [x] Use history correctly
- [ ] Submit PR for adaptive controller

- [ ] Monte-carlo gain optimization (All permutations of some gain sets in some range)
    - For 10 combinations of each p, i and d, this is tenable at 1000 runs, and will take ~2 hrs
        - for 50, we're at 125,000 runs, taking ~230 hours, about 10 days
        - This will work best with some kind of Boltzman annealing by equispaced samples, which will decidedly *not* take over a week
    - [ ] Then do gradient descent near the best N gains
    - [ ] Leave laptop somewhere and run it for 6 hrs
        - [x] Make EC2 ROS snapshot (Do some crazy spot-instance shenanigans)
        - or, rent some EC2 time
        - OR just run it on the CUDA computer...overnight
    - A good sub model means we can do this optimization we can do this well...so we need a good model

- [ ] Multiscale direct collocation
    - [ ] RHC with many guesses using OMP
    - [ ] Generate time-optimal trajectories using Eric's method
        - [ ] GN-solver; no dynamic constraints?
    - [ ] Get PSOPT working
    - [x] get SNOPT (They will call me "Mount Cleverest")

- [ ] LQR
    - [ ] Translation only LQR
        - [x] Simple demo
        - [ ] Lyapunov
        - [ ] Robust
    - [ ] Attitude LQR [iffy](http://automatica.dei.unipd.it/tl_files/events/20100907AlessandroSacconSeminario.pdf)
    - [ ] Submit PR

# Optical Navigation
- [ ] Implement visual odometry
    - [x] Get simple n-point transformation estimation from fundamental matrix working
    - [x] Do it w/ LK Optical Flow
    - [x] Triangulate points in 3D
        - [x] Visualize estimated camera poses using pyplot cones for debugging (Or use Vispy?)
    - [x] Estimate globally consistent 3d transform by solving PnP
    - [x] Record square video
    - [ ] Outlier detection
        - [ ] Depth estimate filtering
        - [ ] Pose kalman filtering
    - [ ] Implement SVO-SLAM
- [ ] Bayesian outlier detection
    - [x] Dense histogram method
    - [ ] OpenCL implementation (I could use ArrayFire and not do it directly for the Sub...)
    - [ ] Alpha-Beta method
- [ ] Make everybody build PCL from source
    - We get C++11
        - We can use PCL visualization with sub8_slam
        - We can use nice C++11 features (Hey-o move semantics!) with

## Problems
- [?] Dismiss invalid pose transforms
- [x] Dismiss the bad camera pose estimates from essential matrix decomposition
- [x] Get motion estimation by PnP to match decomposition of Essential matrix
    - [ ] Better PnP outlier dismissal
- [ ] Recover absolute depth (From either defocus, lens nonlinearities or UKF output)
    - [ ] UKF + Loop closure: Ceres

# Simulation
- [x] Submit a PR for the sim format changes
- [x] Add feature for issuing wrenches via keyboard -- Annie did this
- [x] Make everything work on simulated time so we can run Monte-Carlos in faster than realtime
- [x] Simulated IMU -- Annie
- [x] Simulated DVL -- Annie
- [ ] Add simulated magnetometer
    - [ ] Add simulated magnetic field effects to thrusters
- [ ] Simulated stereo depth + imaging sonar
- [ ] Scenegraph system
- [ ] Consistent widgets
    - Scenegraph is a preqrequisite to do this right
    - Do all widgets have pose?
    - Do all widgets have collision (No!)x
    - Should all add-remove behavior be abstracted into widgets
    - For you snoopers looking at my page: A widget is a simulation object that has some internal logic that isn't contained strictly within `step physics forward` and `draw me`
- [x] Figure out the problem with time-acceleration
    - For high simulated time acceleration, controllers have radically different behavior. Why?
        - This may have been an aliasing effect from sampling rate
- [x] The sim gets slower over time (Is it accumulating garbage?)
    - Not getting slower, just responding more intermittently to keyboard input
- [ ] Verification
    - [ ] IMU
    - [ ] DVL
    - [ ] Magnetomter
- [x] Submit final PR for monte-carlo branch
- [ ] Verify sub orientation
    - [x] Figure out source of inversion in Odometry (Stray transpose?)
- [ ] Add simulated success conditions
- [ ] Add .OBJ color reader (and Phong parameter reader)
- [x] Learn Tess' shader swapping
    - [ ] Add simulated depth imaging
- [ ] Add view swapping
    - [ ] Add simulated ROS cameras (Set camera parameters?)
- [ ] "Regression testing" to force the simulator to match reality (Here, a learned dynamic model is guaranteed to be useful)

# Solvers
- [x] Get Ceres or IPOPT working (Ceres seems more likely)
- [ ] Inverse-compositional GN solver
- [ ] Add single-state horizon quick-solver using CVXGEN
- [ ] Multi-scale quick-DirCol
    - [ ] cvx and subdivide, abuse Jacobian
    - [ ] For subdivides, constrain end collocation point to next subdivision start collocation
        - [ ] Otherwise do unconstrained optimization
- [ ] Finish quick-Ricatti stuff

# Overall
- [ ] Reclaim Honor

# Christmas Break:
- [x] Ask Jason to seal the sub before break so we can do a few pool days
- [x] MAKE SURE Ralph Tess and self have building access during break (Or we will not be able to do anything)
- [x] Get DVL working
- [x] Run real thrusters
- [ ] Do LIDAR SLAM w/ Zach's data
- [ ] Finish SVO
- [x] Fix simulator bugs
- [ ] Finish Jayne PTLS


## Boredom Goals (If critical things get boring, work on these):
- Localization visualization tool using Vispy (What about for C++?)
    - Matplotpp
    - Ch++
- Simulation improvements for boat use (Better submerged volume estimation, or constraints objects)
- [ ] Pose-graph/scene-graph system
    - [ ] Widgets for simulation
- [x] Submit PR for widgets
- [ ] Submit PR for scenegraph

## Alarms
    - [x] Add simple GUI (While waiting for mission PR)
        - [ ] Full sub command gui
        - [ ] Make the GUI take alarms of the same name/id and overwrite them (So we don't get spammed)
    - [ ] Improve everything
    - [x] Fix thrusters

## Missions
    - [x] Get TxROS back in the working tree
    - [ ] Add buoy-bumping mission
    - [ ] Write guide alignment mission


## Perception
    - [x] Buoys
        - [ ] K-means optimal threshold
        - [x] Visualize in RVIZ
        - [ ] Noise rejection, bad cloud rejection
            - [ ] Particle filter
            - [ ] SBA for a much later date
        - [x] Make "find-buoy" service (Or action? Not important! )

    - [ ] Guides
        - [x] K-Means optimal thresholding
        - [x] PCA for principal orientation detection
        - [ ] RANSAC for parallelogram fitting (From above, it's approx a parallelogram)
            -- Really, its a trapezoid
        - [ ] Implement service (Or action!?)
        - [ ] Visualize in RVIZ

    - [ ] "Bag-Nell", a tool for manually sifting through bags and selecting regions, then generating thresholds as parameter .yamls, and storing those
        - [ ] We'd have "forrest_pool_cals.yaml", "easterling_pool_cals.yaml" and eventually "transdec_cals.yaml"
    - [ ] "Occupancy" grid for "explored floor" - simple, 2d, using camera projection + TF of downward cam
        -- Mostly useful for human visualization
            -- Maybe someday we can use it as a search directive
        - [ ] Project down camera (Need some tf monaaay)
        - [ ] Visualize in RVIZ
        - [ ] Restart when we restart sub
        - [ ] Use DVL depth
            - [ ] Visualize (Check orientation of DVL frame, this might be easy): Nope, not easy
    - [ ] Script for auto-testing perception performance on manually segmented bags

Spring:
- [ ] Ready up for pool-day (Make DVL, etc nodes run w/o using kill system)
    - [ ] Perhaps switch them over to alarms?
    - [ ] Reimplement all this nonsense in Python
- [x] DVL
- [x] Depth
- [x] C3
- [x] IMU
- [x] PR for drivers
- [ ] New IMU
- [x] Magnetometer & Calibration
- [ ] MHE
- [ ] Modernize RISE at suggestion of Forrest
    - [ ] Now do a RHC, geometric planner or something wacky and funtacular
    - [ ] Lyapunov-Robust LQR
- [ ] Simple multi-hypothesis tracking toolbox in both 2D and 3D

Random:
- [ ] Pack-map: Draw the hierarchies of the sub packages
- [ ] Some reflection tool for automatically making *stamped messages using __getattr__ and parsing the request
 the a=-b corner case for that make_rotation
- [ ] Make a function for homogeneous quick-inverse (R.T, R.T * -t)
- [ ] Quick-inverse for structured matrices (lapack?!)
    - [Some examples](http://stanford.edu/class/ee364a/lectures/num-lin-alg.pdf)


POOL
    - [x] Fix thrusters (Each should be its own node, we are getting rate issues)
    - [ ] Switch to Twisted (Or figure out a way to handle over-messaging)
    - [ ] Better polling process
    - [x] Tool for more easily specifying waypoints (Use 3D mouse?)
        - [x] Fix weird rotation bugs
        - [ ] Add yaw only mode (Not necessary now that it's easier to use)
    - [x] Visualize height estimated by DVL
        - [x] Adding "range" messages for visualization creates pointless TF overhead
            - [x] Instead, write an rviz-republisher that will send markers for depth and height
    - [x] Visualize depth estimate

    - [x] RISE can begin to command infinite thrusts after error too long, and the thruster mapper will start to fail to map
    - [x] Get RISE working in simulation
        --> Come up with safe way to avoid failure feedback loop
    - [ ] Script for uploading bags

- [ ] Schedule all hands meeting (inc electrical, mechanical)

Not Sub:
- [ ] Finish CPY
    - [x] Core generation
    - [ ] Add .cpy support
    - [ ] Finish raw generation
    - [ ] Add pure Python macros
    - [ ] Do what Bjarne talked about - C+Subset where abstractions are excused
    - [ ] Automatically generate headers (LOL DEFEATING THE POINT!)
    - [ ] Watch more Bjarne lectures
- [ ] Move sub simulator to Gazebo (It's time, Gazebo is ready)

- [ ] Call in to coast-to-coast AM next time up at 3AM (Ralph)
- [ ] Talk to Paracosm when we have SLAM working

Sublime plugins
- [x] Make increment-numbers again
- [ ] Easy-GFM-Tables


Knowledge
- [ ] Learn how to use [Jekyll](http://jekyllrb.com/docs/posts/) for github page
- [ ] Call in to coast-to-coast AM next time up at 3AM (Ralph)
- [ ] Talk to Paracosm when we have SLAM working
- [x] Give money to Jason -- Friendship submitted in lieu of money
- [Writing good C++](https://youtu.be/0iWb_qi2-uI)
- [SID](http://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-435-system-identification-spring-2005/lecture-notes/)


Notes
- Downward camera is usb 3.0