This is an unlisted wiki page that Jake uses to track his little projects. Uh-oh, how did you find it!?


In order...


## Control
- [ ] V & V node for controllers
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
- [ ] Add vector of gains to demo controller

- [ ] Get adaptive controller behaving nicely (The nonlinear project worked ~okay~)
    - [ ] Test w/ Monte-Carlos
    - [ ] Use history correctly

- [ ] Monte-carlo gain optimization (All permutations of some gain sets in some range)
    - For 10 combinations of each p, i and d, this is tenable at 1000 runs, and will take ~2 hrs
        - for 50, we're at 125,000 runs, taking ~230 hours, about 10 days
        - This will work best with some kind of Boltzman annealing by equispaced samples, which will decidedly *not* take over a week
    - [ ] Then do gradient descent near the best N gains
    - [ ] Leave laptop somewhere and run it for 6 hrs
        - or, rent some EC2 time
        - OR just run it on the CUDA computer...overnight
    - A good sub model means we can do this optimization we can do this well...so we need a good model

# Optical Navigation
- [ ] Implement visual odometry
    - [x] Get simple n-point transformation estimation from fundamental matrix working
    - [x] Do it w/ LK Optical Flow
    - [x] Triangulate points in 3D
        - [ ] Visualize estimated camera poses using pyplot cones for debugging (Or use Vispy?)
    - [x] Estimate globally consistent 3d transform by solving PnP
    - [x] Record square video
    - [ ] Outlier detection
        - [ ] Depth estimate filtering
        - [ ] Pose kalman filtering
    - [ ] Implement SVO-SLAM

## Problems
- [?] Dismiss invalid pose transforms
- [x] Dismiss the bad camera pose estimates from essential matrix decomposition
- [x] Get motion estimation by PnP to match decomposition of Essential matrix
    - [ ] Better PnP outlier dismissal
- [ ] Recover absolute depth (From either defocus, lens nonlinearities or UKF output)
    - [ ] UKF + Loop closure: Ceres

# Simulation
- [ ] Submit a PR for the sim format changes
- [x] Add feature for issuing wrenches via keyboard -- Annie did this
- [x] Make everything work on simulated time so we can run Monte-Carlos in faster than realtime
- [ ] Write a tutorial for adding a simulation widget
- [ ] Add simulated magnetometer
    - [ ] Add simulated magnetic field effects to thrusters
- [ ] Simulated stereo depth + imaging sonar
- [ ] Scenegraph system
- [ ] Consistent widgets
    - Scenegraph is a preqrequisite to do this right
    - Do all widgets have pose?
    - Do all widgets have collision (No!)
    - Should all add-remove behavior be abstracted into widgets
    - For you snoopers looking at my page: A widget is a simulation object that has some internal logic that isn't contained strictly within `step physics forward` and `draw me`
- [ ] Figure out the problem with time-acceleration
    - For high simulated time acceleration, controllers have radically different behavior. Why?

# Solvers
- [ ] Get Ceres or IPOPT working (Ceres seems more likely)
    - **Do not** roll our own GN-minimizer
- [ ] Add single-state horizon quick-solver using CVXGEN
- [ ]

# Overall
- [ ] Reclaim Honor

# Christmas Break:
- [ ] Ask Jason to seal the sub before break so we can do a few pool days
- [ ] MAKE SURE Ralph Tess and self have building access during break (Or we will not be able to do anything)

## Boredom Goals (If critical things get boring, work on these):
- Localization visualization tool using Vispy (What about for C++?)
    - Matplotpp
    - Ch++
- Simulation improvements for boat use (Better submerged volume estimation, or constraints objects)
- [ ] Pose-graph/scene-graph system
    - [ ] Widgets for simulation
- [ ] Submit PR for widgets
- [ ] Submit PR for scenegraph

Spring:
- [ ] DVL
- [ ] New IMU
- [ ] Magnetometer & Calibration
- [ ] MHE
- [ ] Modernize RISE at suggestion of Forrest
    - [ ] Now do a RHC, geometric planner or something wacky and funtacular
- [ ] Accurate

Random:
- [ ] Make increment-numbers again
- [ ] Some reflection tool for automatically making *stamped messages using __getattr__ and parsing the request
- [ ] Fix the a=-b corner case for that make_rotation
- [ ] Make a function for homogeneous quick-inverse (R.T, R.T * -t)
- [ ] Quick-inverse for structured matrices (lapack?!)
    - [Some examples](http://stanford.edu/class/ee364a/lectures/num-lin-alg.pdf)

Not Sub:
- [ ] Finish CPY
    - [x] Core generation
    - [ ] Add .cpy support
    - [ ] Finish raw generation
    - [ ] Add pure Python macros
    - [ ] Do what Bjarne talked about - C+Subset where abstractions are excused
    - [ ] Automatically generate headers (LOL DEFEATING THE POINT!)
    - [ ] Watch more Bjarne lectures

- [ ] Call in to coast-to-coast AM next time up at 3AM (Ralph)
- [ ] Talk to Paracosm when we have SLAM working

Knowledge

- [ ] Call in to coast-to-coast AM next time up at 3AM (Ralph)
- [ ] Talk to Paracosm when we have SLAM working
- [x] Give money to Jason -- Friendship submitted in lieu of money
- [Writing good C++](https://youtu.be/0iWb_qi2-uI)
