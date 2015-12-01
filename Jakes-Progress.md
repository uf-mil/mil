This is an unlisted wiki page that Jake uses to track his little projects. Uh-oh, how did you find it!?


In order...

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
    - (I am expecting the integration tests here to fail because the controller is not implemented!)

- [x] Add integration tests for the PD Controller
- [x] Submit a pull-request for the PD controller

# Optical Navigation
- [ ] Implement visual odometry
    - [x] Get simple n-point transformation estimation from fundamental matrix working
    - [x] Do it w/ LK Optical Flow
    - [x] Triangulate points in 3D
        - [ ] Visualize estimated camera poses using pyplot cones for debugging (Or use Vispy?)
    - [x] Estimate globally consistent 3d transform by solving PnP
        - [ ] Internal PnP solver (OpenCV's is a disaster)
    - [ ] Record square video
    - [ ] Implement SVO-SLAM
## Problems
    - [?] Dismiss invalid pose transforms
    - [x] Dismiss the bad camera pose estimates from essential matrix decomposition
    - [x] Get motion estimation by PnP to match decomposition of Essential matrix
    - [ ] Recover absolute depth (From either defocus, lens nonlinearities or UKF output)


- [x] Add feature for issuing wrenches via keyboard -- Annie did this
- [x] Make everything work on simulated time so we can run Monte-Carlos in faster than realtime

- [ ] Add controller integration tool
    - [ ] Write a tutorial for adding a simulation widget
        - [ ] And whatever other wacky stuff people want
- [ ] Add vector of gains to demo controller
- [ ] Reclaim Honor

Christmas Break:
    - [ ] Ask Jason to seal the sub before break so we can do a few pool days
    - [ ] MAKE SURE Ralph Tess and self have building access during break (Or we will not be able to do anything)

    Boredom Goals (If critical things get boring, work on these):
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
    - [ ] UKF
    - [ ] Modernize RISE at suggestion of Forrest
        - [ ] Now do a RHC, geometric planner or something wacky and funtacular

Random:
    - [ ] Make increment-numbers again
    - [ ] Some reflection tool for automatically making *stamped messages using __getattr__ and parsing the request
    - [ ] Fix the a=-b corner case for that make_rotation
    - [ ] Make a function for homogeneous quick-inverse (Transpose R, negate t)
    - [ ] Quick-inverse for structured matrices (lapack?!)
        - [Some examples](http://stanford.edu/class/ee364a/lectures/num-lin-alg.pdf)

Not Sub:

    - [ ] Call in to coast-to-coast AM next time up at 3AM (Ralph)
    - [ ] Talk to Paracosm when we have SLAM working