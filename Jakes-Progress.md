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
- [ ] Submit a pull-request for the PD controller

- [ ] Focus on vision (Implement visual odometry in some meaningful way - we need non-shitty cameras)
    - [x] Get simple n-point transformation estimation from fundamental matrix working
    - [ ] Do it w/ LK Optical Flow
    - [ ] Implement SVO-SLAM

- [x] Make everything work on simulated time so we can run Monte-Carlos in faster than realtime
- [ ] Add controller integration tool for Jason
    - [ ] Write a tutorial for adding a simulation
- [ ] Add vector of gains to controller
- [ ] Reclaim Honor

Random:
    - [ ] Make increment-numbers again
    - [ ] Some reflection tool for automatically making *stamped messages using __getattr__ and parsing the request
    - [ ] Fix the a=-b corner case for that make_rotation
    - [ ] Make a function for homogeneous quick-inverse (Transpose R, negate t)
    - [ ] Quick-inverse for structured matrices (lapack?!)
        - [Some examples](http://stanford.edu/class/ee364a/lectures/num-lin-alg.pdf)

Not Sub:

    - [ ] Study PoEE
    - [ ] Call in to coast-to-coast AM next time up at 3AM (Ralph)