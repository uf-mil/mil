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
        - [ ] Size of convergence ball
    - [x] Visualize many trajectories
- [x] Get sim to run faster than real-time
- [x] Submit a pull-request for the 3D Mouse
- [x] Add headless simulation so we can run on Semaphore
    - [x] OR! Figure out how to run vispy headless on Semaphore
- [ ] Add integration tests for the PD Controller
- [ ] Submit a pull-request for the PD controller

- [ ] Focus on vision (Implement visual odometry in some meaningful way - we need non-shitty cameras)
- [x] Make everything work on simulated time so we can run Monte-Carlos in faster than realtime
- [ ] Add controller integration tool for Jason
    - [ ] Write a tutorial for adding a simulation
- [ ] Add vector of gains to controller

Random:
    - [ ] Make increment-numbers again
    - [ ] Some reflection tool for automatically making *stamped messages using __getattr__ and parsing the request