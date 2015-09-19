Simul8
======

Simul8 is our internally implemented sub dynamics and vision simulation.

# Implement
* [ ] Rendering
    * [x] View control
    * [x] Diffuse lighting (Blinn-Phong)
    * [x] Multi-shader files
    * [ ] Water shading
    * [x] Multi-object rendering
        * [x] Unified Entity model
    * [x] Easy-to-use rendering interface
    * [ ] ROS-Cameras
    * [ ] Simulated depth imaging
    * [ ] Assemblies/SceneGraphs (Does ODE do this?)
    * Stretch Goals
        * [ ] Shadows
        * [ ] Multiple light sources

* [ ] Physics
    * [ ] pyODE driven dynamics
    * [ ] Configurable thruster positions
    * [ ] Simple object interface
    * [ ] Collision
    * [ ] Grasping

* [ ] Sensing
    * [ ] Depth Sensor
    * [ ] DVL
    * [ ] Imaging Sonar (OpenGL)
    * [ ] Stereo depth estimation (OpenGL)

* [ ] Testing
    * [ ] Simulated alarm scenarios
        * [ ] Thruster out, both detected and undetected
        * [ ] Sudden obstacle detection
        * [ ] Battery low
        * [ ] Critical navigation sensor out (Is this necessary?)
        * [ ] Various mission failure scenarios