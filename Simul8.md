Simul8
======

Simul8 is our internally implemented sub dynamics and vision simulation.

# How to install

    mkdir repos
    cd repos
    git clone https://github.com/vispy/vispy.git
    cd vispy
    git checkout b48e4d3cf410b853a74b666c475c603e46725e55
    sudo python setup.py develop

# Install ODE
    sudo apt-get install python-ode
    rm -fr /tmp/pyode-build && mkdir -p /tmp/pyode-build && cd /tmp/pyode-build && sudo apt-get build-dep -y python-pyode && sudo apt-get remove -y python-pyode && apt-get source --compile python-pyode && sudo dpkg -i python-pyode_*.deb
   # Credit to Forrest for this - the Pyode on apt doesn't behave, but compiling from source works


# How do I run it?

Run...

    roslaunch sub8_simulation sim_full.launch

Which will run all of the necessary nodes to run simulations.

# How does it work?
Simul8 has two major submodules: Physics and Rendering.

## Physics
The physics module is driven by pyODE (Python bindings to the Open Dynamics Engine). We define our own "World" object that manages all of the geometries and bodies that exist in the physics universe.

### Physics Entity
An entity is an object with physics. Currently implemented are Spheres, Boxes, and Sub8 approximated as a box. We will soon add arbitrary meshes.

Each entity implements a "step" method that is called for each physics step. They must also implement their own means for applying damping and buoyancy (Right now, everything uses a spherical submerged volume approximation, but you can implement your own)

## Rendering
The rendering module is driven by Vispy, powerful Python bindings to OpenGL. It also has a "world" concept, which manages all of the lighting, cameras, and objects for rendering.

### Rendering Entity
A rendering entity is an object to be drawn. It must have vertices, normals and faces defined. Each entity is drawn once for each view/imaging sensor. In the future, they may be drawn in multiple passes.

Each entity implements a "draw" method that is called each time it must be drawn.

An Entity can optionally implement texcoords for texturing.

Currently implemented are spheres, planes and boxes. We will soon add arbitrary meshes.

### Lighting
Implemented

* Blinn-Phong
* Lambertian

Todo

* Oren-Nayar?
* Shadows

# Todo

## Document
* [x] Installation
* [x] How to run
* [ ] How to expand

## Implement
* [ ] Scene
    * [x] Transdec model
    * [x] Sub8 Model
    * [ ] AUVSI models

* [ ] Rendering
    * [x] View control
    * [x] Diffuse lighting (Blinn-Phong)
    * [x] Multi-shader files
    * [x] Transparent water
    * [x] Multi-object rendering
        * [x] Unified Entity model
    * [x] Easy-to-use rendering interface
    * [ ] ROS-Cameras
    * [ ] Lock camera to sub
    * [ ] Simulated depth imaging
    * [ ] Assemblies/SceneGraphs (Does ODE do this?)
    * [x] Simple .stl or .obj loading
        * [ ] .obj loading WITH textures -- Important!
    * [ ] Simulated caustics -- Important! (Use a randomized texture, or actually draw)
    * Stretch Goals
        * [ ] Shadows
        * [ ] Multiple light sources

* [ ] Physics
    * [x] pyODE driven dynamics
    * [x] Thruster simulation
        * [x] Configurable thruster positions
    * [x] Simple object interface
    * [x] Collision
    * [x] Buoyancy simulation
    * [ ] Grasping

* [ ] Sensing
    * [ ] Depth Sensor
    * [ ] DVL (We need to simulate all 4 beams if we want to test flipping)
    * [ ] IMU (This is conceptually simple, but will take a good bit of work)
    * [ ] Imaging Sonar (OpenGL)
    * [ ] Stereo depth estimation (OpenGL)
    * [ ] Visible-light camera (OpenGL)

* [ ] Testing
    * [ ] Simulated alarm scenarios
        * [ ] Thruster out, both detected and undetected
        * [ ] Sudden obstacle detection
        * [ ] Battery low
        * [ ] Critical navigation sensor out (Is this necessary?)
        * [ ] Various mission failure scenarios
        * [ ] Controller: Target state cannot be made stable

    * [ ] Headless simulation (No rendering) - This is important!
        * We can optimize controller gains, path planners, do monte-carlos, etc

* [ ] Add a way to visually specify a waypoint (Like, move a ghost of the sub around then publish that waypoint)
* [ ] Make the sim a sub-window of a QT gui

# Bug-hunt

There are a few bugs in the simulator that should be chased down, but are not breaking.

* The velocity and force indicators point in the wrong direction sometimes (But not always...?)
* A light source has to be the last thing added
    * In addition, we only support one light-source, any ideas on fixing this?
