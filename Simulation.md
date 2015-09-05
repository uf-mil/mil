Simulation
==========

# Tools

### V-REP
- Very fully featured
- Has in-built velodyne support
- Hard to configure
- No built-in water resistance

### Gazebo
- Bloated
- Hard to configure
- No built-in water resistance

### UWSim
- Already has DVL, imaging sonar, depth sensor support
- Water-resistance support
- Surface wave support
- Installs as an apt package
- This...might be perfect...

### Implement our own
- Will take time
- Have to support internally
- Will do everything we want exactly the way we want it

# Considerations

- Collision
- Nonlinear dynamics (Can't just use state-transition matrices)
- Arbitrary rotations
- Water resistance
- Camera view simulation (Grab color framebuffer)
- Depth-imaging simulation (Grab depth buffer)

## Non-simulation
When developing simulation visualization, we should do so with the intention of reusing it for visualizing real data.

- Current path estimates
- Thruster efforts
- etc