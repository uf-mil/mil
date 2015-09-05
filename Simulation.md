Simulation
==========



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