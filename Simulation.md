Simulation
==========

## Why are we creating our own simulation?
Why not use V-REP or Gazebo?

At the time of writing, neither was really designed for water vehicles. We determined that it would take as much time to configure one of those simulators as it would to write our own. Ours would exactly satisfy our needs, while specifically targeting all of the MIL vehicles.

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