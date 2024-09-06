# Generic representation of one measurement report from a Doppler Velocity Log (DVL)-like sensor

# stamp is the time the measurements were made
# frame_id ...
# * should have its origin at the location where the beam vectors intersect
# * defines the coordinate frame of the `velocity_measurements.direction` vectors
Header header

# velocity_measurements should have one element for each beam the hardware supports (typically 4)
# * invalid beams should still have VelocityMeasurement elements
VelocityMeasurement[] velocity_measurements
