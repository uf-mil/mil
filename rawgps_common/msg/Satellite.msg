int32 prn

float64 cn0

# ECEF position and velocity of the satellite and time at the moment the received signal was sent
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64 time

float64 carrier_distance # m with (half-?)integer multiple of wavelength bias
float64 doppler_velocity # m/s with bias common to all sibling Satellites

# legacy
geometry_msgs/Vector3 direction_enu
float64 velocity_plus_drift
