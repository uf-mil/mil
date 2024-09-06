# Generic representation of measurement from one beam of a Doppler Velocity Log (DVL)-like sensor

# unit vector pointing along beam (in sensor coordinate frame)
geometry_msgs/Vector3 direction

# measured velocity (in m/s) of sensor relative to pool floor, projected onto `direction` vector
# * should be NaN if measurement is bad/invalid/unknown
float64 velocity

# correlation value
# * describes the SNR of the return ping; currently only for diagnostic usage
# * should be NaN if unknown
float64 correlation
