# frame_id of sonar, time when ping generated
Header header     

# Note: elements in corresponding indicies of bearings, ranges, and intensities describe the same hit
# len(bearings) == len(ranges) == len(intensities)

# raw bearings data from ping, in degrees

# Bearing of hits in sonar frame, in degrees, where positive is clockwise looking down at head
float32[] bearings

# Distance to hit in meters
float32[] ranges

# Intensity values, where max(uint16) is 100%
uint16[] intensities
