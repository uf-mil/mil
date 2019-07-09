Header header # frame_id is point measurement was taken at

uint32 sync_WN
float64 sync # value that can be used to approximately synchronize multiple receivers
             # GPS time of week (seconds modulo gps.week_length = 7*24*60*60)

Satellite[] satellites

geometry_msgs/Point position # approximate position of receiver
bool position_valid
