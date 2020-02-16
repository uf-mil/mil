Header header
int32 channels # Number of channels
int32 samples # Number of samples on each channel (length of data should be sampples * channels)
int32 sample_rate # Number of samples / second recorded by a channel 
#   all channels must use the same sample rate
int16[] data # data relevent to the ping that was triggered
float64 ping_time # the time at which the ping was detected (relative to the center of the sample)
#   ie: ping data = 0.01 of a sampling that was 0.2 seconds in length
#   would imply that the ping happend 0.101 seconds from the start of the message
