Header header
int32 channels # Number of channels
int32 samples # Number of samples on each channel (length of data should be sampples * channels)
int32 sample_rate # Number of samples / second recorded by a channel 
#   all channels must use the same sample rate
int16[] data # Example: H0, H1, H2, H3, H0, H1 
