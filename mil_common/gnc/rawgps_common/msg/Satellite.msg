int32 prn

float64 cn0

geometry_msgs/Point position        # ECEF position of the satellite at the
                                    # moment the received signal was sent
geometry_msgs/Vector3 velocity      # ECEF velocity of the satellite at the
                                    # moment the received signal was sent
float64 time                        # time at which signal was transmitted
                                    # from satellite, with bias common to all
                                    # satellites

float64 T_iono                      # ionospheric delay (s), sometimes nan
float64 T_tropo                     # tropospheric delay (s)

float64 carrier_distance            # m with (half-?)integer multiple of
                                    # wavelength bias,
                                    # and with bias common to all satellites,
                                    # sometimes nan
float64 doppler_velocity            # m/s with bias common to all satellites,
                                    # sometimes nan

# legacy data, now only useful for humans looking at topic
geometry_msgs/Vector3 direction_enu
float64 velocity_plus_drift         # receiver's velocity in direction_enu,
                                    # with bias common to all satellites,
                                    # sometimes nan
