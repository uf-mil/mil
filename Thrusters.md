Thrusters
=========

We have 8 VideoRay M5 Thrusters on Subjugator 8. These are the primary mode of actuation for vehicle motion, other than throwing.

# TODO

* [ ] Implement numerical thrust allocation
    * [x] Determine thruster layout configuration file format
    * [ ] Construct control-input matrix from configuration
    * [ ] Handle thruster-out events
* [ ] Implement thruster communication protocol
    * [ ] Implement simulated thruster output
* [ ] Implement ROS thruster interface
    * [ ] Handle thruster Newtons -> Thruster input calibration
    * [ ] Implement srv/msg types
        * Thrusters should communicate their capabilities
    * [ ] Implement alarms
        * [ ] Implement thruster-out handling


# References

[1] Goldstein, Andy; [VideoRay m5 git repository](https://github.com/videoray/Thruster)

[2] Voight, Forrest; [Old Implementation](https://github.com/uf-mil/software-common/blob/master/videoray_m5_thruster_driver/scripts/videoray_m5_thruster_driver)