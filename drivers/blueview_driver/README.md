# Blue View Driver for ROS
Uses the Telodyne Blue View SDK to produce images / range profiles in ROS

## Usage

Currently this ROS driver depends on MIL's catkin-based version of the blue view SDK.
If you are interested in using this outside of MIL, you must ensure this library is
linked with the BlueView SDK and has the BlueView SDK include folder.

``` rosrun blue_view_driver blue_view_driver _device:=SONARIPADDRESS ```

## Config

See example launch file at ``` launch/example.launch ``` for full configuration options


