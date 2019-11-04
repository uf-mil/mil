# Blue View Driver for ROS
Uses the Telodyne Blue View SDK to produce images / range profiles in ROS

Also provides the bvtsdk as a cmake library for use in other packages.

## Instalation

This package requires the proprietary blueview sdk (bvtsdk) to build.

The cmake expects a bvtsdk directory containing the contents of the tarball
distrbuted with the blueview imaging sonar to be placed at the root of this package.
If it not present, this package will throw a warning and not build the driver.

Please extract ```libbvtsdk_BUILDNUMBER.tar.gz``` to the root of this package.


## Usage
Please see the ```launch/example.launch``` for info on how to configure the driver.

``` rosrun mil_blueview_driver blueview_driver _device:=SONARIPADDRESS ```

## Config

See example launch file at ``` launch/example.launch ``` for full configuration options


