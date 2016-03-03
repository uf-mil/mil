Change history
==============

1.10.0 (2014-09-01)
-------------------

 * Initial Indigo release.
 * Add DC1394 trigger support (`#9`_), thanks to Boris Gromov.
 * Add services to get and set camera-specific registers (`#32`_),
   thanks to Tomas Petricek.

1.9.5 (2014-04-29)
------------------

 * Hydro release update.
 * Fix problems with catkin build exports (`#33`_).
 * Make number of DMA buffers a parameter (`#29`_), thanks to Tomas
   Petricek.
 * Turn power on for a feature before setting its operating mode
   (`#15`_), thanks to Brice Rebsamen.

1.9.4 (2013-08-03)
------------------

 * Add ``time_offset`` parameter (`#11`_), thanks to David Gossow.
 * Install ``pluginlib`` XML file (`#10`_).
 * Enable unit tests when ``CATKIN_ENABLE_TESTING`` set.
 * Enable tests requiring a real camera when ``USE_DEVICE`` set.

1.9.3 (2013-06-05)
------------------

 * Add Pan Control feature (`#7`_), works with Point Grey Ladybug 3.
   Thanks to Josep Bosch, Universitat de Girona.

1.9.2 (2013-05-13)
------------------

 * Add ``catkin_package()`` call, needed to install ``package.xml`` (`#6`_).
 * Clean up build to better comply with recommended catkin practices.

1.9.1 (2013-04-14)
------------------

 * Remove roslib import from dynamic reconfigure script.

1.9.0 (2013-04-12)
------------------

 * ROS Hydro.
 * Move official source to https://github.com/ros-drivers/camera1394
 * Convert to catkin build system (`#1`_).

1.8.1 (2013-03-01)
------------------

 * Fix rostest dependency problem with Groovy build.

1.8.0 (2012-03-14)
------------------

 * ROS Fuerte, later released to Groovy.
 * Power off the camera on shutdown (`#5322`_).
 * Provide leading zeros to GUID when needed (`#5350`_).
 * Add diagnostics report on actual camera frame rate
   (`#5292`_). Thanks to Thomas Moulard for this enhancement.

1.7.0 (2011-12-14)
------------------

 * camera1394 package becomes a unary stack, no longer part of
   camera_drivers.

1.6.0 (2011-07-20)
------------------

 * ROS Electric.
 * Handle AVT Guppy F036C in format7 mode. That device does not return
   a valid color filter. Use the bayer_pattern parameter for any
   device not providing one (`#5063`_).
 * Support 16-bit Bayer encodings (`#4738`_).

1.4.2 (2011-06-06)
------------------

 * The driver will stamp messages with the current ROS time if the new
   use_ros_time parameter is set. Fixes USB-1394 camera support
   (`#4841`_)
 * Use proper namespace for camera_info_manager (`#4760`_).

1.4.1 (2011-02-09)
------------------

 * Bug fixes for publishing Format7 Region of Interest in CameraInfo
   (`#4735`_, `#4736`_).

1.4.0 (2011-01-31)
------------------

 * ROS Diamondback.
 * Add nodelet version of driver.
 * Add IIDC Format7 support (`#4222`_), thanks to Ken Tossell.
 * Format7 binning and ROI comply with REP 104.
 * Add Focus and Zoom feature support (`#4631`_), thanks to José Antonio
   Álvarez Ruiz.
 * The driver will only set video_mode or frame_rate to values
   supported by the device.
 * Bayer decoding within the driver deprecated (`#4725`_). Prefer
   image_proc decoding instead. Only image_proc method supported in
   Format7 modes.
 * Better support for Mac OS X (`#4659`_).
 * Re-licensed under LGPL.

1.2.8 (2011-02-09)
------------------

 * Fixes for Mac OS X install and build (`#4659`_).

1.2.5 (2010-10-28)
------------------

 * Provide new retry_on_open parameter (default true). Set it false
   for devices like Videre STH-DCSG-VARX-C, which does not tolerate
   resetting (`#4396`_).

1.2.0 (2010-07-23)
------------------

 * ROS Cturtle.
 * Initial camera1394 package, released to Cturtle as part of the
   camera_drivers stack.
 * Retry camera open, if it fails initially (`#4251`_).
 * Fix libdc1394 debayer frames memory management (`#4261`_).


.. _`#1`: https://github.com/ros-drivers/camera1394/issues/1
.. _`#6`: https://github.com/ros-drivers/camera1394/issues/6
.. _`#7`: https://github.com/ros-drivers/camera1394/issues/7
.. _`#9`: https://github.com/ros-drivers/camera1394/pull/9
.. _`#10`: https://github.com/ros-drivers/camera1394/issues/10
.. _`#11`: https://github.com/ros-drivers/camera1394/pull/11
.. _`#15`: https://github.com/ros-drivers/camera1394/issues/15
.. _`#29`: https://github.com/ros-drivers/camera1394/issues/29
.. _`#32`: https://github.com/ros-drivers/camera1394/pull/32
.. _`#33`: https://github.com/ros-drivers/camera1394/issues/33
.. _`#4222`: https://code.ros.org/trac/ros-pkg/ticket/4222
.. _`#4251`: https://code.ros.org/trac/ros-pkg/ticket/4251
.. _`#4261`: https://code.ros.org/trac/ros-pkg/ticket/4261
.. _`#4396`: https://code.ros.org/trac/ros-pkg/ticket/4396
.. _`#4631`: https://code.ros.org/trac/ros-pkg/ticket/4631
.. _`#4659`: https://code.ros.org/trac/ros-pkg/ticket/4659
.. _`#4696`: https://code.ros.org/trac/ros-pkg/ticket/4696
.. _`#4725`: https://code.ros.org/trac/ros-pkg/ticket/4725
.. _`#4735`: https://code.ros.org/trac/ros-pkg/ticket/4735
.. _`#4736`: https://code.ros.org/trac/ros-pkg/ticket/4736
.. _`#4738`: https://code.ros.org/trac/ros-pkg/ticket/4738
.. _`#4760`: https://code.ros.org/trac/ros-pkg/ticket/4760
.. _`#4841`: https://code.ros.org/trac/ros-pkg/ticket/4841
.. _`#5063`: https://code.ros.org/trac/ros-pkg/ticket/5063
.. _`#5292`: https://code.ros.org/trac/ros-pkg/ticket/5292
.. _`#5350`: https://code.ros.org/trac/ros-pkg/ticket/5350
.. _`#5322`: https://code.ros.org/trac/ros-pkg/ticket/5322
