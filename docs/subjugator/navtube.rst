========
Nav Tube
========

One of the major components of SubjuGator 8 is the navigation (nav tube). This pressure vessel is located right below the main pressure vessel and contains all of our navigation sensors and equipment. The navigation computer located inside the vessel allows access to the data from these components through a direct ethernet connection.

.. warning::

     This system was originally used on SubjuGator 7 and it was ported over with little to no changes in hardware. This does mean that the system will need a major overhaul for continued use of SubjuGator 8.

Navigation Computer
===================

The navigation computer consists of a Gumstix Overo Computer-on-Module that mounts to a carrier board. The carrier board contains headers for the Teledyne Dopper Velocity Logger (DVL) connector board, Analog Devices Inertial Measurement Unit (IMU), and the pressure sensor. At one point, the navigation computer was also connected to a GPS antenna.
.. warning::

     No documentation or PCB design files exist for the navigation computer carrier board and there are no more functioning spare boards. Additionally, only the newer GumStix Overo COM boards are available to purchase (with a lead time of roughly 1 year) which may not be compatible with the current system. Be very careful when working inside of the navigation tube.

Troubleshooting The Nav Tube Connection
---------------------------------------

If you cannot ping the Navigation Computer (192.168.37.61), please ensure (in the following order) that:

* You are pinging 192.168.37.61.
* The ethernet subconn cable has been connected between both pressure vessels.
* You restart the sub at least once.
* That the proper ethernet interface is configured in /etc/netplan YAML file. You may need to check ifconfig for the 'enpXs0' interface (where X is a whole number) if you have unplugged and replugged anything in the sub recently (especially the graphics card). Make sure to test the netplan with ``sudo netplan try`` so that you don't get locked out of the sub because of a bad configuration!
* The ethernet cables are properly connected inside the main pressure vessel.
* The ethernet cables and other cables are properly connected inside the Nav tube.
* The navigation computer is receiving power AND is turned on (green AND blue light on the GumStix). Make sure to be extra careful and vigilent when opening and handling the navigation computer.

ADIS16400/16405-BMLZ IMU & Magnetometer
===========================================

The `ADIS16400-BMLZ <https://www.analog.com/en/products/adis16400.html#product-overview>`_ / `ADIS16405-BMLZ <https://www.analog.com/en/products/adis16405.html#product-overview>`_ is the device responsible for tracking our position and orientation in the water. Currently, there is an ADIS16400 IMU in the sub, but it previously used an ADIS16405.

.. warning::

    These components are obsolete! It may be difficult to find replacement parts in the future if SubjuGator 8 will see continued use...

Calibrating the Magnetometer
----------------------------

The magnetometer inside of the ADIS16400/16405 measures the strength of the magnetic field at the orientation it faces. We use this data in conjunction with the IMU's gyroscopes and accelerometers to determine the orientation of the sub in the water. The process of calibrating the magnetometer is called magnetic hardsoft compensation and essentially changes how we bias the data it gives us.

.. note::

    Calibrating the magnetometer must be done in a pool.

To calibrate the magnetometer you must first collect magnetometer data (``/imu/mag_raw``) to use for the calibration script. This can be done through the following:

.. code-block:: bash

   $ roslaunch subjugator_launch sub8.launch

or if you prefer to not launch the entire sub:

.. code-block:: bash

   $ roslaunch subjugator_launch nav_box.launch imu:=true

Then, in a separate terminal window, navigate to a known directory start recording the rosbag by typing:

.. code-block:: bash

   $ rosbag record -O <name here>.bag /imu/mag_raw

where <name here> is a substitute for whatever name you want to give to your bag (I recommend something memorable, like Frank or Penelope). This will record the bag data in the directory that the terminal window is in.

When you have started recording the bag, have members who are in the water rotate the sub. Only the sub should move, not the members holding onto the sub. The calibration can be done in any order, but you must complete a full roll, pitch, and yaw rotation plus have a few minutes of data with all three occurring at the same time. Once you are done collecting data, kill the recording window using ``ctrl+c``.

Next, we must run the calibration script with our data. This script is located in ``SubjuGator/drivers/magnetic_compensation/sub8_magnetic_hardsoft_compensation/scripts``. Type the following:

.. code-block:: bash

   $ ./generate_config <path to calibration data bag file>

note that this is a python script, so

.. code-block:: bash

   $ python3 generate_config <path to calibration data bag file>

is also valid.

.. note::

    If the script fails because of the ``fit_ellipsoid`` method and the points on the first figure are colinear or nearly colinear you may not have collected thorough enough data. The alternate possibility is a malfunctioning magnetometer.

The output of the script should be a 3x3 matrix labeled ``scale`` and a length 3 vector labeled ``shift``. These values go into the ``scale`` and ``shift`` values located inside of ``subjugator_launch/launch/subsystems/nav_box.launch``.

After running ``cm``, you will have (hopefully) successfully calibrated the magnetometer. Make sure to test the sub after calibration to see if the new configuration values are an improvement over the old ones.

Important Links and Datasheets
==============================

- `ADIS16400/ADIS16405 Datasheet <https://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16400_16405.pdf>`_
