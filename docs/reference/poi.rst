:mod:`mil_poi` - POI Handling
-----------------------------

.. automodule:: mil_poi

Using with Rviz
^^^^^^^^^^^^^^^
To view and interactive with interactive marker POIs in Rviz, you will need to enable
the "Interactive Markers" panel in Rviz. You will then need to set the update topic
to be equal to the update topic published by the interactive marker server published
by the POI server. This topic name will likely end in ``/update``.

All POIs are represented as spheres in the interactive marker server; you can move
these spheres around to change their location. Note that points are currently
fixed to the x and y axes, meaning that you can not have floating or submerged points.

Configuration Files
^^^^^^^^^^^^^^^^^^^
When the POI server is launched, it is provided with several ROS parameters to
describe how the server should function. This includes POIs that are spawned by
default upon server initialization.

The default format of the POI configuration file is the following format:

.. code-block:: yaml

    ---
    global_frame: enu # Name of the frame to derive POI locations from
    initial_pois: # List of POIs that are spawned by default
      start_gate: [0, 0, 0] # Name (key) and location (value) of specific POIs

POIServer
^^^^^^^^^
.. attributetable:: mil_poi.POIServer

.. autoclass:: mil_poi.POIServer
    :members:

TxPOIClient
^^^^^^^^^^^
.. attributetable:: mil_poi.TxPOIClient

.. autoclass:: mil_poi.TxPOIClient
    :members:
