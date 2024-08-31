"""
The MIL POI system is used to keep track of relevant points of interest across
a landscape. This can be used to give a robotic system a headstart in determining
the final target of a mission; for example, a robotic system could be instructed
to head to a specific POI before looking around more closely to find the actual object
it needs to interact with.

.. container:: services

    .. describe:: /poi_server/add

        :class:`mil_poi.srv.AddPOIRequest` → :class:`mil_poi.srv.AddPOIResponse`

        Add a POI object into the POI server listing.

    .. describe:: /poi_server/delete

        :class:`mil_poi.srv.DeletePOIRequest` → :class:`mil_poi.srv.DeletePOIResponse`

        Deletes a POI with the given name, and returns whether the deletion was successful.

    .. describe:: /poi_server/move

        :class:`mil_poi.srv.MovePOIRequest` → :class:`mil_poi.srv.MovePOIResponse`

        Moves a POI with the given name to the given position, and returns whether
        the move was successful.

.. container:: topics

    .. describe:: /points_of_interest/update

        :class:`visualization_msgs.msg.InteractiveMarkerUpdate`

        Sends updates about changes to interactive markers. When attempting to view
        interactive markers in Rviz, the ``update_topic`` parameter of the Interactive
        Markers panel needs to be set to this topic.
"""

from .async_interface import AsyncPOIClient
from .server import POIServer
