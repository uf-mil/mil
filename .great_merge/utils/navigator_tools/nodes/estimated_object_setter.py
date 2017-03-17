#!/usr/bin/env python
import numpy as np

import ast
import sys
from argparse import RawTextHelpFormatter, ArgumentParser

import txros
import navigator_tools
from twisted.internet import defer
from coordinate_conversion_server import Converter
from navigator_msgs.srv import CoordinateConversionRequest, ObjectDBQuery, ObjectDBQueryRequest

@txros.util.cancellableInlineCallbacks
def main(name, lla):
    nh = yield txros.NodeHandle.from_argv("esitmated_object_setter")
    db = yield nh.get_service_client("/database/requests", ObjectDBQuery)

    convert = Converter()
    yield convert.init(nh)

    # Convert the name to Be_Like_This
    name = '_'.join(map(lambda txt: txt.title(), name.split("_")))

    point = yield convert.request(CoordinateConversionRequest(frame="lla", point=lla))
    yield db(ObjectDBQueryRequest(cmd="{}={p[0]}, {p[1]}".format(name, p=point.enu)))    


if __name__ == "__main__":
    usage_msg = "Used to set the estimated position of objects in the database with lla positions."
    desc_msg = "Pass the name of the database object and the lla position and this will set it's esimated position in the database. \n\
    ex. rosrun navigator_tools estimated_object_setter.py scan_the_code \"[85.3, -25.6]\" \n\
    ex. rosrun navigator_tools estimated_object_setter.py Shooter \"[82.32, -26.87, 2]\""

    parser = ArgumentParser(usage=usage_msg, description=desc_msg, formatter_class=RawTextHelpFormatter)
    parser.add_argument(dest='name',
                        help="Name of the object.")
    parser.add_argument(dest='lat', type=float,
                        help="Latitude in degrees of the object of intrest.")
    parser.add_argument(dest='long', type=float,
                        help="Longitude in degrees of the object of intrest.")
    parser.add_argument('--alt', '-a', action='store', type=float, default=0.0,
                        help="Altitude in meters of the object of intrest (usually can be omitted).")

    args = parser.parse_args(sys.argv[1:])
    name = args.name
    lat = args.lat
    lon = args.long  # `long` is python reserved :(
    alt = args.alt

    txros.util.launch_main(lambda: main(name, [lat, lon, alt]))
