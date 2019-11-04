# Type of the object, like "buoy", "dock", or "human"
string name

# Points in an image of the object. It is to be interpreted as follows:
# len(points) == 0: Object is somewhere in image
# len(points) == 1: Center of object is at points[0]
# len(points) == 2: points[0] and points[1] are the upper left and bottom right corners of a bounding rectangle enclosing the object
# len(points) > 2: points are the verticies of a polygon around the object
mil_msgs/Point2D[] points

# Confidence that the object identified is correctly identified. It is to be interpreted as follows:
# -1: No confidence mechanism is in place, it is not clear how confident the classification is
# [0, 1.0]: The classification is this amount confident, with 0 being completly unsure and 1 being completely sure
float64 confidence

# Unspecified additional data associated with the object. May be used for things like color, variety, etc
string attributes
