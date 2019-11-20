# Time stamp and frame of the object. All objects are in ENU frame.
std_msgs/Header header

# CURRENTLY UNUSED, see labeled_classification
string classification

# The classification of the object determined algorithmically or by a human.
string labeled_classification

# Pose of the object in ENU
geometry_msgs/Pose pose        

# Size of the object, like markers, scale is applied first, then pose.
geometry_msgs/Vector3 scale

# The unique id of each object. It is a counter_id, so the later the object is found the 
# higher the id will be.
uint32 id

# The confidence that this is an object of interest from 0-1
float32 confidence    
# The confidence of the classification from 0-1
float32 classification_confidence                 

# Points in ENU frame that belong to the object
geometry_msgs/Point32[] points
