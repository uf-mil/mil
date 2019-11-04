# Represents all labeled objects from a tool like LabelMe

# Header for frame where the objects were labeled
# stamp is the same as the stamp of the frame where these objects were labeled
# frame_id is the frame_id of the camera where this object is labeled
Header header

mil_msgs/LabeledObject[] objects   # List of labeled objects in image
