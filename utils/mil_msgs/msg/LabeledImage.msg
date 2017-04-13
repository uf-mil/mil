# Represents the complete data from a labeled image from LabelMe

sensor_msgs/Image image            # Image where objects are labeled

sensor_msgs/CameraInfo camera_info # CameraInfo from a frame near this image

mil_msgs/LabeledObject[] objects   # List of labeled objects in image
