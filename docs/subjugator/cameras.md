# SubjuGator Cameras

The SubjuGator comes with three cameras: two front cameras and a down camera.
The front cameras can be used for obstacle avoidance, path planning, or any
other vision tasks. The down camera is primarily used for objects placed on the
ground; frequently, objects on the ground are helpful for indicating the direction
of various other objects.

## Namespaces

The sub's cameras can be accessed through a selection of ROS topics. These topics
usually indicate which camera is producing the image, along with any added filters
or pipelines used on the image. Like every topic, the topics publishing data
have names and namespaces.

For example, the topic `/camera/front/right/image_raw` indicates that the image
published on the topic is from the front right camera, and is the raw image from
that camera.

If you are planning to add new vision topics to the architecture of the sub,
please follow these already existing organizational rules. It helps to have
these topics under understandable names so new and old members are not confused.
