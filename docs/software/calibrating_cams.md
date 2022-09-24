# Calibrating Cameras

When using a camera, calibration is required to ensure that we are able to receive
images with no distortion. If we receive distorted images, it may be more difficult
to reconstruct a 3D scene from the camera. Examples of camera distortion and a
more detailed process on calibrating cameras can be found on the
[OpenCV website](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html).

Camera calibration should only need to be performed once per camera model, as
the distortion produced by one camera should generally be the same as another camera
of the same model. However, if two cameras of the same model have different
settings, then calibrating both cameras individually may be required.

## Calibrating a new cameras

Upon receiving a new camera, follow the below instructions for calibrating the camera.
1. Make sure the camera can be visible by your computer and ROS.
1. Create a new launch file for each new camera. Examples can be found in
   ``$(find navigator_launch)/hardware/cameras/``.
1. Run ``rosrun camera_calibration cameracalibration.py``. This script will launch
   a GUI which you can use to calibrate the camera. You will need to move
   a sturdy checkerboard around in front of the camera. The GUI will accumulate
   more and more samples, and finally you can click "Calibrate" to receive the
   camera's parameters.
1. Paste the new calibration parameters into a camera calibration launch file.
1. Reference this new camera calibration file from the camera launch file.

If all has gone well, you should be able to open the rectified image topic and
see that the produced image has no distortion. If the image appears curvy or misaligned,
then camera calibration was not performed correctly.

:::{warning}
You can only verify successful camera calibration on the **rectified image topic**.
Do not use the raw or color image topics - these will likely produce images which
have little to no observable distortion.
:::
