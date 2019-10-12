# Cameras
Currently, NaviGator has 3 camera:
* 1 Point Grey Blackfly U3-23S6C facing forward on the front port side
* 1 [See3cam_cu20](https://www.e-consystems.com/wide-temperature-range-hdr-usb-cameraboard.asp) facing forward on the front starboard side
* 1 other poingrey camera facing starboard on the starboard side

## See3cam
Some details on the See3Cam
* 2.0 MP AR0230AT sensor connected to an ISP for auto exposure, etc
* Has HDR mode which combines 2 images at different exposures for a higher dynamic range
* Run over USB 3.1 Gen1 (aka USB 3.0)
* Plug and play on linux via v4l2, run through the ROS usb_cam driver
* S-mount lens, roughly 120deg FOV


## Point Grey Cameras
The point grey cameras are run through the pointgrey camera driver.

### U3-23S6C
* __Resolution / FPS:__ We're running the cameras in format7_mode4 (960x600 pixels) with raw_8 at 30 fps
* __Camera GUIDs:__ 00b09d0100e84a44 and 00b09d0100e84a42 (currently not attached)
* __Bus Bandwith:__  960 * 800 * 30 (fps) * 8 (raw_8) = ~139 MB/s

#### Frame rates / resolutions
![](https://i.imgur.com/Ai4IQDx.png)

#### Video modes
![](https://i.imgur.com/TTM4IXT.png)

### Starboard one
* USB3