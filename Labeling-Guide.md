# NOTICE: legacy / out of date
> MIL now uses labelbox.io. See [this guide](https://github.com/uf-mil/mil_common/wiki/Labeling-With-labelbox.io)

# Labeling Guide
To provide training data to machine learning algorithms, and to verify performance of our perception, is is important to have explicitly "labeled" images and lidar data, with tags indicating what is in the image/pointcloud and where. For this to be useful, we must all use the same conventions for what these tags are.

## Images
### Scan The Code
#### Config
I used the following config yaml to generate the images:
```
bags:
    - file: stc/comp_bgy.bag
      name: stc/comp_bgy
      topics: /stereo/right/image_rect_color
      freq: 1.0
    - file: stc/comp_brg2.bag
      name: stc/comp_brg2
      topics: /stereo/right/image_rect_color
      freq: 1.0
    - file: stc/comp_brg.bag
      name: stc/comp_brg
      topics: /stereo/right/image_rect_color
      freq: 1.0
    - file: stc/comp_rgb.bag
      name: stc/comp_rgb
      topics: /stereo/right/image_rect_color
      freq: 1.0
    - file: stc/comp_rgr.bag
      name: stc/comp_rgr
      topics: /stereo/right/image_rect_color
      freq: 1.0
    - file: stc/comp_yby.bag
      name: stc/comp_yby
      topics: /stereo/right/image_rect_color
      freq: 1.0
    - file: stc/comp_yry.bag
      name: stc/comp_yry
      topics: /stereo/right/image_rect_color
      freq: 1.0
```
#### Example:
![scan the code](https://i.imgur.com/QSd5oEY.jpg)
#### Guide
* Label the white border outside the LED panel as a 4 sided polygon and name ```stcout```, no attributes
* Label the LED panel itself with a 4 sided polygon and name ```stcin```,
  * In the attributes section, leave one of the following strings ```blue```,```green```,```red```,```yellow```,```off``` based on the color of the LEDs, or off if they are off
* Only label the side of scan the code closest to the camera, dont try to label the ones on the side
* Feel free to jump around or skip images. We want the most **diverse** set of labeled data, not a bunch of labels on very similar images