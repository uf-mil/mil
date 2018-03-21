# Overview
[LabelBox](https://www.labelbox.io/) is MIL's primary tool for image labeling. Labeling is the process of having a human record the correct output for a perception algorithm using their own knowledge of the task. This includes **segmentation**, or drawing where in an image / 3D pointcloud a particular object of importance is and **classification**, or selecting what type of object / feature is currently displayed. Both of these labeling tasks provides the training data needed by some perception algorithms such as neural networks and provides a way to measure accuracy of any perception algorithm. This wiki page described the process of moving image data from rosbag files we record at test days into LabelBox, labeling this data, then exporting the labels and using them in a program.

# Extracting Images from rosbags
Say you have a set of 3 bag files from a test day containing images of buoys. You would like to have a human segment the buoys in a random selection of these images so that you can train a neural network to recognize buoys.

## Creating a extract_bag_images config file
To extract images from the bag files you selected, first copy them to somewhere on your local computer. It is best to preserve the filenames/directories as they are stored on the MIL fileserver so that someone else can get the same images later. In this case, we'll assume you copied them to ```/home/user/bags```, with the 3 files you are extracting images from in in a subdirectory following the MIL fileserver convention for the date is was recorded```2018-02-03```.

Now you use the ```extract_bag_images``` to place the images from these files into a dataset for labeling. This program requires a yaml file specifying which bags to pull images from. Save this file to something you'll remember, like ```buoy_segmentation.yaml```.

```
datasets:
    - name: navigator_buoys1
      sources:
          - file: 2018-02-16/buoys1.bag
            topic: /camera/front/left/image_raw
            encoding: rect_color
            freq: 1.0
          - file: 2018-02-16/buoys1.bag
            topic: /camera/front/left/image_raw
            encoding: rect_color
            freq: 1.0
```

Some things to notice about this config:

* The ```freq``` option specifies how many frames per second to grab from the bag. You likely don't want all images in a bag, as images from the same period of time are likely very similar and will only add time to your labeling. Select a frequency that will give you enough useful data.
* The ```encoding``` flag can be mono, rect, color, or rect_color for grayscale, rectified grayscale, color (debayered) and rectified color images respectively. If it is not set, the images will be saved in whatever format they are in the bag file. Usually, you will want ```rect_color```
* For ```buoy2.bag```, I specified ```start: 5.0``` and ```stop: 15```. This means the first image extract from the bag will be from 5 seconds into the bag and the last image will be from 15 seconds into the bag. You may use this if, for example, only a certain section of the bag contains the data you are interested in.
* For the ```file``` attributes, I specified the file relative to my bags directory. When we later run the ```extract_bag_images``` script, I will specify where to find these bags. Doing it this way is helpful for other people who may use your config but store their bags in a different directory locally.
* The ```topic``` specifies which topic in the bag to pull the raw image from. You can use ```rosbag info``` to see what camera topics are in a bag.

## Running the extract_bag_images script
Now that you have your bags collected and your config made, it is time to extract images! Let's say I want to store the extract images in ```/home/user/images/buoy_images```. Simply run:

```rosrun mil_tools extract_bag_images buoy_segmentation.yaml --source-dir /home/user/bags --image-dir /home/user/images/buoy_images```

This may take a while, so go make some tea. I like earl grey because it has a lot of caffeine. When the command finishes running, verify that images have been put into the directory you specified
```
$ tree images/buoy_images
images/buoy_images
└── navigator_buoys1
    ├── 1517758829189875385.png
    ├── 1517758849209839979.png
    ├── 1517758869232214735.png
    ├── 1517758889239084237.png
    ├── 1517758909257297352.png
    ├── 1517758929272849849.png
    ├── 1517758949293060397.png
    ├── 1517758969307604060.png
    ├── 1517758989303429572.png
    ├── 1517763434560815133.png
    ├── 1517763436595968920.png
    ├── 1517763438599872258.png
    ├── 1517763440636423779.png
    ├── 1517763442640406787.png
    ├── ...
1 directory, 48 files
```

## Adding more data later on
What if you have another test day and record more data that you would also like to label? As of writing this, LabelBox does not allow you to delete / add images to a dataset once it has been uploaded. However, you can simply add another dataset to an existing project. If your new data is in ```/home/user/bags/2018-03-14/```, simply amend you config file and run extract_bag_images again.

```
datasets:
    - name: navigator_buoys1
      sources:
          - file: 2018-02-16/buoys1.bag
            topic: /camera/front/left/image_raw
            encoding: rect_color
            freq: 1.0
          - file: 2018-02-16/buoys1.bag
            topic: /camera/front/left/image_raw
            encoding: rect_color
            freq: 1.0
    - name: navigator_buoys2
      sources:
          - file: 2018-03-14/buoys_cloudy.bag
            topic: /camera/front/left/image_raw
            encoding: rect_color
            freq: 1.0
          - file: 2018-03-14/buoys_raining.bag
            topic: /camera/front/left/image_raw
            encoding: rect_color
            freq: 1.0
```

```rosrun mil_tools extract_bag_images buoy_segmentation.yaml --source-dir /home/user/bags --image-dir /home/user/images/buoy_images```

```
$ tree images/buoy_images
images/buoy_images
└── navigator_buoys1
    ├── ...
└── navigator_buoys2
    ├── ...
1 directory, 48 files
```

# Uploading to LabelBox