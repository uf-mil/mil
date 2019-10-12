# Overview
[LabelBox](https://www.labelbox.io/) is MIL's primary tool for image labeling. Labeling is the process of having a human record the correct output for a perception algorithm using their own knowledge of the task. This includes **segmentation**, or drawing where in an image / 3D pointcloud a particular object of importance is and **classification**, or selecting what type of object / feature is currently displayed. Both of these labeling tasks provides the training data needed by some perception algorithms such as neural networks and provides a way to measure accuracy of any perception algorithm. This wiki page described the process of moving image data from rosbag files we record at test days into LabelBox, labeling this data, then exporting the labels and using them in a program.

# Extracting Images from rosbags
Say you have a set of 3 bag files from a test day containing images of buoys. You would like to have a human segment the buoys in a random selection of these images so that you can train a neural network to recognize buoys.

## Creating a extract_bag_images config file
To extract images from the bag files you selected, first copy them to somewhere on your local computer. It is best to preserve the filenames/directories as they are stored on the MIL fileserver so that someone else can get the same images later. In this case, we'll assume you copied them to ```/home/user/bags```, with the 3 files you are extracting images from in in a subdirectory following the MIL fileserver convention for the date is was recorded```2018-02-03```.

Now you use the ```extract_bag_images``` to place the images from these files into a dataset for labeling. This program requires a yaml file specifying which bags to pull images from. Save this file to something you'll remember, like ```buoy_segmentation.yaml```.

```yaml
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
* Images are only fetched from one camera topic for this dataset. **Please use another dataset for additional cameras.** Some algorithms may work best when only trained on the same sensor it will actually be run against.

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
    ├── ...
```

## Adding more data later on
What if you have another test day and record more data that you would also like to label? As of writing this, LabelBox does not allow you to delete / add images to a dataset once it has been uploaded. However, you can simply add another dataset to an existing project. If your new data is in ```/home/user/bags/2018-03-14/```, simply amend you config file and run extract_bag_images again.

```yaml
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
```

# Uploading to LabelBox
For each dataset you created in the config file above, make a dataset on labelbox.io. To do this log in and go to ```https://app.labelbox.io/data/new```. Click select file and upload all the images for this dataset. You may now add this dataset to your labeling project

# Exporting label data
Once you have labeled the images for a project, you can export them:

1. Go to page for your project https://app.labelbox.io/projects and click on your project
1. Click on the export tab
1. Ensure the "Label Format" box is set to "XY" and the export type is "json". It must be in this format to use the tools mentioned below.
1. Click export

# Visualizing labeled data
MIL has a utility class ```mil_vision.LabelBoxParser``` which is useful for parsing exported labels from LabelBox paired with the images you extracted from bags ealier. If this file is run as an executable, it will run an example program that simply displays the labels. The command is:

```rosrun mil_vision labelbox_parser.py <exported json file> <directory with datasets from bags>```

This will display an OpenCV window with the image and the label drawn on. Click any key to continue to the next image.

# Using labeled data in a program
Let's say you want to read the segmented labels for the buoys and store the mean RGB color of each labeled buoy to a CSV file. Below is an example program which does this, dumping the mean RGB for the segmented buoy into a CSV file for latter use in, for example, training a classifier. The following assumptions are made:
* You have exported labels from the labelbox.io project to labels.json
* The directory containing the images for all the datasets labeled is in /home/user/images/buoy_images

```python
#!/usr/bin/env python
from mil_vision import LabelBoxParser
from mil_vision import contour_mask
import cv2
import pandas

class BuoyColorExtractor(object):
    def __init__(self, labelfile, imagedir):
        self.parser = LabelBoxParser(labelfile, imagedir)
        self.features = []

    def cb(self, label, img):
        '''
        Called for each labeled image. Label contains all segmentations in the image.
        img is a 3 channel uin8 numpy array with the image data.
        '''
        if 'buoy' not in label:  # Ignore labels which don't have a buoy
            print 'no buoy label found'
            return
        for polygon in label['buoy']:  # Store the mean BGR color for each buoy labeled in this image
            points = LabelBoxParser.label_to_contour(polygon, img.shape[0])  # convert label polygon dictionary to numpy array
            mask = contour_mask(points, img.shape)  # Get a mask for the inside of the contour
            mean = cv2.mean(img, mask)  # Get the mean color (3 channel) in this region of the image
            self.features.append(mean)  # Add this mean to the list of other means

    def extract_features(self, filename='mean.csv'):
        self.parser.get_labeled_images(self.cb)  # Call self.cb for each labeled image
        data = np.array(self.features)
        df = pandas.DataFrame(data=data, columns=['B', 'G', 'R'])  # Pandas is used just to write to csv file
        df.to_csv(filename)


if __name__ == '__main__':
    extractor = BuoyColorExtractor('labels.json', '/home/user/images/buoy_images')
    extactor.extract_features('mean.csv')
```
