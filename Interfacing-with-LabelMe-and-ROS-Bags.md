# Interfacing with LabelMe and ROS Bags
MIL uses our [own hosted instance of LabelMe](https://ci.mil.ufl.edu/labelme/tool.html) to label images used to train machine learning systems and evaluate perception performance. We also have a script [label_my_bag.py](https://github.com/uf-mil/mil_common/blob/master/utils/mil_tools/mil_ros_tools/label_me_bag.py) to interface between ROS bag files we collect and this online labeling system. 

To see information about how to use the script:
```rosrun mil_tools label_me_bag.py --help```

Below is a usage guide for this tool.

## Mounting the LabelMe directory locally
In order to add new images to labelme or extract labeled images back into a bag file, it is most convenient to mount the labelme instance locally. Otherwise, you will need to copy the individual affected directories.

From [guide on mounting filesystems over ssh](https://github.com/uf-mil/mil_common/wiki/Mounting-Remote-Filesystems), the following command will mount the labelme directory to your local system:
```
mkdir ci-labelme -p
sshfs plumbusiadmin@ci.mil.ufl.edu:/var/www/labelme ci-labelme
```

## Putting Label Data onto LabelMe
In order to add new images to labelme:
* Gather 1 or more bag files with 1 or more ```std_msgs/Image``` topics with images you want labeled
* Create a yaml file for the label_me_bag.py script to use for both putting images from the bag onto lableme and, eventually, extracting labels from lableme back into the bag. See [the example configuration](https://github.com/uf-mil/mil_common/blob/master/utils/mil_tools/mil_ros_tools/label_me_bag.py).
* Mount the lableme instance locally
* Run label_me_bag.py to put bag images onto labelme ```rosrun mil_tools label_me_bag.py <CONFIGURATION FILE> -d <MOUNTED LABELME DIR> -v```
* Starting labeling! You can periodically check how many images have been labeled and eventually 

## Viewing Label Status
Once your configuration files are made and the images are on labelme, you can run the following command to see a report of the label completion.
```rosrun mil_tools label_me_bag.py <CONFIGURATION FILE> -d <MOUNTED LABELME DIR> -v --generate-report```

## Extracting labels
Once you are happy with the number of labels for the bags you put on labelme, you can now run the script again to extract those labels into ROS messages stored in a new bag file. The name of the new bag can be specified in the configuration YAML.
```rosrun mil_tools label_me_bag.py <CONFIGURATION FILE> -d <MOUNTED LABELME DIR> -v --extract```

Note: By default it will skip bags which have already been created. So if you run once with partial labels and again with more labels, you must also add the ```--force``` flag to override the bag that is already there.
