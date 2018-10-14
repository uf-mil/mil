# Transfer Learning Object Detection - A Short Tutorial

# @TODO
1. Configurable image download
2. Add docker build to the install script. Until then docker build must be done manually. 
3. Have datasets stored on internal server so it can be downloaded. 
4. Configurable resize (Low importance due to no significant performance gain)
5. Configurable CSV input (Currently it still assumes you are handing it a json or PASCAL so it will error out if those are not provided.)

## Quick Start: The Important Bits

This is a quick start to getting this running and skips some lengthy explination. In the future you will not need to edit the generate_tfrecord.py script, and will only need to add the labelmap.pbtxt and data. Note that the model directory that you download from the model zoo needs to be cleaned. Only the three model model.cpkt files and the pipeline.config should be present in the directory. Any further files (such as a graph.pbtxt or a checkpoint file) will break the trainer! 

### Data

Depending on the approach you opted for, you either downloaded a json in the XY format from labelbox, or you downloaded the VOC format. If you downloaded the VOC format, you will need to separate the XML and PNG files into Annotation and Images folders respectively (this will be done automagically in the future). If you downloaded a json, put it into the json_files folder! This should be called data.json. If there is currently a json in there, rename it to correctly match the dataset it reflects or delete it. It has likely been stored somewhere else already.

### process.py
```bash
$ python process.py --json_name='json_files/project_labels.json' --labelmap_path='../data/totems&buoys.pbtxt' --cleanup=False

```
Handles all the data processing from labelbox. Example usage is above. There are many options when running this script, allowing for more dynamic processing depending on your needs. 

```python
flags.DEFINE_boolean('resize', False, 'Resize images? Boolean.')
flags.DEFINE_string('labelbox_format', 'json',
                    'Only supports json XY or PASCAL inputs')
flags.DEFINE_string('json_name', '', 'Path to json file')
flags.DEFINE_string('csv_input', 'default', 'Path to the CSV input')
flags.DEFINE_string('ann_dir', '', 'Path to XML directory')
flags.DEFINE_string('image_dir', 'default', 'Path to the image directory')
flags.DEFINE_string('output_path', 'default', 'Path to output TFRecord')
flags.DEFINE_string('labelmap_path', '', 'Path to labelmap.pbtxt')
flags.DEFINE_boolean('cleanup', True, 'Delete all files? Boolean.')
```
Resize: Allows you to resize images to a default of 256x150. This has proven superfluous as TF automatically resizes images for you and no performance gain was noted. Left as an option just in case.
labelbox_format: If you download PASCAL inputs from labelbox you need to give the ann_dir and image_dir in commandline. Otherwise it defaults too and looks for a json file. 
json_name: If using a json file put the path to the file here. 
csv_input: Csv files are auto-generated if you use json or PASCAL inputs, however if you have your own csv files and just want to run the generate_tfrecords, you can do so by providing the csv_input. 
ann_dir & image_dir: Needed if using the PASCAL inputs. Otherwise these are auto generated.
output_path: Path for outputting tf_records. There is a default since it assumes it is generating two tf_record files however if you wish you can force a direct output. 
labelmap_path: Path to the labelmap. This is always requried and sends the information mapping for ID to label. 
cleanup: Whether or not you wish to delete all downloaded images and annotations and other generated files. Defaults to true.  


### train.sh

Run this after all the above steps have been completed (or you just ran the process_images.sh) and you are good to go!

### generate_tfrecord.py

Takes the CSV files and image/label directories and generates the two tfrecords we need for the trainer. Currently you have to manually edit this file to add the classes you need. In the future this should change to be automated based on the labelmap.pbtxt file you also have to manually edit. This does make a good place to change what specific segments of your dataset you want to train on however. For example if my labelled data had 25 different classes labelled, but I only cared about one, I could only process the labels that have te correct name. An example of editing this file is as follows. Please note that the strings used must **EXACTLY** match the labels used in labelbox.

If I wanted to go from training a network to find roulette_wheels and instead find the entire NaviGator course, I would have to change the way the tfrecord interprets the labels. The example change would be the following:

#### Initial
```python
def class_text_to_int(row_label):
    if row_label == 'Wheel':
        return 1
    else:
        None
```
#### Modified

```python
def class_text_to_int(row_label):
    if row_label == 'Red Totem':
        return 1
    elif row_label == 'Blue Totem':
        return 2
    elif row_label == 'Green Totem':
        return 3
    elif row_label == 'Yellow Totem':
        return 4
    elif row_label == 'Black Totem':
        return 5
    elif row_label == 'Black Buoy':
        return 6
    else:
        None
```

### labelmap.pbtxt

Currently you have to manually edit this file as well, which is unfortunate. Ideally only one of these files should be manually edited. This contains all the labels and matching ID's you will be using for your identification. An example of this file is the following:

#### Equivalent labelmap for the first block from generate_tfrecord.py
```
item {
  id: 1
  name: 'Wheel'
}
```

#### Equivalent labelmap for the second block from generate_tfrecord.py
```
item {
  id: 1
  name: 'Red Totem'
  id: 2
  name: 'Blue Totem'
  id: 3
  name: 'Green Totem'
  id: 4
  name: 'Yellow Totem'
  id: 5
  name: 'Black Totem'
  id: 6
  name: 'Black Buoy'
}
```

## Smart Start: A "Brief" Tour of MIL's Tensorflow Object Detection Pipeline
Make your way into the "mil_common/docker_tf" directory. We will need to build the docker image if this was updated recently. Inside of this you should see a dockerfile and a shell file (pipeline.sh). Run pipeline.sh using the following commands:

```bash
$ docker build -t grymestone:tensorpipeline01 .
$ chmod +x pipeline.sh
$ ./pipeline.sh
```

This will start a Docker container that downloads a series of scripts from the MIL, as well as downloading the needed dependencies and tensorflow object detection repository. This container will drop you into "home/tensorflow/models/research/object_detection" directory. There are a few important scripts here, so let's take some time to go through them.

### export_inference_graph.py

This script generates an inference graph, something needed for training a network. The script uses the tensor graph of the model to generate a inference graph, checkpoint models, and a frozen graph. This is essentially the tool that allows us to freeze certain points of the graph and target the last few layers for our training. This is run to create our files from a model that are then put into that model's directory before training. A typical example of running this script in the command line is below:

```bash
$ python export_inference_graph.py --input_type image_tensor --pipeline_config_path transfer_learning/models/faster_rcnn_resnet101_coco_2018_01_28/pipeline.config --trained_checkpoint_prefix transfer_learning/models/faster_rcnn_resnet101_coco_2018_01_28/model.ckpt-25165 --output_directory inference_graph/
```
Change the last four-five numbers of the model.ckpt to match whatever checkpoint you wish to freeze the graph at. Move these files into your model's directory (the same directory you found your pipeline.config file in) and you are good to go. 

### model_main.py

This is the actual training script. It takes in our pipeline.config and the model directory. Then it runs! This script has a bunch of dependencies and is the entire reason we clone the whole Tensorflow repo when we run this, which in turn is why we chose to use Docker to deploy this package. You can define how many steps you want the training script to run by changing the command line argument, but generally you won't run this through the command line. We have a bash script setup in a directory. 

## labelbox_dateset_processing

Inside this folder you will find several useful scripts used to go from our json file we downloaded from labelbox, to tfrecords useable by tensorflow! Now it should be noted that labelbox currently allows you to download the tfrecords straight from the browser, however this is not advised. Labelbox generates tfrecords for each individual image/label pair. We want all of our these image/label pairs to be included in two big tfrecord files: one for training and the other for testing. Let's go through these scripts in detail so we understand whats going on with each.

### cvt_pascal.py

This script downloads all of the images and xml files using the json script from labelbox to gather all of our data onto the container. It then puts this data into the PascalVOC format for easy processing. It creates two folders for these files.

### split_data.py

This script splits the xmls and images into a 80/20 split. 80% of the data goes to training, 20% to validation. It creates two folders and places the images into these. 

### xml_to_csv.py

Takes the xml files and puts them into the CSV format for generate_tfrecord.py

### generate_tfrecord.py

Takes the CSV files and image/label directories and generates the two tfrecords we need for the trainer. Currently you have to manually edit this file to add the classes you need. In the future this should change to be automated based on the labelmap.pbtxt file you also have to manually edit. This does make a good place to change what specific segments of your dataset you want to train on however. For example if my labelled data had 25 different classes labelled, but I only cared about one, I could only process the labels that have te correct name. An example of editing this file is as follows. Please note that the strings used must **EXACTLY** match the labels used in labelbox.

If I wanted to go from training a network to find roulette_wheels and instead find the entire NaviGator course, I would have to change the way the tfrecord interprets the labels. The example change would be the following:

#### Initial
```python
def class_text_to_int(row_label):
    if row_label == 'Wheel':
        return 1
    else:
        None
```
#### Modified

```python
def class_text_to_int(row_label):
    if row_label == 'Red Totem':
        return 1
    elif row_label == 'Blue Totem':
        return 2
    elif row_label == 'Green Totem':
        return 3
    elif row_label == 'Yellow Totem':
        return 4
    elif row_label == 'Black Totem':
        return 5
    elif row_label == 'Black Buoy':
        return 6
    else:
        None
```

### labelmap.pbtxt

Currently you have to manually edit this file as well, which is unfortunate. Ideally only one of these files should be manually edited. This contains all the labels and matching ID's you will be using for your identification. This is found in the data directory, which is likely one directory above the one you are currently in (you should see a data folder, a labelbox_data_processing folder, and a train.sh script. You'll also need to make a models folder if one is not present there and place a model in it that you want to retrain). An example of this file is the following:

#### Equivalent labelmap for the first block from generate_tfrecord.py
```
item {
  id: 1
  name: 'Wheel'
}
```

#### Equivalent labelmap for the second block from generate_tfrecord.py
```
item {
  id: 1
  name: 'Red Totem'
  id: 2
  name: 'Blue Totem'
  id: 3
  name: 'Green Totem'
  id: 4
  name: 'Yellow Totem'
  id: 5
  name: 'Black Totem'
  id: 6
  name: 'Black Buoy'
}
```

### Data! 

Depending on the approach you opted for, you either downloaded a json in the XY format from labelbox, or you downloaded the VOC format. If you downloaded the VOC format, you will need to separate the XML and PNG files into Annotation and Images folders respectively (this will be done automagically in the future). If you downloaded a json, put it into the json_files folder! This should be called data.json. If there is currently a json in there, rename it to correctly match the dataset it reflects or delete it. It has likely been stored somewhere else already.

### process_images.sh

Gathers all these scripts and runs them procedurally for you (aside from editing the labelmap.pbtxt and generate_tfrecord.py files)! Handles all the grunt work of passing the paths for our images and labels and outputs two nice files for you! If you enable the cleanup option it will then delete the created directories and images, though this isn't strictly necessary on a docker container. These tfrecords you will need later. 

## transfer_learning

The only other directory we care about here is the transfer_learning dir, so go ahead and move into this directory. At this stage you are presented with two directories, data and models, and a shell script, train.sh. The models directory is where we put our models and the data directory is where we put our tf_records and labelmap.pbtxt. Models can be downloaded from the tensorflow model zoo, found on their github and as long as you modify the pipeline config in each of them, it doesn't matter which model you choose! The methods are all the same. We have already discussed how the tf_records are generated, so we will move on and discuss the most important file in this whole mix: the pipeline.config. 

### pipeline.config

This file tells model_main.py where to look for the data, how many steps to train, which equations to use when training, how many objects you are trying to classify and so on. This essentially unifies all of our options into one easy to use config file! (Easy is maybe not the right word). Since this is always being updated by Tensorflow, it will likely remain a manually edited file for the forseeable future, rather than one stored on the MIL fileservers or github. 

All we really care about are a few lines here anyway. You'll want to look at the following variables and change them to match your specific file: 

1. num_classes
	- This should be the number of different objects you are trying to detect. If I only want to train my network to detect roulette wheels for example, this would be num_classes: 1. If I wanted to detect the entire NaviGator obstacle course, this would be num_classes: 25. This needs to be edited everytime you changed datasets. 
2. fine_tune_checkpoint
	- Here we define where the checkpoint is located to start training. This checkpoint is generated from the export_inference_graph we defined earlier and essentially tells the trainer where to start the transfer learning process. Tensorflow doesn't call it "transfer learning", instead they call it "fine tuning". 
3. label_map_path (found twice for the training and validation sets)
	- The label_map_path should be the same for both instances of this variable in the config and should point to the labelmap.pbtxt found in the data folder. Remember that these are absolute paths from the home directory so your path should look something similar to this ""/tensorflow/models/research/object_detection/transfer_learning/data/labelmap.pbtxt" Generally this won't need to be changed.
4. input_reader (occurs twice, once in train_input_reader, again in eval_input_reader)
	- This should be the absolute path to the tfrecord files you have in the data directory. The input_reader for the train_input_reader block should point to the train.record file and the eval_input_reader should point to the test.record file. Generally you won't have to edit this. 

And thats about it! You should be able to run the train.sh script now and begin training your network. If everything was setup correctly you should be good to go. If, however, you do end up with errors, here are some common fixes.

### train.sh

Run this after all the above steps have been completed and you are good to go!

## Common Errors

### Any kind of array mismatch, label mismatch, and so on

These are commonly caused by you leaving a trained model or checkpoints from a previous attempt in the directory. The trainer is finding these checkpoints and attempting to start the network on those. If the architecture has changed (say I was training a network to identify one object (class) and now I want to train a network that identifies 15 objects (classes) then the architectures would be a mismatch and this would error out). We avoid this issue in the starting phases of the training by using the inference graph. Generally speaking you just want to start with a fresh network everytime you do this, so either clone the model again or delete the existing checkpoints. 

You may get an error when changing the batch size of your pipeline.config. This is likely caused by the type of resizer you have at the top of the config file. If you have a keep_aspect_ratio_resizer, you need to change it to a fixed_shape_resizer and the change the max_size and min_size variables to width and height respectively. If you have a fixed_shape_resizer, then the issue lies somewhere else. 

You may occasionally run out of memory training a model. This means you either need to reduce the batch size or the image size. 

If you get a infinite number or nan tensor at any point in the training it means one of two things. One: Your model's batch size is too small or the learning rate is too high and the training algorithm exploded. Two: You have a broken label somewhere and you need to fix it! If option one is your problem then look to your pipeline.config and make the necessary edits. If that fails then it would seem your problem lies in the labels. This is a much more annoying problem to fix and means that our labelbox_data_processing pipeline has a leak. Find this leak and purge it in the name of the Omnissiah and the Machine Spirits shall be please with you!


