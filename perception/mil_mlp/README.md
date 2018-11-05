# The MIL Machine Learning Pipeline (MIL_MLP) - Transfer Learning for Object Detection

# @TODO
1. Configurable image download
2. [BLOCKED BY TENSORFLOW/CUDA COMPATABILITY] Add docker build to the install script. Until then docker build must be done manually.
3. Have datasets stored on internal server so it can be downloaded.
4. Configurable resize (Low importance due to no significant performance gain)
5. Configurable CSV input (Currently it still assumes you are handing it a json or PASCAL so it will error out if
those are not provided.)
6. Add python dependencies to the install script.

## What does this do?
This package is designed to fully automate the process of training neural networks for object detection. One of the major challenges that comes with training networks is ensuring data integrity. This pipeline was developed for use with Tensorflow and the Labelbox labelling tool. The pipeline takes in a JSON or PASCAL VOC dataset from labelbox and processes it into a TFRecord ready for Tensorflow, eliminating superfluous or bad labels and ensuring the integrity of the dataset.

## Quick Start Guide

1. Process the Data:

NOTE: USE PATHS FROM THE DIRECTORY YOU ARE IN, OR USE ABSOLUTE PATHS.

```bash
rosrun mil_mlp labelbox_to_tfrecord.py --json_path='json_files/JSON_NAME.json --labelmap_path='../docker_tf/transfer_learning/data/LABELMAPNAME.pbtxt
```
2. Run the docker image
```bash
rosrun mil_mlp pipeline.sh
```
3. Run the trainer
```bash
cd transfer_learning
./train.sh
```
4. Export the frozen graph from trained model.
```bash
 python export_inference_graph.py --input_type image_tensor --pipeline_config_path transfer_learning/models/MODEL_DIR/pipeline.config --trained_checkpoint_prefix transfer_learning/models/MODEL_DIR/model.ckpt-XXXX --output_directory transfer_learning/inference_graph/
```

## Detailed Walkthrough

### I. The Code Structure
This package is broken up into two distinct parts: Data Processing, and Network Training. Inside of Data Processing you will find all the tools necessary to create TF Records from labelled data. The scripts and functions within automatically trim the data and validate it, ensuring that it is suitable for training. More details on those in part II.

This pipeline is built upon Docker Images for easy deployment and updating. Currently, we build and update off of 3 distinct Docker Images. The first image is the CUDA 10.0 docker image distributed by NVIDIA. This simply installs CUDA drivers into the image and allows us to communicate with the card. The second image we  run is a custom tensorflow 9.0 image, compiled using the dockerfiles made availble to us from Tensorflow. We had to configure the compilation to target CUDA 10.0 instead of CUDA 9.0, which is the default. CUDA 10.0 is not officially supported by TF as of yet, however it is the only version of CUDA our graphics card supports. The final image is the docker image that loads our custom scripts and the Models directory of Tensorflow. Eventually, these will all be merged into a single docker image, however this allows us to update CUDA, Tensorflow, and our scripts separately, without having to recompile everything each time.

### II. The Data: LabelBox Data Processing (ldp)
The entire purpose of this pipeline is to make a streamlined data processing pipeline. Tensorflow has done a great job of ensuring it will work so long as you give it good data. That is our point of failure and thus the first thing you should do if your network fails to train is to check the data.
To that end we compiled, and when necessary created, scripts to handle the process of data verification and processing.
This process starts with labelling images on the service Labelbox, which as it currently stands is the standard for image labeling.
Once the data is labelled you can then download this image set in a variety of formats. This pipeline currently supports two:

1. JSON
2. PASCAL VOC

Of these two, it has been most extensively tested with JSON files and thus we recommend downloading the dataset in this format.
JSON files for MIL projects are found in their respective repositories (i.e. the NaviGator and SubjuGator repositories).

#### A. labelbox_to_tfrecord.py

process.py is the main script through which data is processed. This initiates the process of converting a JSON or PASCAL VOC format into the tf_records needed for a specific project.
It takes in a variety of commandline arguments and are as follows:

```bash
python labelbox_to_tfrecord.py --resize=False --labelbox_format=json --json_name='json_files/project_labels.json' --csv_input=default --ann_dir=default --image_dir=default --output_path=default --labelmap_path='../data/totems&buoys.pbtxt' --cleanup=False
```
Now most of these arguments have default settings, so you really only need to pass in the ones that specifically apply to you. Examples would be the json_path or image_dir and ann_dir which must have inputs depending on what labelbox_format you passed in. If I only wanted to process a json image I would only need the following:
```bash
python labelbox_to_tfrecord.py --json_name='json_files/project_labels.json' --labelmap_path='../data/totems&buoys.pbtxt'
```

Below is the actual code block that handles these inputs and may provide a clearer idea of which values have defaults and which ones are needed.

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

Note that this repo does not make the models or data directory in docker_tf/transfer_learning. Both of those must be present for this to work by default. Teh data directory holds the TFRecords. The models directory holds a pretrained model from the tensorflow model zoo you chose to download. The only files that should be in there are the three model.ckpt files and the pipeline.config. All other files should be deleted.

Further, the dataset is automatically trimmed of any labels that are not present inside the pbtxt. If, for example, I have an image that has labels for buoys and totems, but my labelmap only has the labels for colored shapes inside, the image will be tossed out of the dataset and not processed.

#### B. ldp_utils.py

ldp_utils contains all the necessary utility functions that were written to process the data from a json, csv, or pascal format into tf records. TF records are what tensorflow is able to interpret and process. While it is currently possible to download the TF records from Labelbox, the method they use to create the TF records results in each image getting its own record file. This is not ideal as it increases clutter and would require several changes to our scripts which assume only two TF records are being used.

More details on what the utility functions in ldp_utils.py are found inside the file in comments.

#### C. generate_tfrecord.py

The file that handles the actual conversion to TF records. This is a file taken from the main tensorflow repo and edited to fit nicely into our system. We wanted to be able to process data outside of the docker container and thus we included this utility file in our repo, with all functions necessary for its operation now self contained as opposed to parcelled out amongst utility files in the Tensorflow repo. More details on this found in the comments of the file itself. The major modification we made to this file was to allow it to take in a labelmap input and figure out what the id and name mapping is based on that.

#### D. labelbox2pascal

This directory houses the repo cloned from Labelbox. It contains the tools necessary to download images from labelbox using their json file along with a variety of other useful utils for parsing their json files. At the time of writing this couldn't be installed through pip and it was easier to just clone it into our directory. We only needed the labelbox2pascal functionality of the labelbox toolset, so the entire repo was not cloned. These files are no longer available on the repo and have been depricated. Eventually we may move to using labelbox's own outputs but for now our pipeline needs these utilities to process the json.

### III. The Docker Images: docker_tf and transfer_learning

These directories house the Dockerfile for our image, the pipeline.sh and train.sh scripts, along with our models and processed data. The docker image itself relies on two other docker images. The first image it relies upon is the Nvidia CUDA 10 docker image that they provide. This allows us to interface with our 2080 GPU. CUDA 10 will be necessary for any machine that wishes to interface with the 20xx series of cards.

The second image we use is a custom configured tensorflow 1.9 docker container. To create this container, one needs to clone the tensorflow 1.9 repo and use the tools provided to generate a docker image of it. We use the tensorflow-gpu-devel Dockerfile. Follow the instructions in the readme and it should be relatively trivial to do. Make sure that when you run their script for generating the docker image you specify to use CUDA 10. This will ensure it uses the CUDA 10 Nvidia image as a base and that when tensorflow compiles in this image it will be configured to work with CUDA 10. Eventually when tensorflow updates to use CUDA 10 we will use their own distributed docker image.

With that done, the only thing left to do is to navigate to this directory and run

```bash
docker build -t 'whatever you want to name the image' .
```
This will build our image and assuming you get no errors you are ready to go! You may need to edit the name of the tensorflow image depending on what you chose to name the one you compiled from tensorflow using their script. You can specify this in command line. Again follow their instructions on how to do so.

NOTE: The models we use for transfer learning were compiled in tensorflow 1.9! They are no longer compatable with Tensorflow 1.12+! Until these models are updated you must keep your tensorflow version at 1.11 or lower!

```bash
pip install tensorflow==1.9.0
```
#### pipeline.sh

Once you get your docker image compiled and ready to go, you can run pipeline.sh. This will launch the image (you may need to edit the script to point to the docker image yuo created if you named it something different from ours). This places you in the Object Detection directory of the Tensorflow repo. If all went well you should see a folder in there called transfer_learning. This holds our models and scripts.

#### transfer_learning

This directory holds the models and data we will be training with. This directory is the only one saved after the container is closed, so save any frozen inference graphs in this folder!

##### A. The Models

The models directory contains the models that you pulled from the tensorflow model zoo. As noted above these have not been updated in months at the time of writing this (Nov. 2nd, 2018). As such, we must use an older version of tensorflow to properly use these models. I recommend using 1.9 as that seems to be the version these models were compiled in, but theoretically you could go up to 1.11.

Once you have one of these models downloaded, you will need to remove everything except for the model.ckpt files and the pipeline.config. The pipeline.config may have been updated and it would behoove you to check this repo for updates: https://github.com/tensorflow/models/tree/master/research/object_detection/samples/configs

Replace the pipeline.config file with the corresponding config in the above link if necessary. Make sure the file name is still pipeline.config however as it is easier to work with. Inside of this file you will need to add the absolute paths to the train.record, test.record, and labelmap.pbtxt you will be using. These vary depending on what choices you have made perivously but should be fairly simple to figure out. Also make sure that you update the use checkpoint path to the model.cpkt in this directory! This will allow us to perform the transferlearning part of this shindig. The lines you need to change are clearly marked in caps.


One final thing to note is that the number of classes should be changed to match the number of entities in the labelmap.pbtxt and under the eval_config: {} section of the config you will need to change num_examples to match the number of testing data you have in your dataset. This is printed after completing the ldp script.

##### B. The Data

This folder is populated with the record files you will have generated from the ldp scripts. If they aren't there, an error occurred in ldp or you did not specify the correct directory.

##### C. train.sh

train.sh is what you will run once inside the docker container to actually begin the process of training the network. You will need to change the paths located inside this script to point to the appropriate model directory before you run it though!

##### D. Graphs of Inference!

This script technically isn't in this repo but is important! The script is found when running the docker container in the object detection directory. It generates an inference graph, which is what you need to use the network once it is trained. The script uses the tensor graph of the model to generate a inference graph, checkpoint models, and a frozen graph. This is essentially the tool that allows us to run images through this graph and get results out of it. A typical example of running this script in the command line is below:

```bash
python export_inference_graph.py --input_type image_tensor --pipeline_config_path transfer_learning/models/faster_rcnn_resnet101_coco_2018_01_28/pipeline.config --trained_checkpoint_prefix transfer_learning/models/faster_rcnn_resnet101_coco_2018_01_28/model.ckpt-25165 --output_directory inference_graph/
 ```
Change the last four-five numbers of the model.ckpt to match whatever checkpoint you wish to freeze the graph at. Move these files into the transfer learning directory for future use.

### Common Errors, Tips, Tricks, etc etc etc...

When running the process_data script for the first time, if you encounter an sklearn import error, it is because you need version 0.18 of scikit-learn for python 2. The installation is fairly simple:

```python
pip2 install scikit-learn==0.18
```
For formatting your pbtxt, look around online at COCO dataset labelmaps or any other from the official repo. You could also look into the NaviGator repo in perception/navigator_vision/datasets and check out our labelmaps there for reference. Note we don't use name and display name which you might find some data sets doing. We only use name.

If you ran out of memory while training, you should reduce batch size or image size in the config file.

If you get an infinite numebr or NaN tensor at any point, it means one of two things. One: Your model's batch size was too small or learning rate too high and the algorithm exploded. Two: You have a bad label somewhere in the data set and it needs to be fixed!

Changing the batch size of the pipeline config may throw an error if you are using a keen_aspect_ratio_resizer. To tell if you are using such a resizer, scroll to the top of your config file and check! If you are, you will need to change to a fixed_shape_resizer to adjust the batch size. This also means you will have to change the max_size and min_size variables to instead be named width and height respectively.

If something else broke, its probably your data. Run the verify_data.py to check all the labels and images. If they look correct, check the csv files and the tfrecords. Something may still be going wrong there. Tensorflow has done a good job of making sure the only issues you will run into is your own data. If that isn't the case, posting it on the Tensorflow github page as an issue may be of some help.

If the problem does indeed lie in the data, it is likely the our ldp is broken somewhere. Finding the leak and fixing it would be greatly appreciated! The Machine Spirits will indeed be pleased.


