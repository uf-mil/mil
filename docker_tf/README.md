# Transfer Learning Object Detection - A Short Tutorial

# @TODO
1. Rename the my_scripts folder to be transfer_learning
2. Add labelbox_dataset_processing folder.
3. Move generate_tfrecord & csv_to_tfrecord.sh into labelbox_dataset_processing folder
4. Edit generate_tfrecord to generate labelmap.pbtxt
5. Edit shell script to launch a new python script that edits generate_tfrecord (or loads a config that tfrecord interprets) to allow for easy class adaptation.
6. Merge csv_to_tfrecord.sh and process_images.sh
7. Configurable image download
8. In cvt_pascal.py make the json file name an input from the shell script or enforce a standard name. Temp standard: data.json.
9. Add docker build to the install script. Until then docker build must be done manually. 
10. Give Docker Image a better name.

## A Brief Tour of Tensorflow Object Detection
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
$ python export_inference_graph.py --input_type image_tensor --pipeline_config_path transfer_learning/models/faster_rcnn_network_XXX/pipeline.config --trained_checkpoint_prefix training/model.ckpt-XXXX --output_directory inference_graph/
```
Move these files into your model's directory (the same directory you found your pipeline.config file in) and you are good to go. 

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

### process_images.sh

Gathers all these scripts and runs them procedurally for you (aside from editing the labelmap.pbtxt and generate_tfrecord.py files)! Handles all the grunt work of passing the paths for our images and labels and outputs two nice files for you! If you enable the cleanup option it will then delete the created directories and images, though this isn't strictly necessary on a docker container. These tfrecords you will need later. 

## transfer_learning

The only other directory we care about here is the transfer_learning dir, so go ahead and move into this directory. At this stage you are presented with two directories, data and models, and a shell script, train.sh. The models directory is where we put our models and the data directory is where we put our tf_records and labelmap.pbtxt. Models can be downloaded from the tensorflow model zoo, found on their github and as long as you modify the pipeline config in each of them, it doesn't matter which model you choose! The methods are all the same. We have already discussed how the tf_records are generated, so we will move on and discuss the most important file in this whole mix: the pipeline.config. 

### pipeline.config

This file tells model_main.py where to look for the data, how many steps to train, which equations to use when training, how many objects you are trying to classify and so on. This essentially unifies all of our options into one easy to use config file! (Easy is maybe not the right word). 

All we really care about are a few lines here. You'll want to look at the following variables and change them to match your specific file: 

1. num_classes
	- This should be the number of different objects you are trying to detect. If I only want to train my network to detect roulette wheels for example, this would be num_classes: 1. If I wanted to detect the entire NaviGator obstacle course, this would be num_classes: 25. 
2. fine_tune_checkpoint
	- Here we define where the checkpoint is located to start training. This checkpoint is generated from the export_inference_graph we defined earlier and essentially tells the trainer where to start the transfer learning process. Tensorflow doesn't call it "transfer learning", instead they call it "fine tuning". 
3. label_map_path (found twice for the training and validation sets)
	- The label_map_path should be the same for both instances of this variable in the config and should point to the labelmap.pbtxt found in the data folder. Remember that these are absolute paths from the home directory so your path should look something similar to this ""/tensorflow/models/research/object_detection/transfer_learning/data/labelmap.pbtxt"
4. input_reader (occurs twice, once in train_input_reader, again in eval_input_reader)
	- This should be the absolute path to the tfrecord files you have in the data directory. The input_reader for the train_input_reader block should point to the train.record file and the eval_input_reader should point to the test.record file. 

And thats about it! You should be able to run the train.sh script now and begin training your network. If everything was setup correctly you should be good to go. If, however, you do end up with errors, here are some common fixes.

## Common Errors

### Any kind of array mismatch, label mismatch, and so on

These are commonly caused by you leaving a trained model or checkpoints from a previous attempt in the directory. The trainer is finding these checkpoints and attempting to start the network on those. If the architecture has changed (say I was training a network to identify one object (class) and now I want to train a network that identifies 15 objects (classes) then the architectures would be a mismatch and this would error out). We avoid this issue in the starting phases of the training by using the inference graph. Generally speaking you just want to start with a fresh network everytime you do this, so either clone the model again or delete the existing checkpoints. 


