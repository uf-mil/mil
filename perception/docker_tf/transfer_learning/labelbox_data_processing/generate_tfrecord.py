"""
Usage:
  # From tensorflow/models/
  # Create train data:
  python generate_tfrecord.py --csv_input=train/train_labels.csv --image_dir=train --output_path=train.record

  # Create test data:
  python generate_tfrecord.py --csv_input=test/test_labels.csv  --image_dir=test --output_path=test.record
"""
from __future__ import division
from __future__ import print_function
from __future__ import absolute_import

import os
import io
import sys
import pandas as pd
import tensorflow as tf
from PIL import Image
from collections import namedtuple, OrderedDict

flags = tf.app.flags
# flags.DEFINE_string('csv_input', '', 'Path to the CSV input')
# flags.DEFINE_string('image_dir', '', 'Path to the image directory')
# flags.DEFINE_string('output_path', '', 'Path to output TFRecord')
# flags.DEFINE_string('labelmap_path', '', 'Path to labelmap.pbtxt')
FLAGS = flags.FLAGS
# dictionary = {}

# TO-DO replace this with label map


def create_dict():
    global dictionary
    with open(('../data/' + FLAGS.labelmap_path)) as f:
        txt = f.read()
    labels = []
    ids = []
    # print(txt)
    # txt = txt[2, :]
    full_split = [s.strip().split(': ') for s in txt.splitlines()]
    # print(full_split)
    full_split = full_split[1:]
    # try:
    for i in full_split:
        if len(i) < 2:
            continue
        if isinstance(i[1], str):
            if i[1].isdigit():
                # print(i[1])
                ids.append(int(i[1]))
            else:
                # print(i[1].strip("'"))
                labels.append(i[1].strip("'"))
        else:
            print(
                "Error, incorrect key located in labelmap. Should be only id or name. Instead found: ", i[1])
    # except:
        # print("It errored! Whomp whomp. ", i)
    dictionary = dict(zip(labels, ids))
    # print(dictionary)


def class_text_to_int(row_label):
    # print(dictionary)
    # print(row_label)
    # print(dictionary.get(row_label))
    return dictionary.get(row_label)


def split(df, group):
    data = namedtuple('data', ['filename', 'object'])
    gb = df.groupby(group)
    return [data(filename, gb.get_group(x)) for filename, x in zip(gb.groups.keys(), gb.groups)]


def create_tf_example(group, path):
    with tf.gfile.GFile(os.path.join(path, '{}'.format(group.filename)), 'rb') as fid:
        encoded_jpg = fid.read()
    encoded_jpg_io = io.BytesIO(encoded_jpg)
    image = Image.open(encoded_jpg_io)
    width, height = image.size

    filename = group.filename.encode('utf8')
    image_format = b'jpg'
    xmins = []
    xmaxs = []
    ymins = []
    ymaxs = []
    classes_text = []
    classes = []

    for index, row in group.object.iterrows():
        xmins.append(row['xmin'] / width)
        xmaxs.append(row['xmax'] / width)
        ymins.append(row['ymin'] / height)
        ymaxs.append(row['ymax'] / height)
        classes_text.append(row['class'].encode('utf8'))
        new_class = class_text_to_int(row['class'])
        # print("width: ", width)
        # print("height: ", height)
        if new_class is None:
            continue
        classes.append(new_class)
    tf_example = tf.train.Example(features=tf.train.Features(feature={
        'image/height': int64_feature(height),
        'image/width': int64_feature(width),
        'image/filename': bytes_feature(filename),
        'image/source_id': bytes_feature(filename),
        'image/encoded': bytes_feature(encoded_jpg),
        'image/format': bytes_feature(image_format),
        'image/object/bbox/xmin': float_list_feature(xmins),
        'image/object/bbox/xmax': float_list_feature(xmaxs),
        'image/object/bbox/ymin': float_list_feature(ymins),
        'image/object/bbox/ymax': float_list_feature(ymaxs),
        'image/object/class/text': bytes_list_feature(classes_text),
        'image/object/class/label': int64_list_feature(classes),
    }))
    return tf_example

'''
Utility Functions for processing datasets into tf_records. These were copied from utils/dataset_util.py provided in the Tensorflow Object Detection Repo to cut down on file usages.
'''


def int64_feature(value):
    return tf.train.Feature(int64_list=tf.train.Int64List(value=[value]))


def int64_list_feature(value):
    return tf.train.Feature(int64_list=tf.train.Int64List(value=value))


def bytes_feature(value):
    return tf.train.Feature(bytes_list=tf.train.BytesList(value=[value]))


def bytes_list_feature(value):
    return tf.train.Feature(bytes_list=tf.train.BytesList(value=value))


def float_list_feature(value):
    return tf.train.Feature(float_list=tf.train.FloatList(value=value))


def read_examples_list(path):
    """Read list of training or validation examples.

    The file is assumed to contain a single example per line where the first
    token in the line is an identifier that allows us to find the image and
    annotation xml for that example.

    For example, the line:
    xyz 3
    would allow us to find files xyz.jpg and xyz.xml (the 3 would be ignored).

    Args:
      path: absolute path to examples list file.

    Returns:
      list of example identifiers (strings).
    """
    with tf.gfile.GFile(path) as fid:
        lines = fid.readlines()
    return [line.strip().split(' ')[0] for line in lines]


def recursive_parse_xml_to_dict(xml):
    """Recursively parses XML contents to python dict.

    We assume that `object` tags are the only ones that can appear
    multiple times at the same level of a tree.

    Args:
      xml: xml tree obtained by parsing XML file contents using lxml.etree

    Returns:
      Python dictionary holding XML contents.
    """
    if not xml:
        return {xml.tag: xml.text}
    result = {}
    for child in xml:
        child_result = recursive_parse_xml_to_dict(child)
        if child.tag != 'object':
            result[child.tag] = child_result[child.tag]
        else:
            if child.tag not in result:
                result[child.tag] = []
            result[child.tag].append(child_result[child.tag])
    return {xml.tag: result}
