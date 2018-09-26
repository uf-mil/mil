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
from object_detection.utils import dataset_util
from collections import namedtuple, OrderedDict

flags = tf.app.flags
flags.DEFINE_string('csv_input', '', 'Path to the CSV input')
flags.DEFINE_string('image_dir', '', 'Path to the image directory')
flags.DEFINE_string('output_path', '', 'Path to output TFRecord')
FLAGS = flags.FLAGS


# TO-DO replace this with label map
def class_text_to_int(row_label):
    if row_label == 'None':
        return 1
    elif row_label == 'target_large':
        return 2
    elif row_label == 'target_small':
        return 3
    elif row_label == 'triangle_red':
        return 4
    elif row_label == 'totem_blue':
        return 5
    elif row_label == 'cruciform_green':
        return 6
    elif row_label == 'buoy_black':
        return 7
    elif row_label == 'totem_red':
        return 8
    elif row_label == 'totem_yellow':
        return 9
    elif row_label == 'circle_green':
        return 10
    elif row_label == 'totem_green':
        return 11
    elif row_label == 'totem_white':
        return 12
    elif row_label == 'triangle_blue':
        return 13
    elif row_label == 'stc_platform':
        return 14
    elif row_label == 'stc_panel_off':
        return 15
    elif row_label == 'totem_black':
        return 16
    elif row_label == 'stc_panel_yellow':
        return 17
    elif row_label == 'stc_panel_green':
        return 18
    elif row_label == 'stc_panel_blue':
        return 19
    elif row_label == 'stc_panel_red':
        return 20
    elif row_label == 'triangle_green':
        return 21
    elif row_label == 'circle_blue':
        return 22
    elif row_label == 'circle_red':
        return 23
    elif row_label == 'cruciform_blue':
        return 24
    elif row_label == 'dock_2018_platform':
        return 25
    else:
        None


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
        classes.append(class_text_to_int(row['class']))

    tf_example = tf.train.Example(features=tf.train.Features(feature={
        'image/height': dataset_util.int64_feature(height),
        'image/width': dataset_util.int64_feature(width),
        'image/filename': dataset_util.bytes_feature(filename),
        'image/source_id': dataset_util.bytes_feature(filename),
        'image/encoded': dataset_util.bytes_feature(encoded_jpg),
        'image/format': dataset_util.bytes_feature(image_format),
        'image/object/bbox/xmin': dataset_util.float_list_feature(xmins),
        'image/object/bbox/xmax': dataset_util.float_list_feature(xmaxs),
        'image/object/bbox/ymin': dataset_util.float_list_feature(ymins),
        'image/object/bbox/ymax': dataset_util.float_list_feature(ymaxs),
        'image/object/class/text': dataset_util.bytes_list_feature(classes_text),
        'image/object/class/label': dataset_util.int64_list_feature(classes),
    }))
    return tf_example


def main(_):
    writer = tf.python_io.TFRecordWriter(FLAGS.output_path)
    path = os.path.join(os.getcwd(), FLAGS.image_dir)
    examples = pd.read_csv(FLAGS.csv_input)
    grouped = split(examples, 'filename')
    for group in grouped:
        tf_example = create_tf_example(group, path)
        writer.write(tf_example.SerializeToString())

    writer.close()
    output_path = os.path.join(os.getcwd(), FLAGS.output_path)
    print('Successfully created the TFRecords: {}'.format(output_path))


if __name__ == '__main__':
    tf.app.run()
