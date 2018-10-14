from __future__ import division
from __future__ import print_function
from __future__ import absolute_import

import os
import glob
import pandas as pd
import shutil
import xml.etree.ElementTree as ET
from __init__ import xml_to_csv, json_to_pascal, split_data
import tensorflow as tf
import generate_tfrecord as gtfr

flags = tf.app.flags
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
FLAGS = flags.FLAGS


def main(_):
    if FLAGS.labelbox_format is 'json':
        json_name = FLAGS.json_name
        # json_to_pascal(json_name)
        split_data(resize=FLAGS.resize)

    elif FLAGS.labelbox_format is 'PASCAL':
        image_dir = FLAGS.image_dir
        ann_dir = FLAGS.ann_dir
        split_data(image_dir, ann_dir, resize=FLAGS.resize)

    for folder in ['train', 'test']:
        labelmap_name = FLAGS.labelmap_path
        #image_path = os.path.join(os.getcwd(), ('images/' + folder))
        xml_df, num_of_images = xml_to_csv(
            folder, labelmap_name, resize=FLAGS.resize)
        xml_df.to_csv((folder + '_labels.csv'), index=None)
        #xml_df.to_csv(('images/' + folder + '_labels.csv'), index=None)
        print('Successfully converted xml to csv.')
        print(" Number of Images in: ", folder, num_of_images)
        # Run Generate_tfrecords
        gtfr.create_dict()
        if FLAGS.output_path is 'default':
            writer = tf.python_io.TFRecordWriter(
                '../data/' + (folder + '.record'))
        else:
            writer = tf.python_io.TFRecordWriter(FLAGS.output_path)
        if FLAGS.image_dir is 'default':
            path = os.path.join(os.getcwd(), folder)
        else:
            path = os.path.join(os.getcwd(), FLAGS.image_dir)
        if FLAGS.csv_input is 'default':
            examples = pd.read_csv((folder + '_labels.csv'))
        else:
            examples = pd.read_csv(FLAGS.csv_input)
        grouped = gtfr.split(examples, 'filename')
        for group in grouped:
            tf_example = gtfr.create_tf_example(group, path)
            writer.write(tf_example.SerializeToString())

        writer.close()
        output_path = os.path.join(os.getcwd(), FLAGS.output_path)
        print('Successfully created the TFRecords: {}'.format(output_path))

    # Clean up
    if FLAGS.cleanup:
        try:
            os.remove('test_labels.csv')
            os.remove('train_labels.csv')
            shutil.rmtree('Images/')
            shutil.rmtree('Annotations/')
            shutil.rmtree('test/')
            shutil.rmtree('train/')
        except Exception as e:
            print(
                "Cleanup failed, either a directory or file was missing. Could indicate failed download.", e)
if __name__ == '__main__':
    tf.app.run()
    # main()
