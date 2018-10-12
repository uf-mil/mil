import os
import glob
import pandas as pd
import shutil
import xml.etree.ElementTree as ET
from __init__ import xml_to_csv, json_to_pascal, split_data
import tensorflow as tf

flags = tf.app.flags
flags.DEFINE_string('labelbox_format', 'json',
                    'Only supports json XY or PASCAL inputs')
flags.DEFINE_string('json_name', '', 'Path to json file')
flags.DEFINE_string('csv_input', '', 'Path to the CSV input')
flags.DEFINE_string('ann_dir', '', 'Path to XML directory')
flags.DEFINE_string('image_dir', '', 'Path to the image directory')
flags.DEFINE_string('output_path', '', 'Path to output TFRecord')
flags.DEFINE_string('labelmap_path', '', 'Path to labelmap.pbtxt')
flags.DEFINE_boolean('cleanup', True, 'Delete all files?')
FLAGS = flags.FLAGS


def main():
    if data == 1:
        json_name = FLAGS.json_name
        json_to_pascal(json_name)
        split_data()

    elif data == 2:
        image_dir = FLAGS.image_dir
        ann_dir = FLAGS.ann_dir
        split_data(image_dir, ann_dir)

    for folder in ['train', 'test']:
        labelmap_name = FLAGS.labelmap_path
        #image_path = os.path.join(os.getcwd(), ('images/' + folder))
        xml_df, num_of_images = xml_to_csv(folder, labelmap_name)
        xml_df.to_csv((folder + '_labels.csv'), index=None)
        #xml_df.to_csv(('images/' + folder + '_labels.csv'), index=None)
        print('Successfully converted xml to csv.')
        print(" Number of Images in: ", folder, num_of_images)

        # Clean up
        if FLAGS.cleanup:
            try:
                os.remove('test.csv')
                os.remove('train.csv')
                shutil.rmtree('Images/')
                shutil.rmtree('Annotations/')
                shutil.rmtree('test/')
                shutil.rmtree('train/')
            except:
                Print(
                    "Cleanup failed, either a directory or file was missing. Could indicate failed download.")
if __name__ == '__main__':
    main()
