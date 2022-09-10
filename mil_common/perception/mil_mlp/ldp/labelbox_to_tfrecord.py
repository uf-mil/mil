#! /usr/bin/env python3

import os
import shutil

import generate_tfrecord as gtfr
import pandas as pd
import tensorflow as tf
from __init__ import json_to_pascal, split_data, xml_to_csv

flags = tf.app.flags
flags.DEFINE_boolean("resize", False, "Resize images? Boolean.")
flags.DEFINE_string("labelbox_format", "json", "Only supports json XY or PASCAL inputs")
flags.DEFINE_string("json_path", "", "Path to json file")
flags.DEFINE_string("csv_input", "default", "Path to the CSV input")
flags.DEFINE_string("ann_dir", "", "Path to XML directory")
flags.DEFINE_string("image_dir", "default", "Path to the image directory")
flags.DEFINE_string("output_path", "default", "Path to output TFRecord")
flags.DEFINE_string("labelmap_path", "", "Path to labelmap.pbtxt")
flags.DEFINE_boolean("cleanup", False, "Delete all files? Boolean.")
flags.DEFINE_boolean("check_tfrecords", False, "If true, print tfrecords.")
FLAGS = flags.FLAGS


def main(_):
    if FLAGS.labelbox_format == "json":
        json_path = FLAGS.json_path
        json_to_pascal(json_path)
        split_data(resize=FLAGS.resize)

    elif FLAGS.labelbox_format == "PASCAL":
        image_dir = FLAGS.image_dir
        ann_dir = FLAGS.ann_dir
        split_data(image_dir, ann_dir, resize=FLAGS.resize)

    for folder in ["train", "test"]:
        labelmap_name = FLAGS.labelmap_path
        # image_path = os.path.join(os.getcwd(), ('images/' + folder))
        xml_df, num_of_images = xml_to_csv(folder, labelmap_name, resize=FLAGS.resize)
        xml_df.to_csv((folder + "_labels.csv"), index=None)
        # xml_df.to_csv(('images/' + folder + '_labels.csv'), index=None)
        print("Successfully converted xml to csv.")
        print(" Number of Images in: ", folder, num_of_images)
        # Run Generate_tfrecords
        gtfr.create_dict()
        if FLAGS.output_path == "default":
            writer = tf.python_io.TFRecordWriter(
                "../docker_tf/transfer_learning/data/" + (folder + ".record")
            )
        else:
            writer = tf.python_io.TFRecordWriter(FLAGS.output_path)
        if FLAGS.image_dir == "default":
            path = os.path.join(os.getcwd(), folder)
        else:
            path = os.path.join(os.getcwd(), FLAGS.image_dir)
        if FLAGS.csv_input == "default":
            examples = pd.read_csv(folder + "_labels.csv")
        else:
            examples = pd.read_csv(FLAGS.csv_input)
        grouped = gtfr.split(examples, "filename")
        for group in grouped:
            tf_example = gtfr.create_tf_example(group, path)
            writer.write(tf_example.SerializeToString())

        writer.close()
        output_path = os.path.join(os.getcwd(), FLAGS.output_path)
        print(f"Successfully created the TFRecords: {output_path}")

        if FLAGS.check_tfrecords:
            print(
                "Checking Validity of TFRecords. Expect image encoding alongside label data."
            )
            if FLAGS.output_path == "default":
                for example in tf.python_io.tf_record_iterator(
                    "../docker_tf/transfer_learning/data/" + (folder + ".record")
                ):
                    result = tf.train.Example.FromString(example)
                    print(result)
            else:
                for example in tf.python_io.tf_record_iterator(FLAGS.output_path):
                    result = tf.train.Example.FromString(example)
                    print(result)
    # Clean up
    if FLAGS.cleanup:
        try:
            if FLAGS.csv_input == "default":
                os.remove("test_labels.csv")
                os.remove("train_labels.csv")
            else:
                os.remove(FLAGS.csv_input)
            if FLAGS.image_dir != "default" or FLAGS.ann_dir != "default":
                shutil.rmtree(FLAGS.image_dir)
                shutil.rmtree(FLAGS.ann_dir)
            else:
                shutil.rmtree("Images/")
                shutil.rmtree("Annotations/")
            shutil.rmtree("test/")
            shutil.rmtree("train/")
        except Exception as e:
            print(
                "Cleanup failed, either a directory or file was missing. Could indicate failed download.",
                e,
            )


if __name__ == "__main__":
    tf.app.run()
