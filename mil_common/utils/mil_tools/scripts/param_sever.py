#!/usr/bin/env python3
import os
import random
import sys

import rclpy
import yaml

rclpy.init(args=sys.argv)
node = rclpy.create_node("param_saver")


class MyDumper(yaml.Dumper):
    def represent_mapping(self, tag, mapping, flow_style=False):
        return yaml.Dumper.represent_mapping(self, tag, mapping, flow_style)


while not rclpy.is_shutdown():
    rclpy.sleep(rclpy.Duration(3))

    entries = node.declare_parameter("~")
    for entry in entries.values():
        filename = entry["filename"]
        file_basepath = entry["file_basepath"]
        param_paths = entry["param_paths"]

        p = node.declare_parameter(file_basepath)
        data = yaml.dump(
            {k: v for k, v in p.items() if k in param_paths},
            Dumper=MyDumper,
        )

        if os.path.exists(filename):
            with open(filename, "rb") as f:
                if f.read() == data:
                    continue

        tmp_filename = "%s.%i" % (filename, random.randrange(10000))
        with open(tmp_filename, "wb") as f:
            f.write(data)
        os.rename(tmp_filename, filename)

        print(f"Saved {file_basepath} to {filename}")
