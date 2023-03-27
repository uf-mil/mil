#!/usr/bin/env python3
import os
import random

import rospy
import yaml

rospy.init_node("param_saver", anonymous=True)


class MyDumper(yaml.Dumper):
    def represent_mapping(self, tag, mapping, flow_style=False):
        return yaml.Dumper.represent_mapping(self, tag, mapping, flow_style)


while not rospy.is_shutdown():
    rospy.sleep(rospy.Duration(3))

    entries = rospy.get_param("~")
    for entry in entries.values():
        filename = entry["filename"]
        file_basepath = entry["file_basepath"]
        param_paths = entry["param_paths"]

        p = rospy.get_param(file_basepath)
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
