#!/usr/bin/python
import os
import signal
import subprocess

import rospy
from std_srvs.srv import SetBool, SetBoolResponse

"""
This is the main script for running Tensorflow inference. It a hacky
solution to the problem that tensorflow leaves loaded graphs inside the GPU,
even on 'session.close()'... And only clears it after processes has died.
Hence, this script opens a ROS Enable service, that will open up a subprocess
on enable, and close it on disable.
"""

# Exec instructions
exec_names = ["vamp", "stake", "garlic"]

# Store subprocesses in global
alive_processes = {}


class launcher:
    def __init__(self, name):
        """
        Object to store service threads and name of executable
        param name: executable name assumed to be in sub8_perception pkg
                    which also enable a SetBoot ROS service under the path
                    /{param name}/enable
        """
        self.name = name
        rospy.Service(f"/vision/{name}/enable", SetBool, self.enable_callback)

    def enable_callback(self, srv):
        global alive_processes
        if srv.data:
            rospy.loginfo(f"Enabling {self.name}")
            # Open up a subprocess under PID
            alive_processes[self.name] = subprocess.Popen(
                f"rosrun sub8_perception localizer.py --{self.name}",
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
            )
        else:
            rospy.loginfo(f"Disabled {self.name}")
            # Kill subprocess under PID
            os.killpg(os.getpgid(alive_processes[self.name].pid), signal.SIGTERM)

        return SetBoolResponse(success=True)


if __name__ == "__main__":
    rospy.init_node("object_detection_tf", anonymous=True)
    meh = []
    for name in exec_names:
        meh.append(launcher(name))

    rospy.spin()
