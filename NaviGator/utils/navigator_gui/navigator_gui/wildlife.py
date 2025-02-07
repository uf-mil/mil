import os

import rospkg
import rospy
from navigator_msgs.msg import ScanTheCode
from python_qt_binding import QtGui, QtWidgets, loadUi
from python_qt_binding.QtWidgets import QWidget
from qt_gui.plugin import Plugin


class WildlifeEncounterPlugin(Plugin):
    UPDATE_MILLISECONDS = 1000
    STC_TOPIC = "/stc_sequence"

    colors = {
        "R": "QWidget {background-color:#FF5733;}",  # Red for wildlife 1
        "G": "QWidget {background-color:#B1EB00;}",  # Green for wildlife 2
        "B": "QWidget {background-color:#3357FF;}",  # Blue for wildlife 3
    }

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("WildlifeEncounterPlugin")

        # Parse command-line arguments
        from argparse import ArgumentParser

        parser = ArgumentParser()
        parser.add_argument(
            "-q",
            "--quiet",
            action="store_true",
            dest="quiet",
            help="Put plugin in silent mode",
        )
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print("arguments: ", args)
            print("unknowns: ", unknowns)

        # Create QWidget
        self._widget = QWidget()
        ui_file = os.path.join(
            rospkg.RosPack().get_path("navigator_gui"),
            "resource",
            "wildlife_encounter.ui",
        )
        loadUi(ui_file, self._widget)
        self._widget.setObjectName("WildlifeEncounterUi")

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (" (%d)" % context.serial_number()),
            )

        context.add_widget(self._widget)
        self.connect_ui()

        # ROS Subscriber
        self.stc_sub = rospy.Subscriber(self.STC_TOPIC, ScanTheCode, self.update_gui)

    def connect_ui(self):
        # Find UI components for the boxes and labels
        self.color1_rect = self._widget.findChild(QtWidgets.QWidget, "color1_rect")
        self.color2_rect = self._widget.findChild(QtWidgets.QWidget, "color2_rect")
        self.color3_rect = self._widget.findChild(QtWidgets.QWidget, "color3_rect")
        self.label1 = self._widget.findChild(QtWidgets.QLabel, "label1")
        self.label2 = self._widget.findChild(QtWidgets.QLabel, "label2")
        self.label3 = self._widget.findChild(QtWidgets.QLabel, "label3")
        self.compass_label = self._widget.findChild(QtWidgets.QLabel, "compass_label")

        # Set compass icon
        compass_icon = QtGui.QPixmap(
            os.path.join(
                rospkg.RosPack().get_path("navigator_gui"), "resource", "compass.png",
            ),
        )
        self.compass_label.setPixmap(
            compass_icon.scaled(50, 50, QtCore.Qt.KeepAspectRatio),
        )

    def translate_to_label(self, letter):
        words = {
            "R": "Wildlife 1\nLat: 2° N\nLon: 8° E",
            "G": "Wildlife 2\nLat: 5° N\nLon: 5° E",
            "B": "Wildlife 3\nLat: 8° N\nLon: 3° E",
        }
        return words.get(letter, "Undefined")

    def translate_to_style(self, letter):
        return self.colors.get(letter, "QWidget {background-color: #FFFFFF;}")

    def update_gui(self, msg: ScanTheCode):
        print("Updating GUI with new wildlife data...")

        # Update color boxes
        self.color1_rect.setStyleSheet(self.translate_to_style(msg.color_pattern[0]))
        self.color2_rect.setStyleSheet(self.translate_to_style(msg.color_pattern[1]))
        self.color3_rect.setStyleSheet(self.translate_to_style(msg.color_pattern[2]))

        # Update labels with color names and coordinates
        self.label1.setText(self.translate_to_label(msg.color_pattern[0]))
        self.label2.setText(self.translate_to_label(msg.color_pattern[1]))
        self.label3.setText(self.translate_to_label(msg.color_pattern[2]))

        print("Finished updating GUI.")

    def shutdown_plugin(self):
        # Unregister subscriber to avoid any errors on shutdown
        self.stc_sub.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
