import os

import rospkg
import rospy
from navigator_msgs.msg import ScanTheCode
from python_qt_binding import QtWidgets, loadUi
from python_qt_binding.QtWidgets import QWidget
from qt_gui.plugin import Plugin


class StcDisplay(Plugin):

    UPDATE_MILLISECONDS = 1000
    STC_TOPIC = "/stc_sequence"
    colors = {
        "red": "QWidget {background-color:#FF432E;}",
        "green": "QWidget {background-color:#B1EB00;}",
        "blue": "QWidget {background-color:#4AA8DB;}",
    }

    def __init__(self, context):
        super().__init__(context)
        # Give QObjects reasonable names
        self.setObjectName("STC Display")

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser

        parser = ArgumentParser()
        # Add argument(s) to the parser.
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
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(
            rospkg.RosPack().get_path("navigator_gui"), "resource", "stc.ui"
        )
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName("MyPluginUi")
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (" (%d)" % context.serial_number())
            )
        # Add widget to the user interface
        context.add_widget(self._widget)
        self.connect_ui()
        self.stc_sub = rospy.Subscriber(self.STC_TOPIC, ScanTheCode, self.update_gui)

    def connect_ui(self) -> None:
        self.color1_rect = self._widget.findChild(QtWidgets.QColumnView, "color1_rect")
        self.color2_rect = self._widget.findChild(QtWidgets.QColumnView, "color2_rect")
        self.color3_rect = self._widget.findChild(QtWidgets.QColumnView, "color3_rect")
        self.label1 = self._widget.findChild(QtWidgets.QLabel, "label")
        self.label2 = self._widget.findChild(QtWidgets.QLabel, "label_2")
        self.label3 = self._widget.findChild(QtWidgets.QLabel, "label_3")

    def translate_to_label(self, letter: str) -> str:
        if letter == "R":
            return "Red"
        elif letter == "G":
            return "Green"
        elif letter == "B":
            return "Blue"
        return "Undefined"

    def translate_to_style(self, letter: str) -> str:
        if letter == "R":
            return "QWidget {background-color:#FF432E;}"
        elif letter == "G":
            return "QWidget {background-color:#B1EB00;}"
        elif letter == "B":
            return "QWidget {background-color:#4AA8DB;}"
        return "Undefined"

    def update_gui(self, msg: ScanTheCode):
        print("updating gui...")
        self.color1_rect.setStyleSheet(self.translate_to_style(msg.color_pattern[0]))
        self.color2_rect.setStyleSheet(self.translate_to_style(msg.color_pattern[1]))
        self.color3_rect.setStyleSheet(self.translate_to_style(msg.color_pattern[2]))
        self.label1.setText(self.translate_to_label(msg.color_pattern[0]))
        self.label2.setText(self.translate_to_label(msg.color_pattern[1]))
        self.label3.setText(self.translate_to_label(msg.color_pattern[2]))
        print("finished updating!")

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    # def trigger_configuration(self):
    # Comment in to signal that the plugin has a way to configure
    # This will enable a setting button (gear icon) in each dock widget title bar
    # Usually used to open a modal configuration dialog
