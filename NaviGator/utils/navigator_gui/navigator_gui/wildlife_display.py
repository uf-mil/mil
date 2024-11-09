import os

import matplotlib.pyplot as plt
import rospkg
import rospy
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from navigator_msgs.msg import Wildlife
from python_qt_binding import QtWidgets, loadUi
from python_qt_binding.QtWidgets import QVBoxLayout, QWidget
from qt_gui.plugin import Plugin


class WildlifeDisplay(Plugin):
    UPDATE_MILLISECONDS = 1000
    WILDLIFE_TOPIC = "/wildlife_report"
    colors = {
        "red": "QWidget {background-color:#FF432E;}",
        "green": "QWidget {background-color:#B1EB00;}",
        "blue": "QWidget {background-color:#4AA8DB;}",
    }

    def __init__(self, context):
        super().__init__(context)
        # Give QObjects reasonable names
        self.setObjectName("Wildlife Display")

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
            rospkg.RosPack().get_path("navigator_gui"),
            "resource",
            "wildlife.ui",
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
                self._widget.windowTitle() + (" (%d)" % context.serial_number()),
            )
        # Add widget to the user interface
        context.add_widget(self._widget)
        self.connect_ui()
        self.wildlife_pub = rospy.Subscriber(
            self.WILDLIFE_TOPIC,
            Wildlife,
            self.update_gui,
        )

    def connect_ui(self) -> None:
        # Existing UI connections
        self.color1_rect = self._widget.findChild(QtWidgets.QColumnView, "color1_rect")
        self.color2_rect = self._widget.findChild(QtWidgets.QColumnView, "color2_rect")
        self.color3_rect = self._widget.findChild(QtWidgets.QColumnView, "color3_rect")
        self.label1 = self._widget.findChild(QtWidgets.QLabel, "label")
        self.label2 = self._widget.findChild(QtWidgets.QLabel, "label_2")
        self.label3 = self._widget.findChild(QtWidgets.QLabel, "label_3")

        # Add a matplotlib canvas to display the graph
        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)
        layout = QVBoxLayout()
        layout.addWidget(self.canvas)

        self._widget.findChild(QtWidgets.QWidget, "graph_area").setLayout(layout)

        # Test display TODO: Remove
        # msg = Wildlife()
        # msg.has_red_python = True
        # msg.red_python = Point(2.0, 3.0, 0.0)

        # msg.has_green_iguana = False
        # msg.green_iguana = Point()

        # msg.has_blue_manatee = True
        # msg.blue_manatee = Point(-4.0, 5.0, 0.0)

        # self.plot_points(msg)

    def plot_points(self, msg: Wildlife):
        """
        Plot points on the graph with their names and corresponding colors based on the message.

        :param msg: WildlifeDisplay message containing points and availability flags
        """
        self.figure.clear()
        ax = self.figure.add_subplot(111)

        # Define the points and colors for the animals
        animals = [
            (msg.has_red_python, msg.red_python, "Red Python", "red"),
            (msg.has_green_iguana, msg.green_iguana, "Green Iguana", "green"),
            (msg.has_blue_manatee, msg.blue_manatee, "Blue Manatee", "blue"),
        ]

        # Plot only the points that are present
        for has_animal, point, name, color in animals:
            if has_animal:  # Plot only if the animal is present
                x, y = point.x, point.y
                ax.plot(x, y, "o", label=name, color=color)  # 'o' for circular marker
                ax.text(x, y, name, fontsize=10, ha="right")

        # Set graph limits and titles
        ax.set_xlabel("X Axis")
        ax.set_ylabel("Y Axis")
        ax.set_title("Wildlife Display Points")
        ax.legend()

        # Refresh the canvas
        self.canvas.draw()

    def update_gui(self, msg: Wildlife):
        """
        Update the GUI with the received message.

        :param msg: WildlifeDisplay message containing the data
        """
        print("updating gui...")

        # Plot the points based on the message
        self.plot_points(msg)

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
