# Implementing GUI Based Tools Using `rqt`

## Introduction

RQT (ROS Qt-based GUI Framework) is a powerful tool that provides a graphical
user interface for ROS (Robot Operating System) applications. It allows users
to monitor ROS topics, interact with nodes, visualize data, and more. This
document serves as a user guide for understanding and effectively using RQT.

### Benefits of RQT

- **Graphical Interface**: RQT provides a user-friendly graphical interface,
  making it easier to visualize data, monitor topics, and interact with nodes.
  This is especially helpful for users who prefer a GUI based approach over
  the command line.

- **Centralized Tool**: RQT serves as a centralized tool for various ROS tasks,
  including visualization, configuration, and debugging. This reduces the need
  to switch between multiple terminals and tools, streamlining the development
  process.

- **Extensibility**: RQT's plugin architecture allows users to create custom
  plugins to tailor the tool to their specific application needs. This
  extensibility significantly enhances RQT's capabilities and to various
  robotic systems.

- **Integration with ROS**: RQT is tightly integrated with ROS, ensuring
  seamless interaction with ROS nodes, topics, and services. This integration
  simplifies the process of debugging and analyzing data in real-time.

## Installation

Before using RQT, make sure you have ROS installed on your system. If ROS is
not yet installed, follow the official ROS installation guide for your
operating system. Once ROS is installed, you can install RQT using the package
manager:

```bash
sudo apt-get install ros-noetic-rqt
```

## Getting Started

### Launching RQT

To launch RQT, open a terminal and run the following command:

```bash
rqt
```

This command will launch the RQT Graphical User Interface, displaying a
default layout with available plugins.

## Monitoring Information with RQT

RQT provides various plugins to monitor information published on specific ROS
topics. The most common plugin for this purpose is the Plot plugin.

### Subscribing to Specific Topics

1. Open RQT using the command mentioned in the previous section.
1. Click on the "Plugins" menu in the top toolbar and select "Topics" >
   "Message Publisher."
1. A new "Message Publisher" tab will appear. Click on the "+" button to
   subscribe to a specific topic.
1. Choose the desired topic from the list and click "OK."
1. Now, you can monitor the information published on the selected topic in
   real-time.

### Displaying Data with Plot Plugin

1. Open RQT using the command mentioned earlier.
1. Click on the "Plugins" menu in the top toolbar and select "Visualization"
   > "Plot."
1. A new "Plot" tab will appear. Click on the "+" button to add a plot.
1. Choose the topic you want to visualize from the list, select the message
   field, and click "OK."
1. The plot will start displaying the data sent over the selected topic.

## Configuring RQT

RQT allows users to customize its appearance and layout to suit their preferences.

### Theme Configuration

1. Open RQT using the command mentioned earlier.
1. Click on the "View" menu in the top toolbar and select "Settings."
1. In the "Settings" window, go to the "Appearance" tab.
1. Here, you can change the color scheme, font size, and other visual settings.

### Layout Configuration

1. Open RQT using the command mentioned earlier.
1. Rearrange existing plugins by clicking on their tab and dragging them to
   a new position.
1. To create a new tab, right-click on any existing tab and select "Add New Tab."
1. Drag and drop desired plugins onto the new tab.
1. Save your customized layout by going to the "View" menu and selecting
   "Perspectives" > "Save Perspective."

## Writing RQT Plugins

RQT allows users to create their custom plugins to extend its functionality.
Writing RQT plugins is essential when the available plugins do not fully
meet your application's requirements.

### Plugin Structure

To write an RQT plugin, you need to follow a specific directory structure and
include Python or C++ code to implement the plugin's functionality. The
essential components of an RQT plugin are:

- **Plugin XML (`plugin.xml`)**: This file defines the plugin and its
  dependencies, specifying essential information like name, description,
  and which ROS packages it depends on.

- **Python or C++ Code**: This code contains the actual implementation of the
  plugin's functionality. For Python plugins, the code should extend the
  `rqt_gui_py::Plugin` class, while for C++ plugins, it should extend the
  `rqt_gui_cpp::Plugin` class.

### Implementing a Basic Plugin

In this section, we will walk through an example of implementing a basic RQT
plugin. We will create a simple plugin to display the current time published
by a ROS topic. The plugin will consist of a label that updates with the
current time whenever a new message is received.

#### Step 1: Create the Plugin Package

Create a new ROS package for your plugin using the `catkin_create_pkg` command:

```bash
catkin_create_pkg my_rqt_plugin rospy rqt_gui_py
```

The package dependencies `rospy` and `rqt_gui_py` are required to work with
ROS and create a Python-based RQT plugin.

#### Step 2: Implement the Plugin Code

Next, create a Python script for your plugin. In the `src` directory of your
package, create a file named `my_rqt_plugin.py` with the following content:

```python
#!/usr/bin/env python

import rospy
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import QLabel, QVBoxLayout, QWidget
from std_msgs.msg import String
import time

class MyRqtPlugin(Plugin):

    def __init__(self, context):
        super(MyRqtPlugin, self).__init__(context)
        self.setObjectName('MyRqtPlugin')

        # Create the main widget and layout
        self._widget = QWidget()
        layout = QVBoxLayout()
        self._widget.setLayout(layout)

        # Create a label to display the current time
        self._label = QLabel("Current Time: N/A")
        layout.addWidget(self._label)

        # Subscribe to the topic that provides the current time
        rospy.Subscriber('/current_time', String, self._update_time)

        # Add the widget to the context
        context.add_widget(self._widget)

    def _update_time(self, msg):
        # Update the label with the received time message
        self._label.setText(f"Current Time: {msg.data}")

    def shutdown_plugin(self):
        # Unregister the subscriber when the plugin is shut down
        rospy.Subscriber('/current_time', String, self._update_time)

    def save_settings(self, plugin_settings, instance_settings):
        # Save the plugin's settings when needed
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # Restore the plugin's settings when needed
        pass

```

In this code, we create a plugin class named `MyRqtPlugin`, which inherits
from `rqt_gui_py.Plugin`. The `__init__` method sets up the GUI elements, such
as a label, and subscribes to the `/current_time` topic to receive time updates.

#### Step 3: Add the Plugin XML File

Create a file named `plugin.xml` in the root of your package to define the
plugin. Add the following content to the `plugin.xml`:

```xml
<library path="src">
  <class name="MyRqtPlugin" type="my_rqt_plugin.MyRqtPlugin" base_class_type="rqt_gui_py::Plugin">
    <description>
      A custom RQT plugin to display the current time.
    </description>
    <qtgui>
      <label>My RQT Plugin</label>
      <icon type="theme">settings</icon>
      <statustip>Display the current time.</statustip>
    </qtgui>
  </class>
</library>
```

This XML file defines the plugin class, its description, and the icon to be
displayed in the RQT GUI.

#### Step 4: Build the Plugin

Build your plugin package using `catkin_make`:

```bash
catkin_make
```

#### Step 5: Launch RQT and Load the Plugin

Launch RQT using the command mentioned earlier:

```bash
rqt
```

Once RQT is open, navigate to the "Plugins" menu and select "My RQT Plugin"
from the list. The plugin will be loaded, and you should see the label
displaying the current time.

### Adding Custom Functionality

The example plugin we implemented is a simple demonstration of how to create
an RQT plugin. Depending on your application's requirements, you can add more
sophisticated features, such as custom data visualization, interactive
controls, and integration with other ROS elements.

You can also improve the plugin by adding error handling, additional options
for time formatting, and the ability to pause/resume the time updates.

## Implementing a More Advanced Plugin

In this section, we will implement a more advanced RQT plugin that visualizes
data from multiple ROS topics using a 2D plot. This plugin will allow users to
choose which topics to plot and specify the message field to display on the X
and Y axes. Let's call this plugin "ROS Data Plotter."

### Step 1: Create the Plugin Package

Create a new ROS package for your plugin using the `catkin_create_pkg` command:

```bash
catkin_create_pkg ros_data_plotter rospy rqt_gui_py matplotlib
```

The package dependencies `rospy`, `rqt_gui_py`, and `matplotlib` are required
to work with ROS, create a Python-based RQT plugin, and plot data, respectively.

### Step 2: Implement the Plugin Code

Next, create a Python script for your plugin. In the `src` directory of your
package, create a file named `ros_data_plotter.py` with the following content:

```python
#!/usr/bin/env python

import rospy
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QComboBox
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from std_msgs.msg import Float32  # Modify this import based on your data type

class ROSDataPlotter(Plugin):

    def __init__(self, context):
        super(ROSDataPlotter, self).__init__(context)
        self.setObjectName('ROSDataPlotter')

        # Create the main widget and layout
        self._widget = QWidget()
        layout = QVBoxLayout()
        self._widget.setLayout(layout)

        # Create a combo box to select topics and message fields
        self._topic_field_selector = QComboBox()
        self._topic_field_selector.currentIndexChanged.connect(self._update_plot)
        layout.addWidget(self._topic_field_selector)

        # Create the matplotlib figure and plot
        self._figure = Figure()
        self._canvas = FigureCanvas(self._figure)
        layout.addWidget(self._canvas)

        # Initialize the plot
        self._update_plot()

        # Add the widget to the context
        context.add_widget(self._widget)

    def _update_plot(self):
        # Clear the previous plot data
        self._figure.clear()
        axes = self._figure.add_subplot(111)

        # Get the selected topic and field from the combo box
        selected_topic_field = self._topic_field_selector.currentText()
        topic, field = selected_topic_field.split(' - ')

        # Subscribe to the selected topic
        rospy.Subscriber(topic, Float32, self._plot_data)  # Modify this based on your data type

    def _plot_data(self, msg):
        # Get the selected topic and field from the combo box
        selected_topic_field = self._topic_field_selector.currentText()
        topic, field = selected_topic_field.split(' - ')

        # Plot the data on the graph
        # Modify this part based on your data type and plot requirements
        x_data = rospy.Time.now().to_sec()  # Use timestamp as X-axis
        y_data = getattr(msg, field)  # Use the specified field from the message as Y-axis data
        axes = self._figure.add_subplot(111)
        axes.plot(x_data, y_data)

        # Refresh the plot
        self._canvas.draw()

    def shutdown_plugin(self):
        # Unsubscribe from the topic when the plugin is shut down
        self._topic_field_selector.clear()
        rospy.Subscriber(topic, Float32, self._plot_data)

    def save_settings(self, plugin_settings, instance_settings):
        # Save the plugin's settings when needed
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # Restore the plugin's settings when needed
        pass

```

In this code, we create a plugin class named `ROSDataPlotter`, which inherits
from `rqt_gui_py.Plugin`. The `__init__` method sets up the GUI elements,
such as a combo box to select topics and message fields, and the matplotlib
figure to plot the data. The `_update_plot` method updates the plot whenever
a new topic or message field is selected from the combo box. The `_plot_data`
method is called when new messages are received on the selected topic, and it
plots the data on the graph.

### Step 3: Add the Plugin XML File

Create a file named `plugin.xml` in the root of your package to define the
plugin. Add the following content to the `plugin.xml`:

```xml
<library path="src">
  <class name="ROSDataPlotter" type="ros_data_plotter.ros_data_plotter.ROSDataPlotter" base_class_type="rqt_gui_py::Plugin">
    <description>
      An advanced RQT plugin to plot data from multiple ROS topics.
    </description>
    <qtgui>
      <label>ROS Data Plotter</label>
      <icon type="theme">settings</icon>
      <statustip>Plot data from selected ROS topics.</statustip>
    </qtgui>
  </class>
</library>
```

This XML file defines the plugin class, its description, and the icon
to be displayed in the RQT GUI.

### Step 4: Build the Plugin

Build your plugin package using `catkin_make`:

```bash
catkin_make
```

### Step 5: Launch RQT and Load the Plugin

Launch RQT

 using the command mentioned earlier:

```bash
rqt
```

Once RQT is open, navigate to the "Plugins" menu and select "ROS Data Plotter"
from the list. The plugin will be loaded, and you should see a combo box to
select topics and fields, and a graph to display the plotted data.

### Adding Custom Functionality

In this more advanced example, we created an RQT plugin that allows users to
plot data from multiple ROS topics. The example focused on plotting Float32
messages, but you can modify the `_plot_data` method to handle different
message types or plot multiple data series on the same graph.

You can further enhance the plugin by adding features like legends, axis
labels, custom plot styles, data smoothing, and real-time updates.
Additionally, you can provide options to save the plotted data as CSV
or images for further analysis.

With custom plugins, you have the freedom to tailor RQT to suit your
specific application's visualization needs, making it a versatile tool
for ROS developers.

## External Resources

| Resources |
| ----------- |
| [ROS wiki page for rqt](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&cad=rja&uact=8&ved=2ahUKEwin36-OnbmAAxU1sDEKHf_UCSoQFnoECBAQAQ&url=http%3A%2F%2Fwiki.ros.org%2Frqt&usg=AOvVaw2p3mQTkmoXih9cZS1NTOW6&opi=89978449) |
| [GeorgiaTech RoboJacket's rqt tutorial](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&cad=rja&uact=8&ved=2ahUKEwin36-OnbmAAxU1sDEKHf_UCSoQwqsBegQIDxAG&url=https%3A%2F%2Fwww.youtube.com%2Fwatch%3Fv%3Do90IaCRje2I&usg=AOvVaw04mXzk9sBXpTMhkhgGqXIb&opi=89978449)   |
| [Creating Custom rqt Plugins](https://github.com/ChoKasem/rqt_tut) |

## Conclusion

Congratulations! You have now learned the basics of using RQT, including
monitoring information sent over specific topics, configuring RQT to your
preferences, writing your custom plugins, and understanding the importance
of using RQT in ROS development. RQT provides a user-friendly and powerful
graphical interface for ROS applications, making it an essential tool for
developers and researchers working with ROS-based robotic systems. By
leveraging RQT's capabilities, you can streamline your development process
and gain valuable insights into your robot's behavior.
