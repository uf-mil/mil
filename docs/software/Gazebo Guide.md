# What is Gazebo?

Gazebo is an open-source 3D dynamic simulator, that allows us to design, model, and test robots and their behavior in a virtual world. Similar to game engines like Unity, Gazebo takes in scripts and models and performs physics simulations on them. While it is similar, Gazebo offers physics simulations at a higher level of fidelity, a suite of sensors, and interfaces for both users and programs.

There are many versions of Gazebo, but this guide was written for **Gazebo Classic 11** as this is the version of Gazebo currently being used at MIL (Machine Intelligence Lab).

## How To Run Gazebo?

There are many ways run Gazebo.

* Click the "Show Applications" button on Ubuntu (the apps button located in the bottom left corner). Then search for the Gazebo Icon and press that icon to open Gazebo.

* You can press Alt + F2 (or Alt+Fn+F2) to bring up the "Run a Command" window. Then type "gazebo" and press Enter to open Gazebo.

* You can also open a terminal and type "gazebo" and it will open Gazebo.

To launch Gazebo will all the necessary files for simulating Subjugator, follow these steps:

1. Open a terminal window and execute the following command. This command uses ROS to start all the relevant ROS nodes and to load the world file for subjugator. This also starts a Gazebo Sever.
```bash
roslaunch subjugator_launch gazebo.launch --screen
```
:::{note}
-\-screen just forces all ROS node output to the screen. It is used for debugging.
:::

2. Then in another terminal window run this command to start the Gazebo graphical client, which connects to the Gazebo Sever.
```bash
gazebogui
```

3. Then in another terminal window run this command and then press Shift C to unkill the sub to allow movement.
 ```bash
amonitor kill
```

 4. Execute the following command to start a specific mission, replacing "StartGate2022" with the name of the desired mission:
 ```bash
mission run StartGate2022
```

# How to use Gazebo

## User Interface

When you launch Gazebo you will be greeted by its user interface.

![Gazebo Interface](/software/Gazebo/Interface.png)

The Gazebo interface consists of three main sections: The **Left Panel**, the **Scene**, and the **Right Panel**. By default the Right Panel is hidden. This is because we do not have anything selected. To show the right panel you can always Click and drag the bar on the right to open it.

![Gazebo Labeled Interface](/software/Gazebo/Labeled_Interface.png)

### Left Panel

The Left Panel has three tabs, each with different features. You can see these tabs at the top of the Left Panel. You can just click on them to switch between them. The tabs are:

#### World Tab

The World Tab displays the models that are currently in the scene. Within this tab, you can view and modify various model parameters, like their pose (their position and rotation). Additionally, you can expand the GUI option to adjust the camera view angle by modifying the camera pose.

#### Insert Tab

The Insert Tab allows you to add new models (objects) to the Gazebo simulation. Here, you will find a list of file paths where your models are saved. To view the model list, click on the arrow located on the left side of each path to expand the folder. Select the desired model and click again in the scene to place it.

#### Layers Tab

The Layers tab organizes and displays different visualization groups within the simulation. Layers can contain one or more models, and enabling or disabling a layer will show or hide all the models within it. While not mandatory, layers can be helpful for organizing your simulation. Note that this tab may be empty if no layers are defined.

### Scene

The Scene is the main window where objects are animated, and you interact with the environment. Two toolbars are available:

#### The Upper Toolbar

The Upper Toolbar consists of various buttons that allow you to select, move, rotate, and scale objects. It also provides options to create simple shapes, as well as copy and paste objects.

![Gazebo Upper Toolbar](/software/Gazebo/Upper_Toolbar.png)

#### The Bottom Toolbar

The Bottom Toolbar displays information about the simulation time and its relationship to real time. It helps you track the progress of your simulation.

![Gazebo Bottom Toolbar](/software/Gazebo/Bottom_Toolbar.png)

### Right Panel

The Right Panel is used to interact with the mobile parts (joints) of a selected model. It provides controls and settings specific to manipulating the joints of a model.

### Menus (File, Edit, Camera, View, Window, Help)

Most Linux apps have menus. If you don't see it move your cursor to the top of the application window and the menus should appear. Below describes the features of each menu.

![Gazebo Menus](/software/Gazebo/Menus.png)

### Mouse

It is recommended that you use a mouse when using Gazebo. Below is a diagram showing all the mouse controls.

![Gazebo Mouse Controls](/software/Gazebo/Mouse.png)



## Model Creation
You can use the **Model Editor** to create simple models all within Gazebo, but for more complex models you will want to create/write your own SDF files.

To enter the **Model Editor**, click on Edit in the menu bar and select Model Editor.

The Model Editor Interface looks similar to the regular Gazebo UI with some slight changes. The left panel and the top toolbar have been changed to contain only buttons and features for editing and creating parts of a model. The bottom toolbar is now hidden as the simulation is paused.

![Gazebo Model Editor](/software/Gazebo/Model_Editor.png)


### Insert Tab

The Insert Tab allows you to add new parts, including links and models, to your model. You have two options for inserting shapes (links):

* Simple Shapes: Click on the desired shape in the left panel and then click again in the scene to place it.

* Custom Shapes: You can add COLLADA (.dae), 3D Systems (.stl), Wavefront (.obj), and W3C SVG (.svg) files as custom shapes. Create these shapes using 3D modeling software like Blender.

You can also insert other models into your model as nested models. These models can be obtained from the Gazebo Model Database (http://gazebosim.org/models/), which should be listed as one of your file paths under Model Databases. For example, if you need a depth sensor, you can add a depth sensor model from the database to your model.

### Model Tab

The Model Tab displays the settings for the model you are creating. Here, you can change the model's name and modify its basic parameters. Additionally, you can add plugins to give your model functionality here as well.

### Placing Shapes

Once you insert a shape, you can use the toolbar to move, rotate, and scale it. For finer control, you can double-click the shape or right-click and select "Open Link Inspector" to access the link inspector. In the link inspector, you can modify the shape's position, rotation, and scale to achieve the desired configuration. Make sure to adjust the scale in both the Visual and Collision tabs.

### Adding Joints

To constrain the motion between shapes, you can add joints. Follow these steps:

* Click on the joint icon in the toolbar (a line connecting two points).

* In the Joint Creation Window, select the parent and child links (shapes) of the joint.

* Select the type of joint you need in the Joint Types section near the top of the window.

* Select the joint axis. Some joints do not have an axis.

* Align the link (shape). Use the align links section to align the parent and the child with each other.

### Adding a Plugin

To control your model, you need to create a plugin. You can do this in the Model Tab by specifying the necessary details for the plugin.

You can find more information on how to create your own custom plugins here https://classic.gazebosim.org/tutorials?tut=ros_gzplugins

### Model Creation Workflow Example

To illustrate the model creation process, let's consider creating a car model using Blender:

* Create .dae files for the wheels, chassis, and other parts in Blender.

* Insert these shapes into the Model Editor.

* Use the toolbar and link inspector to position each shape precisely.

* Add joints between the shapes to enable motion constraints.

* Finally, create a plugin to control the model's behavior.

## World File

A World in Gazebo is used to describe the collection of robots and objects, and global parameters like the sky, ambient light, and physics properties. Basically, a World is the entire virtual environment that you have been working in. The World stores important information like where all the models are, their properties, and important global properties.

You can save the World file by selecting File and Save World As.

:::{note}
When using roslaunch to start Gazebo, it is crucial to update the World file if you make any changes to the simulation environment. At MIL, there is a dedicated "worlds" folder where Gazebo World files are saved. When you update a World file, ensure that you replace the old file in this folder. Failing to do so will result in the continued use of the old World file when launching Gazebo using roslaunch.
:::

# More Info

If you ever need more information on how any aspect of Gazebo works or how to use ROS with Gazebo you can check out the official Gazebo Documentation here: https://classic.gazebosim.org/tutorials.
Note that most of the pictures and information used in this guide are sourced from there.
