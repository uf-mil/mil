# Dock Shapes
The dock shapes package contains two executables for seeing the red,green,and blue triangles,crosses,and cirlces on the dock in the find and deliver challenge

## To build
```catkin_make navigator_shoot_vision vision smart_shape_finder_server```

## To Run 
```roslaunch navigator_shoot_vision dock_shapes.launch```

## Services Provided

Service | Type | Behavior 
--- | --- | --- 
/dock_shapes/GetShape | navigator_msgs/GetDockShape | Returns the center and points of the specified shape if found
/dock_shapes/run | std_srvs/SetBool | Starts/stop the vision services

## Config 
Most of the vision constants can be adjusted through the dock_shapes.yaml file
