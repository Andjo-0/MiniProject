
# Qube Description

This package contains a macro urdf file called qube.macro.xacro which contains a description of a very simple visualisation of 
a quanzer qube. The macro file is initialised in another xacro file called qube.urdf.xacro which initialises a qube as well as creates a link between the qube
and the base of the visualisations world.

The launch file called view_qube.launch.py is a modified launch file based on this urdf launch file https://github.com/ros/urdf_launch/blob/main/launch/display.launch.py.

To view the model simply use the following command

```
ros2 launch qube_description view_qube.launch.py
```





