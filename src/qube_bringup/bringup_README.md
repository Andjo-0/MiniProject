# QUBE bringup 
The purpose of this package is to launch the Rviz visualisation of the qube along with the qube controller from the provided
qube_driver using a single launch file. The package contains a URDF file called controlled_qube.urdf.xacro which initialises the macros of the quanzer qube visualisation from the qube_description package as
well as initialises the qube driver controller for the movement of the qube in the visualisation as well as the physical qube. 

The launch file accepts 3 launch parameters/commands which are baud raute, device and simulation.

The simulation command is a bool thats either true or false and determines if the qube_driver launches as a simulation or not. The default is set to true.

Baud rate is the baud rate for the communication between the quanzer qube and the PC. The default is set to 115200.

The device is the string for the USB device/port that the quanzer qube is connected to on the computer. The correct address needs to be checked as it will vary.

The terminal command to launch the launch file is as follows:

```
ros2 launch qube_bringup bringup.launch.py 
\ baud_rate:=115200 \ device:=/dev/ttyACM0 \ simulation:=true 
```
