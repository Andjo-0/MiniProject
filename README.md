# How to use
## Launch visualisation
To use the qube bring up as well as change the 3 different parameters first build the
packages in this workspaces and run the following terminal command. This should open up the qube simulation in RvIZ. The 3 different parameters that can be set are baud_rate, device and SImulation.
#### NB: If no quanzer qube is connected to the computer then the simulation has to be set to true


<pre> ros2 launch qube_bringup bringup.launch.py 
\ baud_rate:=115200 \ device:=/dev/ttyACM0 \ simulation:=true </pre>


## PID controller
The PID controller node is found in the qube_controller package and in order to run it you have to use the following command

<pre> ros2 run qube_controller pid_controller --ros-args  --param reference:=30.0  --param p:=0.9  --param i:=0.5  --param d:=0.0 </pre>