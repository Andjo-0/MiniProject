# The Nodes
## Qube Controller
Is the Node responsible for implementation of the PID-controller used to controll the velocity of the motor. The Controller tries to make it's output to be the reference value decided by the end user, taking into account the PID values and the current velocity of the motor. The response of the output signal is being mainly changed by change of PID values. In the implementation the maximal and minimal value of the Integral part has been limited to not be greater then 50, and not smaller then -50 to make sure that the sum of error will not grow uncontrolled making the system unstable. The same has been done with commanded_velocity which is the velocity of the motor to add an aditional layer of qube protection. The controller should also allow the dynamical change of Kp, Ki and Kd parameters.

## Qube description
This node holds the urdf/xacro files containing the model of the qube used in the presentation.


# How to use
## Launch visualisation
To use the qube bring up as well as change the 3 different parameters first build the
packages in this workspaces and run the following terminal command. This should open up the qube simulation in RvIZ. The 3 different parameters that can be set are baud_rate, device and SImulation.
#### NB: If no quanzer qube is connected to the computer then the simulation has to be set to true


```
ros2 launch qube_bringup bringup.launch.py 
\ baud_rate:=115200 \ device:=/dev/ttyACM0 \ simulation:=true 
```

## PID controller
The PID controller node is found in the qube_controller package and in order to run it you have to use the following command in a different terminal (The workspaces have to be built and sourced beforehand)


```
ros2 run qube_controller pid_controller \
 --ros-args  --param reference:=30.0  --param p:=0.9  --param i:=0.5  --param d:=0.0 
```

