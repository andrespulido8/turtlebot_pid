# Turtlebot PID Controller 
Package that implements a PID controller in python and a P controller in C++
## Install
Install i[ROS Noetic](wiki.ros.org/noetic/Installation/Ubuntu) and [Gazebo](https://classic.gazebosim.org/tutorials?tut=install_ubuntu).
Then go over this [link](https://automaticaddison.com/how-to-launch-the-turtlebot3-simulation-with-ros/#gazebo) to install the turtlebot packages (or google how to do this).
 
## Usage
For the C++ P controller
```
roslaunch andres_turtlebot_pid turtlebotcpp.launch
```   
For the python PID controller
```
roslaunch andres_turtlebot_pid turtlebotpy.launch 
```

## Hardware launch
The following steps need to be performed:
1. Place the turtlebot in the line of sigh of the mocap (motion capture system). Select the dots corresponding to the robot and give it a name
2. Change the name of the robot in the `hardware.launch` file to the one given in the previous step
3. Connect the computer where the code will run ("Cassowary" in REEEF) to a monitor and keyboard. Clone the repository to the computer and compile
4. Unplug and connect to the turtlebot
5. Remotely connect to the turtlebot (`ssh cassowary@192.168.1.24` for REEF) and source ROS
6. To launch the P controller on hardware type
```
roslaunch andres_turtlebot_pid hardware.launch 
```


### TODO
- Implement dynamic reconfigure on PID control. Resources:[link1](http://wiki.ros.org/dynamic_reconfigure), [link2](http://wiki.ros.org/hokuyo_node/Tutorials/UsingDynparamToChangeHokuyoLaserParameters#PythonAPI), [link3](http://wiki.ros.org/rqt_reconfigure)
