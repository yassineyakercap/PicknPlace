# Pick-n-Place Robotic Simulation
This repository contains a Pick-n-Place simulation implemented using ROS2 Humble and Gazebo Classic. The project showcases the interaction of a conveyor belt, a TurtleBot3 Autonomous Mobile Robot (AMR), and two UR5 robotic arms in a simulated environment.

The demonstration shows a UR5 robotic arm equipped with a vacuum gripper picking up a block from a conveyor belt and placing it onto the TurtleBot3 AMR. The AMR then transports this block to the location of a second UR5 arm. Please note that the pick-up action by the second robot arm has not been implemented yet.

This simulation has been tested on Gazebo Classic with ROS2 Humble. The state machine underlying the simulation is implemented with the smach library, a tool for constructing state machines in ROS.

This project provides an example of a simple multi-robot system and can serve as a resource for anyone interested in robotic simulations.

![image](https://github.com/arshadlab/PicknPlace/assets/85929438/03add9f0-8e31-4216-8aec-aac18ef1e5fe)

## Setup

OS: Ubuntu 22.04

ROS2: Humble (Should work with Foxy too)

### Clone repos

If preferred, can utilize SSH mode as an alternative method for cloning the repository.
  
   e.g git clone git@github.com:arshadlab/PicknPlace.git
   
```
mkdir -p robot_ws/src
cd robot_ws/src
git clone https://github.com/arshadlab/PicknPlace.git
git clone https://github.com/arshadlab/robot_config.git
git clone https://github.com/arshadlab/gazebo_plugins.git
git clone https://github.com/arshadlab/pymoveit2.git
git clone https://github.com/ros/executive_smach.git -b ros2
```

### Install dependencies
```
cd robot_ws
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

### Build 

```
cd robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## Run

```
ros2 launch  picknplace warehouse.launch.py
```
