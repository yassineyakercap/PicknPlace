# Gazebo Pick & Place Demo
This repository contains a Pick-n-Place simulation implemented using ROS2 Humble and Gazebo Classic. The project showcases the interaction of a conveyor belt, a TurtleBot3 Autonomous Mobile Robot (AMR), and two UR5 robotic arms in a simulated environment.  The aim is to harnesses the capabilities of both the Nav2 and MoveIt2 stacks, presenting a comprehensive demonstration of multi-robot coordination in a virtual environment.

This implementation has room for further enhancements, but it can serve as an excellent starting point for anyone looking to understand the integration of Nav2 and Moveit2 stacks in multi robot scenario.

https://github.com/arshadlab/PicknPlace/assets/85929438/1f74f3ae-6e81-4e10-b44e-0b9b33a77aa3

## Setup

OS: Ubuntu 22.04

ROS2: Tested on Humble (but Should work with Foxy too)

### Running via Docker Environment
**Docker Installation**: Ensure that Docker is properly installed and functional.

**File Preparation**:
* Clone this repository. Navigate to the docker directory and copy the Dockerfile and docker-run.sh to your local machine.
* The Dockerfile and docker-run.sh can also be copied directly to local machine without the need to clone the entire repository. The docker method just need these two files.
  
**Building the Docker Image**:
Navigate to the directory containing the **Dockerfile** and execute the following command to build a Docker image named **picknplace**:
```
docker build -t picknplace .
```
Successful creation of the image can be confirmed by checking the list of available Docker images with:
```
docker images
```
**Enable GUI Access for Docker**:
To permit Docker containers to display a GUI on the host's X server, run the following command (Note: This step is required after every system reboot):
```
xhost +local:docker
```
This command grants Docker containers access to the host's graphical interface, which is essential to view the Gazebo GUI.


**Prepare the Docker Run Script**:
If the docker-run.sh script isn't executable, grant it the necessary permissions with:
```
chmod +x ./docker-run.sh
```

**Launching the Docker Container**:
Run the container using the script with:
```
./docker-run.sh picknplace /home/ros/src/PicknPlace/docker/run_sim.sh
```


### Running via local cloning

Clone picknplace and depending repos
   
```
mkdir -p robot_ws/src
cd robot_ws/src
git clone https://github.com/arshadlab/PicknPlace.git
git clone https://github.com/arshadlab/robot_config.git
git clone https://github.com/arshadlab/pymoveit2.git
git clone https://github.com/ros/executive_smach.git -b ros2
```

**Install dependencies**
```
cd robot_ws
source /opt/ros/humble/setup.bash
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

**Build**

```
cd robot_ws
source ~/robot_ws/install/local_setup.bash
colcon build --symlink-install
```

**Run**

```
ros2 launch  picknplace warehouse.launch.py
```

## Overview
The setup consists of:

* **Two robotic arms**: Based on the UR5 model.
* **One Autonomous Mobile Robot (AMR)**: A customized version of the TurtleBot3.

The robotic arms are controlled using the MoveIt2 stack, whereas the AMR is navigated using the Nav2 stack. Each robot operates within its own namespace, showcasing the seamless integration of multiple robots, each with its designated control stack, in a unified Gazebo environment.

The primary goal of this demo is to illustrate the combined and coordinated use of Nav2 and MoveIt2 stacks in a Gazebo simulation.

The demonstration workflow is as follows:

* One of the robotic arms (ARM1) picks up an item from a moving conveyor belt.
* The item is then placed onto the AMR, which is based on the TurtleBot3 Waffle design.
* Using Nav2, the AMR autonomously plans and traverses a path to the second robotic arm, referred to as ARM2.

**Note**: This demo prioritizes the representation of combined stack usage over intricate details. Some assumptions have been made for simplicity. For instance, the item's location on the conveyor belt is sourced directly from Gazebo without integrating perception systems. Additionally, while ARM1 is present in the simulation, it remains static and does not perform any actions at this time.

## Other Details
**State Machine Implementation**: The demo employ the Smach library for designing the state machine that serves as the arm1 controller in Python. SMACH is a valuable tool for creating, managing, and examining hierarchical state machines for robotic operations. 

**Moveit wrapper**: The moveit commands are send using a modified version of pymoveit2, courtesy of Andrej Orsula. This version introduces several enhancements and rectifies existing bugs. However, with the recent availability of Python bindings in the latest Moveit2 stack, it's advisable to use that instead.

**Object location**: This demonstration bypasses perception mechanisms. Instead, object locations are sourced from the get_entity_state service, courtesy of the Gazebo plugin. For prospective integrations, the objects on the conveyor are marked with Aruco markers, readying them for vision-based use cases.

**Serialize Gazebo model spawning**: To maintain the integrity of ROS2 namespaces, Gazebo models (like AMR and arms) undergo sequential deployment. In ROS2 control, the controller manager alters the global gzserver namespace based on the current robot's namespace to facilitate subsequent controller initialization. This can disrupt the namespace configuration for other models launching ROS2 nodes via their embedded plugins. Consequently, I have orchestrated the model deployments to guarantee a clean global namespace before deploying any subsequent model. This namespace reset is achieved through a custom Gazebo plugin found in the robot_config repository.

**Cyclone DDS usage**: It's recommended to execute the demo using Cyclone DDS over FastDDS. I observed some instability with FastDDS, potentially due to the multitude of nodes and associated interfaces instantiated in the gzserver. This might overload a singular DDS participant (like a process). In my tests, Cyclone DDS emerged as the more reliable choice, particularly when handling a vast number of nodes established by a single entity. This might also be attributable to FastDDS's default configurations, optimized for speed.

To utilize Cyclone DDS, enable it through the following commands. The rosdep command highlighted in the setup section will ensure Cyclone DDS is installed.

```
sudo apt-get install ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## Controlling UR5 ARM

Using its namespace, a robotic arm can be commanded through the command line.

To direct ARM2 to the position [0.39,-0.2799,0.1], use the following command:

```
cd robot_ws
source ./install/setup.bash
ros2 run pymoveit2 ex_pose_goal.py --ros-args  -r __ns:=/arm2 -p cartesian:=True -p position:=[0.39,-0.2799,0.1]
```

## Sending Nav2 Pose to ARM
Use the following command to set a new goal for the AMR:

```
ros2 action send_goal  /amr1/navigate_to_pose nav2_msgs/action/NavigateToPose "pose: {header: {frame_id: map}, pose: {position: {x: -3.2, y: -0.50, z: 0.0}, orientation:{x: 0.0, y: 0.0, z: 0, w: 1.0000000}}}"
```

# Performance
The RTF for this simulation on the TGL i7 NUC is approximately 0.93, which is close to real-time speed. A more advanced machine would easily achieve an RTF of 1.0.  In Gazebo simulation, the Real-Time Factor (RTF) indicates the speed of the simulation relative to real-world time. An RTF of 1.0 means the simulation runs at real-time speed, while values less than 1.0 indicate a slower simulation, and values greater than 1.0 indicate a faster simulation. The goal in many robotic simulations is to achieve an RTF close to 1.0, ensuring realistic behavior, but the actual RTF can vary based on computational resources and simulation complexity.

The simulation is set with a step size of 1 ms, implying that each millisecond of simulation time corresponds to a real-time millisecond. As observed from the snapshot of the trace below, a single Gazebo update takes approximately 1 ms, leading to an accumulated RTF of about 0.93.


![image](https://github.com/arshadlab/PicknPlace/assets/85929438/356dc839-7450-4370-af36-c2e95758878f)

Time chart generated using [repo](https://github.com/arshadlab/time_charting_with_perf)

# Conclusion

This simulation has been tested on Gazebo Classic with ROS2 Humble. This project provides an example of a simple multi-robot system and can serve as a resource for anyone interested in robotic simulations.

Feel free to customize this further to better fit the specifics or nuances of your project!
