FROM osrf/ros:humble-desktop-jammy
# Update


RUN apt-get update
RUN apt-get install -y software-properties-common apt-utils -y



# Set working directory
WORKDIR /home/ros/src

RUN git clone https://github.com/arshadlab/PicknPlace.git
RUN git clone https://github.com/arshadlab/robot_config.git
RUN git clone https://github.com/arshadlab/pymoveit2.git
RUN git clone https://github.com/ros/executive_smach.git -b ros2

WORKDIR /home/ros
# Install ROS dependencies
RUN rosdep update \
&& rosdep install --from-paths src --ignore-src -r -y

RUN bash -c "./src/PicknPlace/docker/build_ros.sh"
