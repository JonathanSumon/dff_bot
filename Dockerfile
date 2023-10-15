# Use a base ROS 2 Foxy image
FROM ros:foxy

# Install additional packages if needed
RUN apt-get update && apt-get install -y \
    ros-foxy-gazebo-ros-pkgs \
    ros-foxy-rviz2


RUN . /opt/ros/foxy/setup.bash

# Create a workspace for your ROS 2 package
WORKDIR /dff_ws
COPY . /dff_ws/src/dff_bot
RUN colcon build

# Source ROS 2 Foxy workspace


# Source the ROS 2 workspace
RUN echo "source /dff_ws/install/setup.bash" >> /root/.bashrc
