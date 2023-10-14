# Use an official ROS 2 Foxy Docker image as the base image
FROM ros:foxy

# Set up the workspace directory
WORKDIR /root/dff_ws

# Install necessary packages and dependencies
RUN apt-get update && apt-get install -y \
    ros-foxy-gazebo-ros-pkgs \
    ros-foxy-rviz2

# Copy your ROS package into the container
COPY . /home/jonathan/dff_ws/src/dff_bot

# Build your ROS package
RUN . /opt/ros/foxy/setup.sh && colcon build --symlink-install

# Run a startup script that launches your Gazebo simulation and RViz
CMD ["ros2", "launch", "dff_bot", "nav2_launch.launch.py"]
