# Use a base ROS 2 Foxy image
FROM ros:foxy

# Install additional packages if needed
RUN apt-get update && apt-get install -y \
    ros-foxy-gazebo-ros-pkgs \
    ros-foxy-rviz2 \ 
    ros-foxy-navigation2 \
    ros-foxy-nav2-bringup \
    ros-foxy-robot-localization \
    ros-foxy-xacro


# Create a workspace for your ROS 2 package
WORKDIR /dff_ws
COPY . /dff_ws/src/dff_bot

RUN . /opt/ros/foxy/setup.sh \
    && colcon build 

RUN . install/setup.sh

# # Source the ROS 2 workspace
# RUN echo "source /dff_ws/install/setup.bash" >> /root/.bashrc
