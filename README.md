<h1 align="center">dff_bot ROS 2 Package</h1>

<p align="center">
  <em>Your Comprehensive Guide to dff_bot Simulation and Navigation in ROS 2</em>
</p>

---

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
  - [Navigation and Mapping](#navigation-and-mapping)
  - [Custom Configuration](#custom-configuration)
  - [Additional Scripts](#additional-scripts)
- [Models and Meshes](#models-and-meshes)
- [Maps](#maps)
- [World Files](#world-files)
- [Testing](#testing)
- [Code Quality Check](#code-quality-check)
- [Room for improvement and Future scope](#room-for-improvement-and-future-scope)
- [Contact](#contact)

---

## Overview

This ROS 2 package provides support for simulating and controlling the dff_bot robot in a Gazebo environment for this particular Case Study. The package includes various configuration files, launch files, and resources to help you get started with dff_bot simulation and navigation.

## Prerequisites

- Ubuntu 20.04
- ROS 2 Foxy Fitzroy with Gazebo installed
- Docker


Make sure the display xhost configurations are correct so that the gazebo and Rviz screen can be seen properly

`sudo xhost +local:docker`

## Installation

1. Clone this repository into your ROS 2 workspace.
2. Build the package using `colcon build`.

or 

Preferred.

1. cd your_workspace/src/dff_bot
2. sudo docker-compose build

Alternatively you can build from the docker file
sudo docker build -t dff_bot:foxy .

Replace dff_bot:foxy with your desired container name and tag.


## Usage

If built using DockerFile and want to run as an interactive terminal, you can use the following commands. Make sure you have allowed xhost with local. 

`sudo docker run -it --rm --network host --device=/dev/dri:/dev/dri -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix dff_bot:foxy`

If built using docker-compose

`sudo docker-compose up`


Please note: First time when you launch the world, it might take some time as it heavy models. Expect a wait time of atleast 5-7 mins.

This should launch the dff_bot in a custom world. Please note that I have taken the bot and world straight out of tutorials available online.
I have put all the necessary files such urdf, world, map and nav2 params as a part of the package. 


After the gazebo and rviz launches succesfully, Initiate the inital pose of the robot. (Could be automated.)


For the bot to move in a rectangular shape, I have written 3 custom scripts. 

1. scripts/monitor.py
2. scripts/rectangular_movement_test.py - Please note that this is a custom test script which I've written and did not use test tools.
3. scripts/waypoint_publisher.py


Now the codes need to be launched in the same order as given above. We will be manually launching the codes. (Could be automated)

To launch the codes and make the robot move in the rectangular shape, follow the below given instructions.

1. Find the container id by the following command - sudo docker ps.
2. sudo docker exec -it (your-container-id-here) bash

Open 3 interactive terminals. One for each code. Once you are in the terminal. Follow these instructions

Terminal 1:
- python3 src/dff_bot/scripts/monitor.py

Terminal 2:
- python3 src/dff_bot/tests/rectangular_movement_test.py

Terminal 3:
- python3 src/dff_bot/scripts/waypoint_publisher.py


The robot should start moving in a rectangular shape. The test code will monitor if the waypoints are getting hit. 


### Navigation and Mapping

I have provided launch files for running navigation and mapping with dff_bot:

- `nav2_launch.launch.py`: The one stop Launch file to launch the entire world



### Custom Configuration

- Configuration files can be found in the `config` directory.
- Parameters can be adjusted in the `params` directory.
- RViz configuration files are located in the `rviz` directory.

### Additional Scripts

I have included some helpful Python scripts:

- `monitor.py`: A custom script which listens to odometry of the robot and goal poses given to the robot. Based on odometry and reaching the goal, it publishes in a custom topic that the robot has reached it's goal
- `waypoint_publisher.py`: Publishes waypoints for robot navigation in the form of goal poses to form a rectangle shape. Please feel free to change the coordinates.

## Models and Meshes

The `models` and `meshes` directories contain the URDF and STL files for dff_bot's visual and collision representations.

## Maps

Maps used for navigation can be found in the `maps` directory.

## World Files

World files for the Gazebo simulator are located in the `worlds` directory.

## Testing

I have included a sample test script in the `tests` directory: `rectangular_movement_test.py`. This is not a standard test build using a test framework. This is a custom script which I wrote to se

## Docker Support

For users who prefer Docker, I have provided a `Dockerfile` and `docker-compose.yml` to help set up your environment.


## Code Quality Check

I have added a github workflow which simply checks the linting of monitor.py.
If the code test report is above 7.5, the build continues.
If it is less that 7.5, it fails


## Room for improvement and Future scope

- Proper test case using colcon_tests can be written. The above test script is a realtime sub-optimal code.
- The test script logic can be improved much further. To check whether the robot is moving in a rectangular shape, we can check the following:
    - Number of turns
    - Slope of waypoints and check if alternative slopes are matching and one set of coordinates is perpendicular.
- All codes and be launched automatically.
- Code quality checks can be improved. Currently it is simple linting check. For C++ codes we can adapt and write custom scripts which checks if it adhere 
to CMU Coding standards etc.
- And a lot more !

## Contact

For any questions or issues, please contact jonathansumon@gmail.com

<p align="center">
  <em>DFF</em>
</p>
