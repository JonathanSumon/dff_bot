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
- [Docker Support](#docker-support)
- [License](#license)
- [Acknowledgments](#acknowledgments)
- [Contact](#contact)

---

## Overview

This ROS 2 package provides support for simulating and controlling the dff_bot robot in a Gazebo environment for this particular Case Study. The package includes various configuration files, launch files, and resources to help you get started with dff_bot simulation and navigation.

## Prerequisites

- Ubuntu 20.04
- ROS 2 Foxy Fitzroy
- Docker

## Installation

1. Clone this repository into your ROS 2 workspace.
2. Build the package using `colcon build`.

## Usage

### Navigation and Mapping

I provide launch files for running navigation and mapping with dff_bot:

- `map_saver.launch.py`: Launches the map server to save maps.
- `nav2_launch.launch.py`: The one stop Launch file to launch the entire world

Please note: First time when you launch the world, it might take some time as it heavy models. 

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

## Contact

For any questions or issues, please contact jonathansumon@gmail.com

<p align="center">
  <em>DFF</em>
</p>
