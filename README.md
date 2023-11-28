# gazebo-tutorials

## [License](LICENSE)

## Overview

This package runs a turtlebot in a gazebo world and implements the walker algorithim on it.

## Building and Running

Clone the repository in your ROS2 workspace.

```sh
cd < path_to_your_workspace >/src

# For cloning using SSH
git clone git@github.com:KshitijKarnawat/gazebo_tutorials.git

```

### Building

To build the package follow the following steps

```sh
cd .. # Make sure you are in the workspace folder and not in src

# This will build all the packages in your workspace
colcon build

# To build only this package use
colcon build --package-select gazebo_tutorials

# Source your setup file
source install/setup.bash
```

### Running

```sh
cd < path_to_your_workspace >

# Using ROS2 Launch
ros2 launch gazebo_tutorials walker.py record:=< true or false >
```

### Running ROSBAG

To record the data published by the `publisher` and `server` we can use ROSBAG.

```sh
cd < path_to_your_workspace >

# Source your setup file
. install/setup.bash

# Runs the bag file
ros2 bag play src/beginner_tutorials/bagfiles/tutorial_bagfile
```

## Dependencies

- [ROS2 Humble](https://docs.ros.org/en/humble/index.html)
- CMake Version 3.8 or greater
- C++ 17 or newer
- [Gazebo](http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros)
- Turtlebot3
- Turtlebot3 Gazebo

## Assumptions

The above instruction assume that you have installed all the Dependecies and are working on a Ubuntu 22.04 LTS system and have created your ROS2 workspace beforehand.

## Results

Results for `cppcheck`, `cpplint` can be viewed in `results` folder
