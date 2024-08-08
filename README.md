# butlerbot

## Overview

The Butlerbot project aims to develop a robotic butler designed to operate in a café environment. The robot will autonomously deliver food from the kitchen to customers at their tables, optimizing efficiency and reducing the need for human staff during busy hours.


## Development Environment

This project uses a Dockerized VSCode devcontainer environment for development, based on the [devcontainer_for_ros template](https://github.com/manojm-dev/devcontainer_for_ros). This setup ensures a consistent development environment across different systems and simplifies the setup process.

You can also choose to develop using a normal ROS host machine with ROS 2 Humble installed if you prefer. However, the Dockerized environment provides additional convenience and consistency for development across various platforms.

## How to Build

### In above mention devcontainer environment

1. use vscode tasks shortcut and choose: `Install Dependencies`
2. use vscode tasks shortcut and choose: `Build RelWithDebInfo`
3. type `sourcews` in terminal for sourcing the workspace

### In normal ros humble installed in host machine

1. create a workspace
```
mkdir -p butlerbot_ws/src
```
2. Clone this repo in src 
```
cd butlerbot_ws
git clone https://github.com/manojm-dev/butlerbot.git src
```
3. Install Dependencies
```
rosdep install --from-path src --ignore-src
```
4. Building
```
 colon build
```
5. Sourcing the workspace
```
source install/setup.bash
```


## Project Structure

The project is organized into several ROS2 packages, each handling different aspects of the robot's functionality:

1. **butlerbot_description**: Contains the robot's URDF/XACRO files and launch files, enabling visualization and simulation of Butlerbot in various environments.
2. **butlerbot_gazebo**: Includes configurations and launch files for simulating Butlerbot in the Gazebo environment, allowing for realistic physics and sensor simulation.
3. **butlerbot_localization**: Manages the robot's localization using SLAM or other localization techniques, ensuring accurate positioning within the café environment.
4. **butlerbot_navigation**: Implements the ROS2 Navigation stack (Nav2) for path planning, obstacle avoidance, and autonomous movement of Butlerbot throughout the café.

## Packages Description

### 1) butlerbot_description

The butlerbot_description package provides the URDF/XACRO files and launch configurations for visualizing Butlerbot robot. It includes files to manage robot state publishing, visualization in RViz, and other related tools.

#### Launch files

1) **display.launch.py**: Launches the robot state publisher, joint state publisher, joint state publisher GUI, RViz, and Rqt. The default parameters are : `use_sim_time:=true`, `use_jsp:=true` ,`jsp_gui:=false`, `urdf_model:=defalt_location`, `use_rviz:=true` and `use_rqt:=false`.

2) **rsp.launch.py**: Launches the robot state publisher node with configuration parameters. The default parameters are: `urdf_model:=default_location`, `use_sim_time:=true` and future scope parameters `use_gazebo:=true`, `use_gzsim:=false`.

3) **visualize.launch.py**: Launches RViz and Rqt visualization tools with configurable parameters. The default parameters are: `use_sim_time:=true`, `use_rviz:=true` and `use_rqt:=false`

### 2) butlerbot_gazebo
The butlerbot_gazebo package will include configurations and launch files for simulating the Butlerbot in Gazebo, with realistic physics and sensor simulation.

### 3) butlerbot_localization 
The butlerbot_localization package will manage the robot's localization using techniques such as SLAM, ensuring accurate positioning within the café environment.

### 4) butlerbot_navigation 
The butlerbot_navigation package will implement the ROS2 Navigation stack (Nav2) for path planning, obstacle avoidance, and autonomous movement of the Butlerbot throughout the café.



