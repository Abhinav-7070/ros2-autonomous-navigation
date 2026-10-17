# ROS 2 Autonomous Navigation with Nav2 and Gazebo

A ROS 2 Humble simulation project demonstrating autonomous goal navigation using a TurtleBot3 Waffle, Gazebo Classic, Nav2, and a saved 2D map.

## Features
- Custom Gazebo navigation-loop world with obstacles
- 2D mapping using Cartographer
- Saved occupancy-grid maps and Cartographer state files
- AMCL localization on a saved map
- Nav2 global path planning and controller-based path following
- LiDAR, odometry, and TF integration
- Global and local costmap inspection

## Technology Stack
- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic
- TurtleBot3 Waffle
- Nav2
- Cartographer
- Python

## Repository Structure
- `config/` — Cartographer configuration and goals
- `maps/` — saved maps and Cartographer state files
- `scripts/` — Gazebo world-generation script
- `worlds/` — custom Gazebo world
- `continue_mapping.launch.py` — Cartographer continuation launch file

## Navigation Demonstration
The robot was tested in Gazebo using a saved map. Nav2 planned a path from the spawn position to a user-defined goal, and the robot followed the path and stopped at the destination.

## Current Status
The end-to-end simulated navigation test succeeded. Global and local costmap parameters were inspected. Further work includes testing additional goals and obstacle scenarios and improving launch reproducibility.

- Cartographer
- Python

## Repository Structure
- `config/` — Cartographer configuration and goals
- `maps/` — saved maps and Cartographer state files
- `scripts/` — Gazebo world-generation script
- `worlds/` — custom Gazebo world
- `continue_mapping.launch.py` — Cartographer continuation launch file

## Navigation Demonstration
The robot was tested in Gazebo using a saved map. Nav2 planned a path from the spawn position to a user-defined goal, and the robot followed the path and stopped at the destination.

## Current Status
The end-to-end simulated navigation test succeeded. Global and local costmap parameters were inspected. Further work includes testing additional goals and obstacle scenarios and improving launch reproducibility.
