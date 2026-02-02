# Rover Description

This package contains the URDF/Xacro robot description and visualization configurations for the rover robot.

## Contents

- **urdf/**: Robot URDF/Xacro files defining the robot model
- **config/**: RViz configuration files for visualization
- **launch/**: Launch files for robot state publisher

## Launch Files

### rsp.launch.py
Launches the robot state publisher with the rover URDF.

**Arguments:**
- `use_sim_time` (default: false): Use simulation time
- `use_ros2_control` (default: true): Enable ros2_control

**Usage:**
```bash
ros2 launch rover_description rsp.launch.py
```

## RViz Configurations

- `drive_bot.rviz`: Configuration for driving the robot
- `main.rviz`: Main visualization configuration
- `map.rviz`: Map-focused visualization
- `view_bot.rviz`: Robot viewing configuration
- `view_robot.rviz`: Alternative robot view
