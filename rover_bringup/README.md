# Rover Bringup

Main orchestration package for launching the complete rover robot system.

## Contents

- **launch/**: System-level launch files
- **scripts/**: System-level utility scripts

## Launch Files

### full_system.launch.py
Launches the complete rover system including simulation, navigation, and web interface.

**Arguments:**
- `world_file` (default: cafe.world): Gazebo world file to load
- `use_rviz` (default: true): Launch RViz for visualization
- `slam` (default: true): Enable SLAM mapping
- `explore` (default: false): Enable autonomous exploration
- `localization` (default: false): Use localization with existing map
- `map` (default: ''): Map file for localization

**Usage:**
```bash
# Launch full system with SLAM
ros2 launch rover_bringup full_system.launch.py

# Launch with exploration
ros2 launch rover_bringup full_system.launch.py explore:=true

# Launch with localization
ros2 launch rover_bringup full_system.launch.py localization:=true map:=/path/to/map.yaml slam:=false
```

### launch_robot.launch.py
Launches the rover on real hardware (non-simulation).

## Scripts

- `odom_tf_publisher.py`: Publishes odom→base_link transform

## Package Dependencies

This package depends on:
- `rover_description`: Robot model
- `rover_navigation`: Navigation and SLAM
- `rover_simulation`: Simulation environments
- `rover_web_interface`: Web control interface
