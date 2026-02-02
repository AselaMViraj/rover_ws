# Rover Navigation

This package contains navigation, SLAM, and exploration configurations for the rover robot.

## Contents

- **config/**: Nav2, SLAM Toolbox, and exploration parameters
- **launch/**: Navigation, SLAM, and exploration launch files
- **maps/**: Saved map files
- **scripts/**: Navigation-related Python nodes

## Launch Files

### slam.launch.py
Launches SLAM Toolbox for mapping.

### navigation_launch.py
Launches Nav2 navigation stack.

### exploration_launch.py
Launches autonomous exploration with frontier exploration.

### localization_launch.py
Launches AMCL localization with a pre-existing map.

### online_async_launch.py
Launches SLAM Toolbox in online async mode.

## Scripts

- `waypoint_manager.py`: Manages waypoint navigation goals
- `exploration_manager.py`: Manages exploration behavior

## Usage

```bash
# Start SLAM
ros2 launch rover_navigation slam.launch.py

# Start Navigation
ros2 launch rover_navigation navigation_launch.py

# Start Exploration
ros2 launch rover_navigation exploration_launch.py
```
