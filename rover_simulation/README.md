# Rover Simulation

This package contains Gazebo simulation assets and configuration for the rover robot.

## Contents

- **worlds/**: Gazebo world files
- **models/**: Gazebo model files and assets
- **config/**: Simulation parameters (Gazebo, controllers, joystick)
- **launch/**: Simulation launch files

## Launch Files

### launch_sim.launch.py
Launches Gazebo Classic simulation with the rover.

### launch_ignition.launch.py
Launches Ignition Gazebo simulation with the rover.

**Arguments:**
- `world_file` (default: house.world): World file to load
- `slam` (default: false): Start SLAM alongside simulation

### joystick.launch.py
Launches joystick teleoperation.

## Worlds

- `cafe.world`: Cafe environment
- `house.world`: House environment  
- `obstacles.world`: Obstacle course environment

## Usage

```bash
# Launch Gazebo Classic
ros2 launch rover_simulation launch_sim.launch.py

# Launch Ignition Gazebo with specific world
ros2 launch rover_simulation launch_ignition.launch.py world_file:=cafe.world

# Launch with SLAM
ros2 launch rover_simulation launch_ignition.launch.py slam:=true
```
