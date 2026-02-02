# Rover Robot - Autonomous Navigation & Exploration System

A complete ROS2-based autonomous mobile robot simulation featuring SLAM mapping, autonomous exploration, path planning, and web-based control interface.

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Classic%20%7C%20Ignition-orange)
![License](https://img.shields.io/badge/license-MIT-green)

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Architecture](#system-architecture)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Usage](#usage)
- [Package Documentation](#package-documentation)
- [Configuration](#configuration)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)

---

## Overview

This project implements a fully autonomous mobile robot with a differential drive configuration (two hub motor wheels and one caster wheel) capable of navigating unknown environments, building maps, and performing autonomous exploration. The system can be used for various autonomous navigation tasks including surveillance, delivery, inspection, and mapping applications.

### What Does This Simulation Do?

The rover simulation provides a complete autonomous robot system that can:

1. **Navigate Autonomously** - Move through environments while avoiding obstacles using Nav2
2. **Build Maps** - Create 2D occupancy grid maps of unknown environments using SLAM
3. **Explore Autonomously** - Discover and map new areas using frontier-based exploration
4. **Accept Commands** - Receive navigation goals through RViz, command line, or web interface
5. **Avoid Obstacles** - Plan safe paths around static and dynamic obstacles
6. **Localize** - Determine its position in known environments using AMCL
7. **Operate in Multiple Environments** - Works in various simulated spaces (cafe, house, obstacle course)

### About the Simulation

### Robot Configuration

The rover is a **differential drive robot** with the following physical configuration:

**Drive System:**
- **2 Hub Motor Wheels** - Independently controlled drive wheels for differential steering
- **1 Caster Wheel** - Passive support wheel for stability
- **Differential Drive Controller** - Enables precise turning and forward/backward motion

**Sensors:**
- **2D Lidar** - 360° laser scanner for obstacle detection and mapping
- **Wheel Odometry** - Estimates position based on wheel rotation

**Additional Components:**
- **Robot Arm** - Manipulator for task execution (model included)

The simulation runs in Gazebo (Classic or Ignition) and provides realistic physics, sensor simulation, and world environments.

---

## Features

### Core Capabilities
- **SLAM Mapping** - Real-time map generation using SLAM Toolbox
- **Autonomous Navigation** - Nav2-based path planning and obstacle avoidance
- **Frontier Exploration** - Automatic discovery and mapping of unknown areas
- **Localization** - AMCL-based localization in known maps
- **Web Interface** - Browser-based robot control and monitoring
- **Multiple Worlds** - Cafe, house, and obstacle course environments
- **Dual Simulator Support** - Gazebo Classic and Ignition Gazebo

### Advanced Features
- Interactive map visualization with real-time updates
- Click-to-navigate waypoint system
- Joystick teleoperation support
- Costmap-based safety validation
- Real-time robot state monitoring
- Map saving and loading

---

## System Architecture

The project follows a modular ROS2 package structure:

```
rover_ws/
├── rover_description/      # Robot URDF model and RViz configs
├── rover_simulation/       # Gazebo worlds, models, and sim launches
├── rover_navigation/       # Nav2, SLAM, exploration configs
├── rover_web_interface/    # Web UI and control server
├── rover_bringup/          # System orchestration launches
└── m-explore-ros2/         # Frontier exploration (external)
```

### Data Flow

```
┌─────────────┐
│   Gazebo    │ ← Simulation Environment
└──────┬──────┘
       │ /scan, /odom, /cmd_vel
       ↓
┌─────────────┐
│  SLAM/Nav2  │ ← Mapping & Navigation
└──────┬──────┘
       │ /map, /goal_pose
       ↓
┌─────────────┐
│ Exploration │ ← Autonomous Discovery
└──────┬──────┘
       │
       ↓
┌─────────────┐
│  Web UI     │ ← User Interface
└─────────────┘
```

---

## Prerequisites

### Required Software

#### Operating System
- **Ubuntu 22.04 LTS** (recommended)
- Other Linux distributions may work but are not officially supported

#### ROS2
- **ROS2 Humble Hawksbill** (recommended)
- ROS2 Iron or later should also work

#### Gazebo
At least one of:
- **Gazebo Classic 11** 
- **Ignition Gazebo (Fortress or Garden)**

### ROS2 Dependencies

Install the following ROS2 packages:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro \
  ros-humble-rviz2 \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ros-gz \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-rosbridge-server \
  ros-humble-twist-mux \
  ros-humble-controller-manager \
  ros-humble-diff-drive-controller \
  ros-humble-joint-state-broadcaster
```

### Additional Dependencies

```bash
# Python dependencies
sudo apt install -y python3-pip
pip3 install flask

# Build tools
sudo apt install -y \
  git \
  python3-colcon-common-extensions \
  python3-rosdep
```

### Hardware Requirements

**Minimum:**
- CPU: Intel i5 or AMD equivalent (4 cores)
- RAM: 8 GB
- GPU: Integrated graphics
- Storage: 10 GB free space

**Recommended:**
- CPU: Intel i7 or AMD equivalent (8 cores)
- RAM: 16 GB
- GPU: Dedicated GPU (NVIDIA/AMD)
- Storage: 20 GB free space

---

## Installation

### 1. Create Workspace

```bash
mkdir -p ~/Desktop/rover_ws/src
cd ~/Desktop/rover_ws/src
```

### 2. Clone Repository

```bash
git clone https://github.com/YOUR_USERNAME/rover_ws.git .
```

### 3. Install Dependencies

```bash
cd ~/Desktop/rover_ws

# Update rosdep
sudo rosdep init  # Only if you haven't run this before
rosdep update

# Install package dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build Workspace

```bash
cd ~/Desktop/rover_ws
colcon build --symlink-install
```

### 5. Source Workspace

```bash
source ~/Desktop/rover_ws/install/setup.bash
```

**Tip:** Add this to your `~/.bashrc` for automatic sourcing:
```bash
echo "source ~/Desktop/rover_ws/install/setup.bash" >> ~/.bashrc
```

---

## Quick Start

### Launch Full System

The easiest way to start everything:

```bash
ros2 launch rover_bringup full_system.launch.py
```

This launches:
- Ignition Gazebo simulation
- SLAM Toolbox for mapping
- RViz visualization
- Web interface
- Navigation stack

### Access Web Interface

Once launched, open your browser to:
```
http://localhost:8000
```

### Send Navigation Goal

**Via RViz:**
1. Click "2D Goal Pose" button
2. Click and drag on the map to set goal position and orientation

**Via Web Interface:**
1. Click on the map where you want the robot to go
2. Waypoint will be validated and sent to navigation

---

## Usage

### Launch Modes

#### 1. SLAM Mode (Default)
Build a map while navigating:
```bash
ros2 launch rover_bringup full_system.launch.py slam:=true
```

#### 2. Exploration Mode
Autonomous frontier-based exploration:
```bash
ros2 launch rover_bringup full_system.launch.py explore:=true slam:=false
```

#### 3. Localization Mode
Navigate with a pre-existing map:
```bash
ros2 launch rover_bringup full_system.launch.py \
  localization:=true \
  slam:=false \
  map:=/path/to/your/map.yaml
```

### World Selection

Choose different environments:

```bash
# Cafe environment
ros2 launch rover_bringup full_system.launch.py world_file:=cafe.world

# House environment
ros2 launch rover_bringup full_system.launch.py world_file:=house.world

# Obstacle course
ros2 launch rover_bringup full_system.launch.py world_file:=obstacles.world
```

### Save a Map

While SLAM is running:

```bash
cd ~/Desktop/rover_ws
ros2 run nav2_map_server map_saver_cli -f src/rover_navigation/maps/my_map
```

This creates:
- `maps/my_map.yaml` - Map metadata
- `maps/my_map.pgm` - Map image

### Individual Components

Launch components separately for testing:

```bash
# Simulation only
ros2 launch rover_simulation launch_ignition.launch.py

# SLAM only (requires simulation running)
ros2 launch rover_navigation slam.launch.py

# Navigation only (requires SLAM/localization)
ros2 launch rover_navigation navigation_launch.py

# Robot description visualization
ros2 launch rover_description rsp.launch.py
```

### Teleoperation

Control the robot manually with keyboard:

```bash
# In a new terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

**Controls:**
- `i` - Forward
- `k` - Stop
- `j` - Turn left
- `l` - Turn right
- `,` - Backward

---

## Package Documentation

### rover_description
Robot URDF model, meshes, and visualization configurations.
- [View Details](rover_description/README.md)

### rover_simulation
Gazebo worlds, models, and simulation launch files.
- [View Details](rover_simulation/README.md)

### rover_navigation
Nav2, SLAM Toolbox, and exploration configurations.
- [View Details](rover_navigation/README.md)

### rover_web_interface
Web-based control interface with real-time visualization.
- [View Details](rover_web_interface/README.md)

### rover_bringup
System orchestration and high-level launch files.
- [View Details](rover_bringup/README.md)

---

## Configuration

### Navigation Parameters

Edit navigation behavior in `rover_navigation/config/nav2_params.yaml`:

```yaml
# Example: Adjust robot speed
controller_server:
  FollowPath:
    max_vel_x: 0.5        # Maximum forward velocity (m/s)
    max_vel_theta: 1.0    # Maximum angular velocity (rad/s)
```

### SLAM Parameters

Configure SLAM in `rover_navigation/config/mapper_params_online_async.yaml`:

```yaml
slam_toolbox:
  resolution: 0.05        # Map resolution (meters/pixel)
  minimum_travel_distance: 0.2  # Minimum distance before updating
```

### Exploration Parameters

Adjust exploration in `rover_navigation/config/explore_params.yaml`:

```yaml
explore:
  robot_base_frame: base_link
  costmap_topic: map
  visualize: true
  planner_frequency: 1.0
```

---

## Troubleshooting

### Gazebo won't start
```bash
# Kill any existing Gazebo processes
killall gzserver gzclient
killall ruby

# Try launching again
ros2 launch rover_bringup full_system.launch.py
```

### Robot not moving
1. Check if navigation received the goal:
   ```bash
   ros2 topic echo /goal_pose
   ```

2. Verify cmd_vel is being published:
   ```bash
   ros2 topic echo /cmd_vel
   ```

3. Check for errors in navigation:
   ```bash
   ros2 node info /controller_server
   ```

### No map appearing
1. Ensure SLAM is running:
   ```bash
   ros2 node list | grep slam
   ```

2. Check scan data:
   ```bash
   ros2 topic echo /scan
   ```

3. Verify transforms:
   ```bash
   ros2 run tf2_tools view_frames
   ```

### Build errors
```bash
# Clean and rebuild
cd ~/Desktop/rover_ws
rm -rf build/ install/ log/
colcon build --symlink-install
```

### Web interface not accessible
1. Check if web server is running:
   ```bash
   ros2 node list | grep web_server
   ```

2. Verify ROSBridge:
   ```bash
   ros2 node list | grep rosbridge
   ```

3. Check firewall settings:
   ```bash
   sudo ufw allow 8000
   sudo ufw allow 9090
   ```

---

## Complete Workflow Examples

### Workflow 1: Autonomous Exploration and Mapping

**Step 1: Build the workspace**
```bash
cd ~/Desktop/rover_ws
colcon build
source install/setup.bash
```

**Step 2: Start SLAM with autonomous exploration**
```bash
ros2 launch rover_bringup full_system.launch.py slam:=true explore:=true world_file:=cafe.world
```

**Step 3: Save the map** (in a separate terminal after exploration completes)
```bash
source ~/Desktop/rover_ws/install/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/Desktop/rover_ws/src/rover_navigation/maps/cafe_map
```

This creates:
- `cafe_map.yaml` - Map metadata
- `cafe_map.pgm` - Map image

---

### Workflow 2: Navigate with Saved Map

**Step 1: Launch with localization mode**
```bash
cd ~/Desktop/rover_ws
source install/setup.bash
ros2 launch rover_bringup full_system.launch.py \
  slam:=false \
  localization:=true \
  map:=~/Desktop/rover_ws/src/rover_navigation/maps/cafe_map.yaml
```

**Step 2a: Send navigation goal via RViz**
- Click the "Nav2 Goal" button in RViz
- Click and drag on the map to set destination and orientation

**Step 2b: Send navigation goal via command line**
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: -0.5, z: 0.0}, orientation: {w: 1.0}}}}"
```

---

### Workflow 3: Web Interface Control

**Step 1: Launch the full system**
```bash
ros2 launch rover_bringup full_system.launch.py
```

**Step 2: Access the web interface**
- On same device: `http://localhost:8000`
- On different device: `http://<YOUR_IP>:8000`

**Step 3: Mark places and navigate**
1. Click on the map to mark locations (e.g., "Kitchen", "Living Room")
2. Saved places appear in the list
3. Click "Go" button to send the robot to that location

---

### Workflow 4: Manual Teleoperation

**Step 1: Launch simulation**
```bash
ros2 launch rover_bringup full_system.launch.py
```

**Step 2: Control with keyboard** (in a separate terminal)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

Use `i`, `j`, `k`, `l`, `,` keys to control the robot.

---

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues.

### Development Workflow

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Build and test (`colcon build && colcon test`)
5. Commit your changes (`git commit -m 'Add amazing feature'`)
6. Push to the branch (`git push origin feature/amazing-feature`)
7. Open a Pull Request

---

## License

This project is licensed under the MIT License - see the LICENSE file for details.

---

## Acknowledgments

- ROS2 Navigation Stack (Nav2) team
- SLAM Toolbox developers
- m-explore-ros2 contributors
- Gazebo simulation team

---

## Contact

For questions and support:
- Open an issue on GitHub
- Email: asela.m.viraj@gmail.com

---

## Roadmap

- [ ] Add 3D point cloud mapping
- [ ] Implement dynamic obstacle avoidance
- [ ] Add multi-robot coordination
- [ ] Integrate object detection
- [ ] Add voice commands
- [ ] Create mobile app interface

---

**Happy Exploring! **
