# Rover Web Interface

This package provides a web-based control and monitoring interface for the rover robot.

## Contents

- **web_ui/**: HTML, CSS, and JavaScript files for the web interface
- **scripts/**: Web server node
- **launch/**: Web interface launch files (if any)

## Features

- Interactive map display with zoom and pan
- Real-time robot position visualization
- Waypoint marking and navigation
- Costmap overlay visualization
- Safety features for waypoint validation

## Scripts

- `web_server.py`: HTTP server for serving web UI files

## Usage

The web interface is typically launched as part of the full system:

```bash
ros2 launch rover_bringup full_system.launch.py
```

Alternatively, start the web server standalone:

```bash
ros2 run rover_web_interface web_server.py
```

Make sure ROSBridge is running:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Then access the interface at `http://localhost:8000`

## Configuration

The web interface connects to ROSBridge WebSocket on port 9090 by default.
