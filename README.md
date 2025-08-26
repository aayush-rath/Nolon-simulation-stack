# Nolon Bot

A comprehensive ROS2 package for a mobile manipulator robot with navigation, mapping, and manipulation capabilities.

## Overview

The Nolon Bot is a mobile manipulator system that combines:
- **Mobile Base**: Differential drive robot with omnidirectional wheels
- **Robotic Arm**: 6-DOF manipulator arm with end-effector
- **Sensors**: Lidar for navigation and camera for perception
- **Containment System**: Specialized tools for cleaning applications

<div align="center">
<img src="images/Nolon_Bot.png">
</div>

## Features

- **ROS2 Jazzy** support with Gazebo Ionic simulation
- **SLAM** mapping using Nav2 stack
- **Autonomous Navigation** with obstacle avoidance
- **MoveIt2** motion planning for the robotic arm
- **Teleoperation** support for manual control
- **Docker** containerized deployment
- **RViz** visualization configurations

## System Requirements

- **OS**: Ubuntu 24.04 LTS
- **ROS2**: Jazzy Jalisco
- **Gazebo**: Ionic
- **Docker**: 20.10+ with Docker Compose v2
- **Hardware**: Minimum 4GB RAM, GPU recommended for simulation

## Quick Start with Docker

### 1. Clone the Repository
```bash
git clone https://github.com/aayush-rath/Nolon-simulation-stack.git
cd Nolon-simulation-stack
```

### 2. Build and Run with Docker Compose
```bash
# Build the Docker image
docker compose build

# Run the complete system
docker compose up

# For headless operation
docker compose up -d
```

### 3. Access the System
- **RViz**: Connect to the container's display
- **Gazebo**: GUI will be available through X11 forwarding
- **Logs**: `docker compose logs -f`

## Native Installation

### Dependencies
```bash
# Install ROS2 Jazzy
sudo apt update
sudo apt install ros-jazzy-desktop-full

# Install additional packages
sudo apt install \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-moveit \
    ros-jazzy-moveit-servo \
    ros-jazzy-gazebo-ros-pkgs \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    ros-jazebo-ros2-control
```

### Build from Source
```bash
# Create workspace
mkdir -p ~/nolon_ws/src
cd ~/nolon_ws/src

# Clone repository
git clone <your-repo-url> .

# Install dependencies
cd ~/nolon_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

## Usage

### 1. Simulation
Launch the complete simulation environment:
```bash
ros2 launch nolon_bot_description sim.launch.py
```

### 2. Mapping
Create a map of the environment:
```bash
# Start mapping
ros2 launch nolon_bot_description mapping.launch.py

# Save the map when done
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### 3. Navigation
Navigate autonomously using the created map:
```bash
ros2 launch nolon_bot_description navigation.launch.py
```

### 4. Manipulation
Control the robotic arm with MoveIt2:
```bash
ros2 launch nolon_bot_description move_group.launch.py
```

### 5. Teleoperation
Manual control of the robot:
```bash
# Keyboard teleoperation
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Or use the custom teleop nodes
ros2 run nolon_bot_description teleop_node
```

## Package Structure

```
nolon_bot_description/
├── config/           # Configuration files
│   ├── nolon_bot_control.yaml
│   ├── nolon_bot_mapper.yaml
│   └── nolon_bot_navigation.yaml
├── launch/           # Launch files
│   ├── sim.launch.py
│   ├── mapping.launch.py
│   ├── navigation.launch.py
│   └── move_group.launch.py
├── maps/             # Saved maps
├── meshes/           # 3D models
│   ├── mobile_base/
│   ├── robotic_arm/
│   ├── sensors/
│   └── containment/
├── moveit_config/    # MoveIt2 configurations
├── rviz/             # RViz configurations
├── src/              # Source code
├── urdf/             # Robot descriptions
└── world/            # Gazebo worlds
```

## Configuration

### Navigation Parameters
Edit `config/nolon_bot_navigation.yaml` to tune:
- Global planner settings
- Local planner parameters
- Costmap configurations
- Recovery behaviors

### MoveIt2 Configuration
Modify files in `moveit_config/`:
- `joint_limits.yaml`: Joint constraints
- `kinematics.yaml`: Kinematics solvers
- `ompl_planning.yaml`: Motion planning algorithms
- `servo.yaml`: Real-time servo control

### Control Parameters
Adjust `config/nolon_bot_control.yaml` for:
- Joint controllers
- Differential drive controller
- Hardware interface settings

## Advanced Usage

### Custom Worlds
Add your own Gazebo worlds to the `world/` directory and reference them in launch files:
```python
world_file = os.path.join(pkg_share, 'world', 'your_custom_world.sdf')
```

### Additional Tools
The robot includes specialized tools:
- `toilet_brush.stl`: Cleaning brush attachment
- `urinal_nozzle.stl`: Liquid dispenser
- `some_tool.stl`: Generic tool interface

### Multi-Robot Support
Launch multiple robots by modifying the namespace in launch files:
```python
DeclareLaunchArgument('robot_name', default_value='nolon_bot_1')
```

## Development

### Adding New Sensors
1. Create mesh files in `meshes/sensors/`
2. Add URDF description in `urdf/sensors/`
3. Update the main robot URDF
4. Configure in launch files

### Custom Behaviors
Implement custom navigation behaviors by:
1. Creating plugins for Nav2
2. Adding behavior trees
3. Configuring in `nolon_bot_navigation.yaml`

### Testing
```bash
# Run unit tests
colcon test --packages-select nolon_bot_description

# Check code style
ament_flake8 src/
ament_cpplint src/
```

## Troubleshooting

### Common Issues

**Gazebo won't start**
```bash
# Check Gazebo installation
gazebo --version
# Ensure proper environment sourcing
source /opt/ros/jazzy/setup.bash
```

**TF errors**
```bash
# Check TF tree
ros2 run tf2_tools view_frames
# Verify robot_state_publisher is running
ros2 node list | grep robot_state_publisher
```

**Navigation not working**
```bash
# Check navigation stack
ros2 topic list | grep nav
# Verify map is loaded
ros2 topic echo /map --once
```

**MoveIt2 planning fails**
```bash
# Check joint states
ros2 topic echo /joint_states
# Verify planning scene
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene
```

### Performance Optimization

**For better simulation performance:**
- Reduce mesh complexity
- Lower physics update rate
- Disable unnecessary sensors
- Use headless mode when possible

**For real robot deployment:**
- Tune controller gains
- Adjust navigation timeouts
- Optimize sensor frequencies
- Configure hardware interfaces

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

This project is licensed under the terms specified in the LICENSE file.

## Support

For issues and questions:
- Create GitHub issues for bugs
- Use discussions for general questions
- Check documentation in `docs/` directory

## Acknowledgments

- ROS2 Navigation2 team
- MoveIt2 developers
- Gazebo simulation community
- Open source robotics community
- Xarm URDF
