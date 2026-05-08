# 🧩 Maze Solver Bot — ROS 2 + Gazebo + Nav2

A fully simulated autonomous **maze-solving differential drive robot** built with ROS 2, Gazebo, and the Nav2 navigation stack. The robot uses a 2D LiDAR and onboard camera to localize on a prebuilt map and navigate autonomously through a maze world.

---

## 📦 Package Structure

```
maze_solver_bot/
├── src/
│   ├── gazebo_world/               # Maze world, SDF models, map files
│   │   ├── launch/maze_spawn.launch.py
│   │   ├── map/                    # Occupancy grid maps (.pgm + .yaml)
│   │   ├── models/maze_model_rs/   # Custom maze SDF + mesh (.dae)
│   │   └── worlds/                 # Gazebo world files
│   └── maze_robot/                 # Robot description + motion scripts
│       ├── config/bridge_param.yaml
│       ├── launch/
│       │   ├── display_rviz.launch.py
│       │   └── gazebo_maze.launch.py
│       ├── scripts/square_motion.py
│       ├── urdf/
│       │   ├── robot.xacro         # Differential drive robot URDF (Xacro)
│       │   └── robot.gazebo        # Gazebo-specific plugins
│       └── rviz/rviz_display.rviz
└── my_map.yaml / my_map.pgm       # Saved map for Nav2
```

---

## 🤖 Robot Description

The robot is a **differential drive** platform modeled with physically accurate inertial properties:

| Component | Details |
|-----------|---------|
| Chassis | 1.0 × 0.6 × 0.3 m box, density 2710 kg/m³ |
| Drive Wheels | 2 × continuous joints, radius 0.15 m |
| Caster Wheel | Spherical, front-mounted |
| LiDAR | Mounted on top of chassis |
| Camera | Front-facing, fixed joint |

---

## 🚀 Quickstart

### Prerequisites

```bash
sudo apt install ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-xacro
```

### 1. Clone & Build

```bash
mkdir -p ~/maze_ws/src && cd ~/maze_ws/src
git clone https://github.com/ilovevampire/maze_solver_bot.git .
cd ~/maze_ws
colcon build
source install/setup.bash
```

### 2. Launch Gazebo — Spawn Robot in Maze

```bash
ros2 launch gazebo_world maze_spawn.launch.py
```

### 3. Launch RViz — Visualization

```bash
ros2 launch maze_robot display_rviz.launch.py
```

### 4. Launch Nav2 — Autonomous Navigation

```bash
ros2 launch nav2_bringup bringup_launch.py \
  map:=~/maze_ws/my_map.yaml \
  use_sim_time:=True
```

> ⚠️ Update the `map:=` path to match your system.

### 5. (Optional) Square Motion Test

```bash
ros2 run maze_robot square_motion.py
```

Drives the robot in a 1m × 1m square using `/cmd_vel` at 0.2 m/s linear, 0.5 rad/s angular.

---

## 🔄 Workflow Overview

```
Step 1: Gazebo → Loads maze world + spawns robot
Step 2: RViz   → Visualize TF, LiDAR, costmaps
Step 3: Nav2   → Localize on map + plan path + navigate
```

---

## 🛠️ ROS 2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR data |
| `/map` | `nav_msgs/OccupancyGrid` | Navigation map |
| `/tf` | `tf2_msgs/TFMessage` | Transform tree |

---

## 💡 Future Improvements

- Implement SLAM for real-time map building (no prebuilt map needed)
- Add dynamic obstacle avoidance
- Integrate frontier-based exploration for unknown mazes
- Tune Nav2 costmap parameters for tighter maze corridors

---

## 🔗 Dependencies

- ROS 2 (Jazzy)
- Gazebo (Harmonic)
- Nav2
- `ros_gz_bridge`, `xacro`, `robot_state_publisher`
