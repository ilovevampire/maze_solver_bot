# 🧩 Maze Solver (ROS 2 + Nav2)

This project demonstrates an autonomous **maze-solving robot** using ROS 2, Gazebo simulation, RViz visualization, and the Nav2 navigation stack. The robot is capable of navigating through a predefined maze using a generated map.

---

## 🚀 Getting Started

Follow the steps below to launch the complete simulation environment and navigation system.

---

## 🏗️ 1. Launch Gazebo and Spawn Robot

Start the Gazebo simulation environment and spawn the robot inside the maze world:

```bash
ros2 launch gazebo_world maze_spawn.launch.py
```

📌 This will:

* Open the Gazebo simulator
* Load the maze environment
* Spawn the robot model inside the maze

---

## 👀 2. Launch RViz for Visualization

Open RViz to visualize the robot, sensors, and navigation data:

```bash
ros2 launch maze_robot display_rviz.launch.py
```

📌 This allows you to:

* View robot position and orientation
* Monitor sensor data (e.g., LiDAR, costmaps)
* Set navigation goals interactively

---

## 🧭 3. Start Nav2 Navigation Stack

Launch the Nav2 stack to enable autonomous navigation using the saved map:

```bash
ros2 launch nav2_bringup bringup_launch.py \
map:=/home/vshal/maze_ws/my_map.yaml \
use_sim_time:=True
```

📌 Parameters explained:

* `map`: Path to the YAML map file used for localization and planning
* `use_sim_time`: Ensures synchronization with Gazebo simulation time

---

## 🔄 Workflow Summary

1. Start Gazebo → Loads maze + robot
2. Open RViz → Visual monitoring & goal setting
3. Launch Nav2 → Enables autonomous navigation

---

## ⚠️ Notes

* Ensure all required ROS 2 packages and dependencies are installed
* Verify the map file path is correct for your system
* Run each command in separate terminals for proper execution

---

## 💡 Future Improvements

* Implement advanced path planning algorithms
* Add dynamic obstacle avoidance
* Improve localization accuracy
* Enhance robot perception with additional sensors

---

## 🤝 Contribution

Feel free to fork the project, open issues, or submit pull requests to improve the maze-solving capabilities.

---

Make sure to map area precisely to get better nav2 output

