# Car-Like Robot EKF ROS2 Package

This repository provides a **C++ Extended Kalman Filter (EKF)** implementation for a car-like robot in **ROS 2 (Kilted)**, along with a **Python simulator**, data logging, and post‑processing scripts for trajectory visualization and animation.

---

## Repository Structure

```
src/kalman_filter/
├── CMakeLists.txt
├── package.xml
├── include/
│   ├── KalmanFilter.hpp
│   └── EKF.hpp
├── src/
│   ├── EKF.cpp
│   ├── EKF_ros_node.cpp
│   ├── kalman_filter_ros_node.cpp
│   ├── KalmanFilter.cpp
│   ├── PID_controller.cpp
│   └── reference_path_publisher.cpp
├── scripts/
│   └── car_like_sim.py
├── kf_pose.csv
├── true_pose.csv
├── runs
│   ├── EKF_vs_GT_trajectory_Carlike_robot.png
│   ├── EKF_vs_GT_animation.gif
│   ├── kf_pose.csv
│   ├── true_pose.csv
│   ├── plot_traj.py
│   ├── metrics_logger.py
│   ├── Rviz_output_pursue_pid.mp4
│   └── Rviz_output_pursue_pid.gif
└── README.md
```

---

## Overview

This package demonstrates an EKF for 2D pose estimation of a car-like robot:

- **State vector**:  
  \[ x, y, θ, v \]  
  – position, heading, velocity
- **Inputs**:  
  \[ a, ω \]  
  – longitudinal acceleration (IMU), yaw rate (IMU)
- **Measurements**:  
  \[ v \]  
  – velocity from wheel encoder

A Python script simulates the robot motion and publishes IMU and encoder data, as well as ground truth pose. The EKF node fuses these to estimate the robot’s trajectory.

---

## Requirements

- Ubuntu 24.04 / WSL2  
- ROS 2 Kilted  
- C++17 compiler  
- Python 3 (system), with:
  - `rclpy`
  - `sensor_msgs`
  - `std_msgs`
  - `geometry_msgs`
- Python dependencies for analysis:
  - `pandas`
  - `numpy`
  - `matplotlib`

---

## Installation & Build

```bash
# Clone and build in your ROS 2 workspace
cd ~/ros2_ws/src
git clone https://github.com/AayushmanSharma96/ROS2-Sensor-Fusion.git
cd ~/ros2_ws
colcon build --packages-select kalman_filter
source install/setup.bash
```

---

## Running the Simulation & EKF

### 1. Launch the Python Simulator

```bash
ros2 run kalman_filter car_like_sim.py
```

This node publishes:
- `/imu/data`        → `sensor_msgs/msg/Imu`
- `/encoder/speed`   → `std_msgs/msg/Float64`
- `/sim/true_pose`   → `geometry_msgs/msg/PoseStamped`

### 2. Launch the EKF Node

In a new terminal (after sourcing):

```bash
ros2 run kalman_filter EKF_ros_node
```

This node subscribes to IMU and encoder, runs the EKF, and publishes:
- `/kf/pose`        → `geometry_msgs/msg/PoseStamped`

---

## Data Logging

Record both EKF and ground truth for offline analysis:

```bash
ros2 topic echo /kf/pose --csv > kf_pose.csv
ros2 topic echo /sim/true_pose --csv > true_pose.csv
```

---

## Plots & Animation

### Static Trajectory Plot

```bash
python3 scripts/animate_kf.py
# Generates:
#  - EKF_vs_GT_trajectory_Carlike_robot.png
#  - EKF_vs_GT_animation_fast_dashed.gif
```

![Static Trajectory](src/kalman_filter/runs/EKF_vs_GT_trajectory_Carlike_robot.png)

### Animated Comparison

![Animated EKF vs GT](src/kalman_filter/runs/EKF_vs_GT_animation.gif)

---

## Tuning Tips

- **Process noise Q**: Increase `Q(2,2)` (heading) to allow more flexibility in gyro integration.  
- **Measurement noise R**: Adjust based on encoder accuracy.  
- **Sampling rate**: 50 Hz is realistic for car-like robots.

---

## Contributing

Pull Requests and Issues are welcome!  
Please include:
- Clear description of changes
- Updated plots or metrics if behavior changes

---

## License

This project is licensed under the **MIT License**.  
