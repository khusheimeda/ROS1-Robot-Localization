# CS603 Particle Filter (ROS Package)

## Overview

This ROS package implements a **particle filter algorithm** for robot localization using the simulated Triton robot in a Gazebo house environment.

The robot is equipped with a **360° 2D LiDAR** and is controlled via linear and angular velocity commands. The particle filter localizes the robot on a **known map without any prior knowledge of its starting position**.

This package is based on the CS603 Particle Filter starter code, with custom implementations of:
- Motion model
- Sensor model
- Particle filter algorithm

## Prerequisites

- OS: Ubuntu 20.04 LTS  
- ROS: Noetic  
- Simulator: Gazebo  
- Python: Python 3  

### Required Python Packages

```
pip3 install numpy opencv-python pyyaml
```

## Setup Instructions

### 1. Clone and Build

```
cd ~/catkin_ws/src/

# Add the package here (or extract tarball)
# tar -xzf P3D2_firstname_lastname.tar.gz

cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. Make Scripts Executable

```
chmod +x ~/catkin_ws/src/cs603_particle_filter/scripts/particle_filter.py
chmod +x ~/catkin_ws/src/cs603_particle_filter/scripts/motion_model.py
chmod +x ~/catkin_ws/src/cs603_particle_filter/scripts/sensor_model.py
chmod +x ~/catkin_ws/src/cs603_particle_filter/scripts/teleop_particle_filter.py
```

## How to Run

### Launch the Particle Filter

```
roslaunch cs603_particle_filter particle_filter.launch
```

This will:
- Start Gazebo simulation
- Spawn the Triton robot
- Load the map
- Run the particle filter node
- Open RViz

### Control the Robot

In a new terminal:

```
rosrun cs603_particle_filter teleop_particle_filter.py
```

Use your keyboard to move the robot.

### Spawn Robot in a Different Location (Optional)

```
roslaunch cs603_particle_filter particle_filter.launch x:=3.0 y:=2.0 yaw:=1.57
```

| Argument | Description          | Default |
|----------|----------------------|---------|
| x        | X position (m)       | 0.0     |
| y        | Y position (m)       | 0.0     |
| z        | Z position (m)       | 0.0     |
| roll     | Roll (rad)           | 0.0     |
| pitch    | Pitch (rad)          | 0.0     |
| yaw      | Yaw (rad)            | 0.0     |

## RViz Visualization Setup

If not auto-configured:

### Global Settings
- Fixed Frame: `map`

### Add Displays

- PoseArray  
  Topic: `/particlecloud`  
  Arrow Length: `0.3`

- Map  
  Topic: `/map`

- LaserScan (optional)  
  Topic: `/scan`

## Algorithm Description

### 1. Initialization
- 1000 particles randomly distributed over free space
- Random orientation: θ ∈ [-π, π]
- Published on `/particlecloud`

### 2. Prediction (Motion Model)
- Odometry-based motion model
- Noise parameters: α₁ = α₂ = α₃ = α₄ = 0.05
- Triggered by `/odom`

### 3. Correction (Sensor Model)
- Likelihood field model using `/scan`
- Particles weighted based on scan matching
- Invalid particles assigned near-zero weight (1e-20)

### 4. Resampling
- Low-variance (systematic) resampling
- High-weight particles duplicated
- All weights reset to 1/N

### 5. Convergence
- Initially spread across map
- Converge to true pose after movement
- Estimated using weighted average (circular mean)

## ROS Topics

### Subscribed Topics

| Topic     | Type                  | Description          |
|----------|-----------------------|----------------------|
| /odom    | nav_msgs/Odometry     | Motion prediction    |
| /scan    | sensor_msgs/LaserScan | Sensor correction    |
| /cmd_vel | geometry_msgs/Twist   | Movement detection   |

### Published Topics

| Topic            | Type                        | Description              |
|------------------|-----------------------------|--------------------------|
| /particlecloud   | geometry_msgs/PoseArray     | Particle visualization   |
| /estimated_pose  | geometry_msgs/PoseStamped   | Estimated pose           |

## Parameters

| Parameter            | Value  | Description                          |
|----------------------|--------|--------------------------------------|
| num_particles        | 1000   | Number of particles                  |
| alpha1–alpha4        | 0.05   | Motion noise                         |
| update_every_n_scans | 2      | Scan processing rate                 |
| min_dist_to_start    | 0.3 m  | Start threshold (distance)           |
| min_angle_to_start   | 0.3 rad| Start threshold (angle)              |
| min_dist_per_update  | 0.1 m  | Update threshold (distance)          |
| min_angle_per_update | 0.1 rad| Update threshold (angle)             |
| max_odom_jump        | 0.5 m  | Reject large odometry jumps          |