# Autonomous Underwater Black Box Retrieval Using BlueROV2

This repository contains the implementation for an autonomous underwater black box retrieval system using BlueROV2. The project integrates ROS, Unity simulation, and real-world hardware to achieve tasks such as ArUco marker detection, precise control, and object recovery.

## Table of Contents
- [Overview](#overview)
- [System Requirements](#system-requirements)
- [Installation](#installation)
- [Project Structure](#project-structure)
- [Launching the System](#launching-the-system)
- [Running Tests and Validation](#running-tests-and-validation)
- [Contributing](#contributing)
- [License](#license)

---

## Overview
This project focuses on developing a modular system for underwater black box detection and recovery. The key features include:
- **ArUco Marker Detection:** Real-time detection and pose estimation.
- **PD Control Implementation:** Depth, lateral, forward, and yaw control.
- **Behavior Tree:** High-level decision-making for manual and autonomous modes.
- **Unity Simulation Integration:** Testing in simulated underwater environments.

---

## System Requirements
### Hardware
- BlueROV2
- Newton-Gripper
- Camera with ArUco detection capability

### Software
- Ubuntu 20.04
- ROS 2 Foxy Fitzroy
- Python 3.8+
- Unity 2021 or later
- MAVROS (ROS package for PX4 communication)

---

## Installation

### 1. Clone the Repository
```bash
cd ~
git clone https://github.com/eather0056/Autonomous-Underwater-Black-Box-Retrieval-Using-BlueROV2.git
cd autonomous-underwater
```

### 2. Install ROS Dependencies
Ensure that ROS 2 Foxy is installed on your system. Install the necessary ROS dependencies:
```bash
sudo apt update
sudo apt install -y ros-foxy-mavros ros-foxy-mavros-extras
```

### 3. Build the Workspace
Initialize and build the workspace:
```bash
colcon build --packages-select bluerov2_bringup bluerov2_controller bluerov2_interfaces
source install/setup.bash
```

### 4. Additional Configurations
- Ensure MAVROS is configured for your BlueROV2 setup.
- Connect your BlueROV2 to your local network with the correct IP (`192.168.2.1`).

---

## Project Structure
```
├── src
│   ├── bluerov2_bringup
│   │   ├── launch
│   │   ├── rviz
│   │   └── CMakeLists.txt
│   ├── bluerov2_controller
│   ├── bluerov2_interfaces
│   ├── bluerov_grasp
│   └── tritech_micron
├── build
├── install
├── log
├── README.md
└── ProjectOverview.md
```
- **`bluerov2_bringup`**: Launch files and configurations for initializing the robot.
- **`bluerov2_controller`**: Nodes for controlling the robot and processing input.
- **`bluerov2_interfaces`**: Custom ROS 2 message and service definitions.
- **`bluerov_grasp`**: Grasping-related functionality.

---

## Launching the System

### 1. Start MAVROS Node
First, launch the MAVROS node to establish communication with the BlueROV2:
```bash
ros2 launch mavros node.launch
```

### 2. Launch the Full System
Run the `run_all_test.launch.py` file to initialize all necessary nodes:
```bash
ros2 launch bluerov_grasp run_all_test.launch.py
```

Alternatively, you can launch individual components using the following:
```bash
ros2 launch bluerov2_bringup depth_config.launch.py
ros2 launch bluerov2_controller last_dance.launch.py
```

### 3. Control the Robot
- Use the Python GUI or a joystick to control the robot manually.
- Activate the autonomous mode for ArUco-based navigation.

---

## Running Tests and Validation

### 1. Unity Simulation
- Ensure Unity is running the simulated environment.
- Connect Unity to ROS using the ROS-TCP-Connector.

```bash
https://github.com/eather0056/Telerobotics-and-HRI-Simulation-Server.git
```

### 2. Real-World Testing
- Connect BlueROV2 and verify camera and MAVROS connectivity.
- Validate depth and marker detection nodes using:
```bash
ros2 topic echo /aruco_pose
```
## Project Demo

[![Watch the demo on YouTube](assets/demo-thumbnail.png)](https://youtube.com/shorts/35pSupqHFNI?feature=share)

[![Watch the demo on YouTube](assets/demo-thumbnail.png)](https://youtu.be/mVpci4n3xCk)

[![Watch the demo on YouTube](assets/demo-thumbnail.png)](https://youtu.be/t4nXIaApkf0)


---

## Contributing
Contributions are welcome! Please follow these steps:
1. Fork the repository.
2. Create a feature branch.
3. Commit your changes.
4. Submit a pull request.

---
