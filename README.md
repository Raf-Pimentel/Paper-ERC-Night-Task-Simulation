# Enhancing Night-Task Navigation for Competition Rovers: A Comparative Study of Gazebo Simulations and Sensor Performance

This repository contains the simulation environments, ROS2 nodes, and experimental data associated with the research paper titled "Enhancing Night-Task Navigation for Competition Rovers: A Comparative Study of Gazebo Simulations and Sensor Performance," developed for the Artificial Life and Robotics journal.
Abstract

The European Rover Challenge (ERC) introduces significant operational constraints for autonomous navigation, most notably the "Night Task," which requires reliable perception in near-zero lux environments. This work presents a "Simulation-to-Real" methodology to evaluate and improve the detection of ArUco markers under extreme low-light conditions. Utilizing the ROS2 Jazzy framework and Gazebo (Harmonic), we simulated a mobile rover equipped with an Intel RealSense D435i depth camera and a synchronized spotlight. We measured performance through Detection Success Rate and Pose Estimation Error, validating virtual results against physical experiments conducted at the University of West Bohemia (UWB) workshop.
This research was conducted as part of the visiting research program at the University of West Bohemia (UWB), collaborating with the UWB Robotics Team and Professor Tomas Broum.

Key Contributions

    Validated Sensor Profile: A Gazebo-Harmonic sensor profile optimized for low-light ArUco detection.

    Comparative Analysis: A study of pose estimation drift between virtual and physical IR sensors under varying illumination.

    Sim-to-Real Pipeline: A scalable framework for testing autonomous vision systems in space exploration contexts where physical testing is logistically constrained.

## Repository Structure
(Working on it)

## System Overview
Hardware

    Camera: Intel RealSense D435i (Global shutter and IR capabilities).

    Lighting: Synchronized spotlight mounted 10–15 cm below the camera.

    Validation Tools: Lux meter for standardizing light levels (0.1 to 10 lux).

Software Stack

    Framework: ROS2 (Jazzy/Humble/Iron).

    Simulator: Gazebo Harmonic (formerly Ignition).

    Visualization: Rviz2.

    Vision: OpenCV-based ArUco detection pipeline.

## Methodology & Metrics

The performance of the system is evaluated across two primary metrics:

    Detection Success Rate: The ratio of frames where the ArUco marker was successfully identified by the detectMarkers function over the total frames in a 30-second trajectory.

    Pose Estimation Error: The mathematical difference (Translation Δx,Δy,Δz and Rotation variance) between the actual marker position and the position estimated by the camera.

## Running the Simulation (Docker)

Since this environment runs within a Docker container, follow these steps to initialize the simulation, the ROS2 bridge, and the experimental scripts across four terminal tabs.
0. Cloning repository and Giving Permissions:
Clone the repository:
Bash
git clone https://github.com/raf-pimentel/paper-erc-night-task-simulation.git

(In your host machine) Ensure you gave all the due permissions with:

Bash
xhost +local:root

[comment]: <You should see something like: "non-network local connections being added to access control list">

### 1. Start the Container

First, ensure your container is running and identify its ID:
Bash

docker ps -a

[//]: < Look at the CONTAINER_ID>

Bash
docker start <CONTAINER_ID>

[//]: <For all the other terminals that you will run the docker environment simultaneously, you use the command: docker exec -it [CONTAINER_ID] bash>

### Terminal 1: Launch Gazebo Harmonic

Enter the container, export the Gazebo resource paths, and launch the "Night Task" world:
Bash

docker exec -it <CONTAINER_ID> bash
cd /ros2_ws
export GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins:${GZ_SIM_SYSTEM_PLUGIN_PATH}
export GZ_SIM_RESOURCE_PATH=/ros2_ws/src/uwb_erc_sim/models:${GZ_SIM_RESOURCE_PATH}

gz sim /ros2_ws/src/uwb_erc_sim/worlds/night_task.sdf

### Terminal 2: ROS-Gazebo Bridge

Open a new tab to bridge the communication between Gazebo and ROS2 Jazzy:
Bash

docker exec -it <CONTAINER_ID> bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
    /camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
    /model/aruco_target/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist

### Terminal 3: Movement Script

Open a third tab to execute the ArUco marker trajectory:

``
docker exec -it <CONTAINER_ID> bash
cd /ros2_ws
python3 src/uwb_erc_sim/scripts/fly_square.py
``

### Terminal 4: Vision Evaluation

Open a fourth tab to run the automated detection and pose estimation analysis:
Bash

``
docker exec -it <CONTAINER_ID> bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
``

# Note: This script is currently under development
python3 src/uwb_erc_sim/scripts/erc_vision_eval.py

## Acknowledgments

This research was conducted as part of the visiting researcher program at the University of West Bohemia (UWB), collaborating with the UWB Robotics Team and Professor Tomas Broum.
License

This software is provided "as is" under the MIT License. For full details, see the LICENSE file.
