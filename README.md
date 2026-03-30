Enhancing Night-Task Navigation for Competition Rovers: A Comparative Study of Gazebo Simulations and Sensor Performance

This repository contains the simulation environments, ROS2 nodes, and experimental data associated with the research paper titled "Enhancing Night-Task Navigation for Competition Rovers: A Comparative Study of Gazebo Simulations and Sensor Performance," developed for the Artificial Life and Robotics journal.
Abstract

The European Rover Challenge (ERC) introduces significant operational constraints for autonomous navigation, most notably the "Night Task," which requires reliable perception in near-zero lux environments. This work presents a "Simulation-to-Real" methodology to evaluate and improve the detection of ArUco markers under extreme low-light conditions. Utilizing the ROS2 Jazzy framework and Gazebo (Harmonic), we simulated a mobile rover equipped with an Intel RealSense D435i depth camera and a synchronized spotlight. We measured performance through Detection Success Rate and Pose Estimation Error, validating virtual results against physical experiments conducted at the University of West Bohemia (UWB) workshop.
Key Contributions

    Validated Sensor Profile: A Gazebo-Harmonic sensor profile optimized for low-light ArUco detection.

    Comparative Analysis: A study of pose estimation drift between virtual and physical IR sensors under varying illumination.

    Sim-to-Real Pipeline: A scalable framework for testing autonomous vision systems in space exploration contexts where physical testing is logistically constrained.

Repository Structure
Plaintext

├── docs/               # Documentation and paper drafts
├── simulation/         # Gazebo Harmonic worlds and models
├── ros2_ws/            # ROS2 workspace (Jazzy/Humble/Iron)
│   ├── src/
│   │   ├── rover_description/   # URDF/Xacro for the D435i and spotlight
│   │   ├── perception_nodes/    # OpenCV detection and thresholding logic
│   │   └── data_logger/         # Nodes for recording detection metrics
├── data/               # Recorded bag files and experimental logs
└── LICENSE             # MIT License

System Overview
Hardware

    Camera: Intel RealSense D435i (Global shutter and IR capabilities).

    Lighting: Synchronized spotlight mounted 10–15 cm below the camera.

    Validation Tools: Lux meter for standardizing light levels (0.1 to 10 lux).

Software Stack

    Framework: ROS2 (Jazzy/Humble/Iron).

    Simulator: Gazebo Harmonic (formerly Ignition).

    Visualization: Rviz2.

    Vision: OpenCV-based ArUco detection pipeline.

Methodology & Metrics

The performance of the system is evaluated across two primary metrics:

    Detection Success Rate: The ratio of frames where the ArUco marker was successfully identified by the detectMarkers function over the total frames in a 30-second trajectory.

    Pose Estimation Error: The mathematical difference (Translation Δx,Δy,Δz and Rotation variance) between the actual marker position and the position estimated by the camera.

Getting Started

    Clone the Repository:
    Bash

    git clone https://github.com/raf-pimentel/paper-erc-night-task-simulation.git

    Install Dependencies: Ensure you have ROS2 and Gazebo Harmonic installed.

    Build the Workspace:
    Bash

    cd ros2_ws && colcon build

    Launch Simulation:
    Bash

    ros2 launch rover_description simulation.launch.py

Acknowledgments

This research was conducted as part of the visiting researcher program at the University of West Bohemia (UWB), collaborating with the UWB Robotics Team and Professor Tomas Broum.
License

This software is provided "as is" under the MIT License. For full details, see the LICENSE file.
