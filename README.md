# Enhancing Night-Task Navigation for Competition Rovers: A Comparative Study of Gazebo Simulations and Sensor Performance

Some more detailed explanation: [Docs](https://docs.google.com/document/d/1aZhX8T2QJ3hH65VPwwKCBMxovXjShG7vIbJGwNSziSs/edit?usp=sharing)

This repository contains the simulation environments, ROS2 nodes, and experimental data associated with the research paper titled "Enhancing Night-Task Navigation for Competition Rovers: A Comparative Study of Gazebo Simulations and Sensor Performance," developed for the Artificial Life and Robotics journal.

## Abstract

The European Rover Challenge (ERC) introduces significant operational constraints for autonomous navigation, most notably the "Night Task," which requires reliable perception in near-zero lux environments. This work presents a "Simulation-to-Real" methodology to evaluate and improve the detection of ArUco markers under extreme low-light conditions. Utilizing the ROS2 Jazzy framework and Gazebo (Harmonic), we simulated a mobile rover equipped with an Intel RealSense D435i depth camera and a synchronized spotlight. We measured performance through Detection Success Rate and Pose Estimation Error, validating virtual results against physical experiments conducted at the University of West Bohemia (UWB) workshop.

This research was conducted as part of the visiting research program at the University of West Bohemia (UWB), collaborating with the UWB Robotics Team and Professor Tomas Broum.

## Key Contributions

- **Validated Sensor Profile:** A Gazebo-Harmonic sensor profile optimized for low-light ArUco detection.
- **Comparative Analysis:** A study of pose estimation drift between virtual and physical IR sensors under varying illumination.
- **Sim-to-Real Pipeline:** A scalable framework for testing autonomous vision systems in space exploration contexts where physical testing is logistically constrained.

---

## Repository Structure

```
├── luminosity_test_results.csv
├── night_task_results.csv
├── performance_results.csv
└── src
    └── uwb_erc_sim
        ├── CMakeLists.txt
        ├── include
        │   └── uwb_erc_sim
        ├── launch
        │   └── simulation.launch.py
        ├── models
        │   ├── aruco_marker
        │   │   └── materials
        │   │       └── textures
        │   │           ├── NewAruCo.png
        │   │           └── marker.png
        │   └── aruco_target
        │       ├── materials
        │       │   ├── scripts
        │       │   │   └── ArUcoMarker.material
        │       │   └── textures
        │       │       ├── NewArUco.png       ← valid DICT_7X7_50 ID=0 marker
        │       │       └── aruco_marker.png
        │       ├── model.config
        │       └── model.sdf
        ├── package.xml
        ├── scripts
        │   ├── aruco_detector.py
        │   ├── erc_vision_eval.py
        │   ├── experiment_automator.py
        │   ├── fly_square.py
        │   ├── intensity_sweeper.py
        │   ├── light_calibration.py
        │   └── vision_node.py
        ├── setup.py
        ├── urdf
        │   └── d435i.urdf
        └── worlds
            ├── night_task.sdf
            ├── night_task_19h.sdf    ← Twilight 19:00 — 3.0 lx
            ├── night_task_21h.sdf    ← Evening 21:00 — 0.3 lx
            ├── night_task_midnight.sdf ← Midnight 00:00 — 0.1 lx
            └── standard.sdf
```

---

## System Overview

### Hardware

- **Camera:** Intel RealSense D435i (Global shutter and IR capabilities).
- **Lighting:** Synchronized spotlight mounted 10–15 cm below the camera.
- **Validation Tools:** Lux meter for standardizing light levels (0.1 to 10 lux).

### Software Stack

- **Framework:** ROS2 Jazzy
- **Simulator:** Gazebo Harmonic (gz-sim 8)
- **Visualization:** Rviz2
- **Vision:** OpenCV-based ArUco detection pipeline (`DICT_7X7_50`)

---

## Methodology & Metrics

The performance of the system is evaluated across two primary metrics:

- **Detection Success Rate:** The ratio of frames where the ArUco marker was successfully identified by `detectMarkers` over the total frames in a 30-second trajectory.
- **Pose Estimation Error:** The mathematical difference (Translation Δx, Δy, Δz and Rotation variance) between the actual marker position and the position estimated by the camera.

### Simulation Results (Gazebo)

Results were obtained by processing screen recordings of each scenario with `process_recordings.py`. Three runs were performed per lighting condition:

| Scenario | Target Lux | Run | Detection Rate | Mean Depth Error |
|---|---|---|---|---|
| Twilight 19:00 | 3.0 lx | run1 | 81.2% | +1.264 m |
| Twilight 19:00 | 3.0 lx | run2 | 96.2% | +1.373 m |
| Twilight 19:00 | 3.0 lx | run3 | 97.6% | +1.365 m |
| Evening 21:00  | 0.3 lx | run1 | 89.4% | +1.371 m |
| Evening 21:00  | 0.3 lx | run2 | 92.6% | +1.386 m |
| Evening 21:00  | 0.3 lx | run3 | 98.7% | +1.408 m |
| Midnight 00:00 | 0.1 lx | run1 | 66.9% | +1.461 m |
| Midnight 00:00 | 0.1 lx | run2 | 74.3% | +1.216 m |
| Midnight 00:00 | 0.1 lx | run3 | 81.5% | +1.296 m |

**Reliability note:**
- **Detection rate is the primary reliable metric** from these recordings. It depends only on whether OpenCV finds the marker corners and is not affected by camera intrinsic accuracy.
- **Pose estimation error has known limitations** in this dataset: (1) intrinsics are scaled from the 1920×1080 sensor spec down to the screencast resolution (~440×250), introducing a systematic depth bias; (2) the ground-truth reference is fixed at the marker's initial world position (2.0, 0.0, 1.0 m) while the marker moves during the trajectory, so reported errors reflect both estimation bias and actual displacement. For a quantitative sim-to-real pose comparison, recordings should be taken directly from the `/camera/image_raw` ROS topic at full 1920×1080 resolution.
- Detection rate now scales with target lux as expected (19h ≈ 21h > midnight), with run-to-run spread attributed to the marker shrinking in frame as it moves away during the trajectory — this effect is most pronounced at midnight, where the lowest light margin makes the marker hardest to detect once distant.
- The 21h scenario's mean frame brightness (~15.2) is still close to midnight's (~15.7) despite higher target lux and a brighter spotlight setting in the world file; this is noted for future investigation but did not block consistent detection-rate ordering across scenarios.

---

## Running the Simulation (Docker)

Since this environment runs within a Docker container, follow these steps to initialize the simulation, the ROS2 bridge, and the experimental scripts across four terminal tabs.

### 0. Clone the Repository and Give Permissions

```bash
git clone https://github.com/raf-pimentel/paper-erc-night-task-simulation.git
```

On your **host machine**, allow Docker to access the display:

```bash
xhost +local:root
```

You should see: `non-network local connections being added to access control list`

### First-Time Setup: Generate the ArUco Marker PNG

The ArUco marker texture must be generated inside the container once, because the PNG depends on the OpenCV version installed there. Run this **inside the container** after starting it for the first time:

```bash
python3 << 'EOF'
import cv2, numpy as np, os

d = cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_50)
m = cv2.aruco.drawMarker(d, 0, 400)
c = np.ones((512, 512), np.uint8) * 255
c[56:456, 56:456] = m
out = '/ros2_ws/src/uwb_erc_sim/models/aruco_target/materials/textures/NewArUco.png'
cv2.imwrite(out, cv2.cvtColor(c, cv2.COLOR_GRAY2RGB))
print('Generated:', os.path.getsize(out), 'bytes')
EOF
```

Expected output: `Generated: 7456 bytes`

---

### Terminal 1: Launch Gazebo Harmonic

Start the container and enter it:

```bash
docker ps -a                        # find CONTAINER_ID
docker start <CONTAINER_ID>
docker exec -it <CONTAINER_ID> bash
```

Export required paths:

```bash
cd /ros2_ws
export GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins:${GZ_SIM_SYSTEM_PLUGIN_PATH}
export GZ_SIM_RESOURCE_PATH=/ros2_ws/src/uwb_erc_sim/models:${GZ_SIM_RESOURCE_PATH}
```

#### Launch a scenario:

| Scenario | Command |
|---|---|
| Twilight 19:00 — 3.0 lx | `gz sim /ros2_ws/src/uwb_erc_sim/worlds/night_task_19h.sdf` |
| Evening 21:00 — 0.3 lx | `gz sim /ros2_ws/src/uwb_erc_sim/worlds/night_task_21h.sdf` |
| Midnight 00:00 — 0.1 lx | `gz sim /ros2_ws/src/uwb_erc_sim/worlds/night_task_midnight.sdf` |

After the Gazebo window opens:
- Press the **Play** button to start the simulation.
- Click the **⋮ menu (top-right)** → select **Image Display** to see the live camera POV.

---

### Terminal 2: ROS–Gazebo Bridge

```bash
docker exec -it <CONTAINER_ID> bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
    /camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
    /model/aruco_target/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist
```

### Terminal 3: Movement Script

```bash
docker exec -it <CONTAINER_ID> bash
cd /ros2_ws
python3 src/uwb_erc_sim/scripts/fly_square.py
```

### Terminal 4: Vision Evaluation

```bash
docker exec -it <CONTAINER_ID> bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
python3 src/uwb_erc_sim/scripts/erc_vision_eval.py --scenario 19h
```

Use `--scenario` to tag results for each lighting condition:

```bash
python3 src/uwb_erc_sim/scripts/erc_vision_eval.py --scenario 19h
python3 src/uwb_erc_sim/scripts/erc_vision_eval.py --scenario 21h
python3 src/uwb_erc_sim/scripts/erc_vision_eval.py --scenario midnight
```

## Acknowledgments

This research was conducted as part of the visiting researcher program at the University of West Bohemia (UWB), collaborating with the UWB Robotics Team and Professor Tomas Broum.

## License

This software is provided "as is" under the MIT License. For full details, see the LICENSE file.
