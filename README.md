# Night-Task Navigation for Competition Rovers

Simulation environments and experimental data for the paper *"Enhancing Night-Task Navigation for Competition Rovers: A Comparative Study of Gazebo Simulations and Sensor Performance"*, developed for the Artificial Life and Robotics journal.

> **Full write-up:** [Google Docs](https://docs.google.com/document/d/1aZhX8T2QJ3hH65VPwwKCBMxovXjShG7vIbJGwNSziSs/edit?usp=sharing)

## Overview

The European Rover Challenge (ERC) Night Task requires reliable perception in near-zero lux environments. This project implements a Sim-to-Real pipeline to evaluate ArUco marker detection under extreme low-light conditions using ROS2 Jazzy and Gazebo Harmonic.

**Key contributions:**

- Gazebo-Harmonic sensor profile optimized for low-light ArUco detection
- Comparative analysis of pose estimation between virtual and physical IR sensors
- Scalable Sim-to-Real framework for autonomous vision testing

### Tech Stack

| Component | Technology |
|---|---|
| Framework | ROS2 Jazzy |
| Simulator | Gazebo Harmonic (gz-sim 8) |
| Camera | Intel RealSense D435i (simulated) |
| Vision | OpenCV ArUco (`DICT_7X7_50`) |
| Visualization | RViz2 |

---

## Simulation Results

Three runs per lighting condition; values are mean ± std dev.

| Scenario | Target Lux | Detection Rate | Mean Depth Error |
|---|---|---|---|
| Twilight 19:00 | 3.0 lx | 91.7% ± 7.4% | +1.334 ± 0.050 m |
| Evening 21:00 | 0.3 lx | 93.6% ± 3.8% | +1.388 ± 0.015 m |
| Midnight 00:00 | 0.1 lx | 84.3% ± 8.3% | +1.262 ± 0.083 m |

<details>
<summary>Per-run breakdown</summary>

| Scenario | Target Lux | Run | Detection Rate | Mean Depth Error |
|---|---|---|---|---|
| Twilight 19:00 | 3.0 lx | run1 | 81.2% | +1.264 m |
| Twilight 19:00 | 3.0 lx | run2 | 96.2% | +1.373 m |
| Twilight 19:00 | 3.0 lx | run3 | 97.6% | +1.365 m |
| Evening 21:00 | 0.3 lx | run1 | 89.4% | +1.371 m |
| Evening 21:00 | 0.3 lx | run2 | 92.6% | +1.386 m |
| Evening 21:00 | 0.3 lx | run3 | 98.7% | +1.408 m |
| Midnight 00:00 | 0.1 lx | run1 | 77.6% | +1.191 m |
| Midnight 00:00 | 0.1 lx | run2 | 95.9% | +1.378 m |
| Midnight 00:00 | 0.1 lx | run3 | 79.3% | +1.218 m |

</details>

> **Note:** Detection rate is the primary reliable metric. Depth error has a known systematic bias because camera intrinsics were scaled from 1920×1080 to the screencast resolution (~440×250). For precise pose comparison, record directly from the `/camera/image_raw` ROS topic at full resolution.

---

## Running the Simulation

The simulation runs inside a Docker container. You will need **four terminal tabs** — one for Gazebo, one for the ROS bridge, one for the movement script, and one for vision evaluation.

### 0. Prerequisites

```bash
# Clone the repository
git clone https://github.com/raf-pimentel/paper-erc-night-task-simulation.git

# Allow Docker to access the display (host machine)
xhost +local:root
```

### 1. First-Time Setup: Generate the ArUco Marker

Run this **once** inside the container after first start — the PNG depends on the installed OpenCV version:

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

### 2. Terminal 1 — Gazebo

```bash
docker ps -a                        # find CONTAINER_ID
docker start <CONTAINER_ID>
docker exec -it <CONTAINER_ID> bash
```

```bash
cd /ros2_ws
export GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins:${GZ_SIM_SYSTEM_PLUGIN_PATH}
export GZ_SIM_RESOURCE_PATH=/ros2_ws/src/uwb_erc_sim/models:${GZ_SIM_RESOURCE_PATH}
```

Launch a scenario:

| Scenario | Command |
|---|---|
| Twilight 19:00 (3.0 lx) | `gz sim /ros2_ws/src/uwb_erc_sim/worlds/night_task_19h.sdf` |
| Evening 21:00 (0.3 lx) | `gz sim /ros2_ws/src/uwb_erc_sim/worlds/night_task_21h.sdf` |
| Midnight 00:00 (0.1 lx) | `gz sim /ros2_ws/src/uwb_erc_sim/worlds/night_task_midnight.sdf` |

After Gazebo opens: press **Play**, then click **⋮ menu (top-right)** → **Image Display** to see the camera feed.

### 3. Terminal 2 — ROS-Gazebo Bridge

```bash
docker exec -it <CONTAINER_ID> bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
    /camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
    /model/aruco_target/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist
```

### 4. Terminal 3 — Movement Script

```bash
docker exec -it <CONTAINER_ID> bash
cd /ros2_ws
python3 src/uwb_erc_sim/scripts/fly_square.py
```

### 5. Terminal 4 — Vision Evaluation

```bash
docker exec -it <CONTAINER_ID> bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
python3 src/uwb_erc_sim/scripts/erc_vision_eval.py --scenario <SCENARIO>
```

Replace `<SCENARIO>` with `19h`, `21h`, or `midnight`.

---

## Repository Structure

```
├── process_recordings.py              # Post-processes recordings → results
├── recordings/                        # Screen recordings (3 runs × 3 scenarios)
├── results/
│   ├── raw/                           # Per-frame CSV data
│   └── summary/                       # Aggregated stats per run + combined table
└── src/uwb_erc_sim/
    ├── launch/simulation.launch.py
    ├── models/                        # ArUco marker model + textures
    ├── scripts/
    │   ├── aruco_detector.py
    │   ├── erc_vision_eval.py         # Main evaluation script
    │   ├── experiment_automator.py
    │   ├── fly_square.py              # Marker movement trajectory
    │   ├── intensity_sweeper.py
    │   ├── light_calibration.py
    │   └── vision_node.py
    ├── urdf/d435i.urdf                # Camera URDF
    └── worlds/
        ├── night_task_19h.sdf         # Twilight — 3.0 lx
        ├── night_task_21h.sdf         # Evening — 0.3 lx
        ├── night_task_midnight.sdf    # Midnight — 0.1 lx
        └── standard.sdf
```

## Acknowledgments

Research conducted as part of the visiting researcher program at the University of West Bohemia (UWB), in collaboration with the UWB Robotics Team and Professor Tomas Broum.

## License

MIT License — see [LICENSE](LICENSE).
