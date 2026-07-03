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
├── process_recordings.py          ← processes recordings/*.mp4 into results/
├── recordings
│   ├── sim_19h_run1.mp4 .. run3.mp4
│   ├── sim_21h_run1.mp4 .. run3.mp4
│   ├── sim_midnight_run1.mp4 .. run3.mp4
│   ├── real_19h_run1.mp4 .. run3.mp4
│   ├── real_21h_run1.mp4 .. run3.mp4
│   └── real_midnight_run1.mp4 .. run3.mp4
├── results
│   ├── raw
│   │   ├── sim_<scenario>_run<N>_raw.csv        ← one row per frame (simulation)
│   │   └── real_<scenario>_run<N>_raw.csv       ← one row per frame (real-world)
│   └── summary
│       ├── sim_<scenario>_run<N>_summary.csv    ← aggregated stats per run
│       ├── real_<scenario>_run<N>_summary.csv
│       ├── all_scenarios_summary.csv            ← all runs combined
│       └── sim_vs_real_comparison.csv           ← mean ± std per scenario × source
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

Results were obtained by processing screen recordings of each scenario with `process_recordings.py` using `DICT_7X7_50` (matches the simulation marker). Three runs were performed per lighting condition; the table reports mean ± standard deviation across runs.

| Scenario | Target Lux | Detection Rate | Mean Depth Error |
|---|---|---|---|
| Twilight 19:00 | 3.0 lx | 91.7% ± 7.4% | +1.330 ± 0.050 m |
| Evening 21:00  | 0.3 lx | 93.6% ± 3.8% | +1.390 ± 0.020 m |
| Midnight 00:00 | 0.1 lx | 84.3% ± 8.3% | +1.260 ± 0.080 m |

<details>
<summary>Per-run breakdown</summary>

| Scenario | Target Lux | Run | Detection Rate | Mean Depth Error |
|---|---|---|---|---|
| Twilight 19:00 | 3.0 lx | run1 | 81.2% | +1.264 m |
| Twilight 19:00 | 3.0 lx | run2 | 96.2% | +1.373 m |
| Twilight 19:00 | 3.0 lx | run3 | 97.6% | +1.365 m |
| Evening 21:00  | 0.3 lx | run1 | 89.4% | +1.371 m |
| Evening 21:00  | 0.3 lx | run2 | 92.6% | +1.386 m |
| Evening 21:00  | 0.3 lx | run3 | 98.7% | +1.408 m |
| Midnight 00:00 | 0.1 lx | run1 | 77.6% | +1.191 m |
| Midnight 00:00 | 0.1 lx | run2 | 95.9% | +1.378 m |
| Midnight 00:00 | 0.1 lx | run3 | 79.3% | +1.218 m |

</details>

**Reliability note:**
- **Detection rate is the primary reliable metric** from these recordings. It depends only on whether OpenCV finds the marker corners and is not affected by camera intrinsic accuracy.
- **Pose estimation error has known limitations** in this dataset: (1) intrinsics are scaled from the 1920×1080 sensor spec down to the screencast resolution (~440×250), introducing a systematic depth bias; (2) the ground-truth reference is fixed at the marker's initial world position (2.0, 0.0, 1.0 m) while the marker moves during the trajectory, so reported errors reflect both estimation bias and actual displacement. For a quantitative sim-to-real pose comparison, recordings should be taken directly from the `/camera/image_raw` ROS topic at full 1920×1080 resolution.
- Detection rate scales with target lux as expected (19h ≈ 21h > midnight), with run-to-run spread attributed to the marker shrinking in frame as it moves away during the trajectory — this effect is most pronounced at midnight, where the lowest light margin makes the marker hardest to detect once distant.
- **Resolved:** the 21h scenario's mean frame brightness (~15.2) was close to midnight's (~15.7) despite higher target lux. Root cause: `night_task_midnight.sdf`'s `<background>` (sky color) was set to 0.01 — brighter than 21h's 0.005 — breaking the 19h > 21h > midnight darkening progression used by every other light parameter in these worlds. Fixed to 0.0016 to restore the progression. Midnight was re-recorded after the fix, and `mean_brightness` in the screencast pipeline now confirms it: 19h ≈ 44.2, 21h ≈ 15.16, midnight ≈ 5.03 — correctly monotonic.

---

### Real-World Results (UWB Workshop)

Physical experiments were conducted at the University of West Bohemia workshop using an Intel RealSense D435i camera and a synchronized spotlight. Videos were recorded at 1280×720 @ 15 fps and processed with `process_recordings.py` using `DICT_ARUCO_ORIGINAL` (the dictionary of the physical marker, ID 297). Three runs were performed per lighting condition.

| Scenario | Target Lux | Detection Rate | Mean Depth Error |
|---|---|---|---|
| Twilight 19:00 | 3.0 lx | 91.1% ± 0.6% | +0.550 ± 0.030 m |
| Evening 21:00  | 0.3 lx | 83.2% ± 4.2% | +0.470 ± 0.030 m |
| Midnight 00:00 | 0.1 lx | 37.8% ± 3.8% | +0.170 ± 0.030 m |

<details>
<summary>Per-run breakdown</summary>

| Scenario | Target Lux | Run | Detection Rate | Mean Depth Error |
|---|---|---|---|---|
| Twilight 19:00 | 3.0 lx | run1 | 91.3% | +0.590 m |
| Twilight 19:00 | 3.0 lx | run2 | 90.4% | +0.516 m |
| Twilight 19:00 | 3.0 lx | run3 | 91.8% | +0.531 m |
| Evening 21:00  | 0.3 lx | run1 | 89.1% | +0.429 m |
| Evening 21:00  | 0.3 lx | run2 | 80.6% | +0.488 m |
| Evening 21:00  | 0.3 lx | run3 | 80.0% | +0.494 m |
| Midnight 00:00 | 0.1 lx | run1 | 32.6% | +0.143 m |
| Midnight 00:00 | 0.1 lx | run2 | 39.4% | +0.210 m |
| Midnight 00:00 | 0.1 lx | run3 | 41.4% | +0.162 m |

</details>

---

### Sim-to-Real Comparison

| Scenario | Target Lux | Source | Detection Rate | Mean Depth Error |
|---|---|---|---|---|
| Twilight 19:00 | 3.0 lx | Simulation  | 91.7% ± 7.4% | +1.330 ± 0.050 m |
| Twilight 19:00 | 3.0 lx | Real-world  | 91.1% ± 0.6% | +0.550 ± 0.030 m |
| Evening 21:00  | 0.3 lx | Simulation  | 93.6% ± 3.8% | +1.390 ± 0.020 m |
| Evening 21:00  | 0.3 lx | Real-world  | 83.2% ± 4.2% | +0.470 ± 0.030 m |
| Midnight 00:00 | 0.1 lx | Simulation  | 84.3% ± 8.3% | +1.260 ± 0.080 m |
| Midnight 00:00 | 0.1 lx | Real-world  | 37.8% ± 3.8% | +0.170 ± 0.030 m |

**Key observations:**

- **Twilight (3.0 lx):** Detection rates match almost exactly (~91% both), demonstrating good sim-to-real fidelity at the highest light level tested.
- **Evening (0.3 lx):** A ~10 pp gap emerges (real 83% vs sim 94%). The real sensor struggles more as ambient light falls below 1 lx, a gap the simulation does not fully capture.
- **Midnight (0.1 lx):** The largest divergence — real-world detection collapses to 38% while simulation maintains 84%. The simulator overestimates sensor performance at extreme low-light; the physical IR spotlight illuminates a narrower effective cone and the real sensor has higher noise than the Gazebo model assumes.
- **Depth error:** Real-world errors are consistently ~0.8 m smaller than simulation. The simulation depth bias is an artifact of the screencast pipeline: intrinsics for a 1920×1080 sensor were scaled to the ~440×250 screen recording, introducing a systematic offset. Real-world videos at 1280×720 do not have this bias.
- **Run-to-run consistency:** Real-world variance is notably tighter (e.g., ±0.6% at 19h) than simulation (±7.4%), reflecting more controlled physical conditions compared to the GPU-dependent Gazebo frame rate.

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
