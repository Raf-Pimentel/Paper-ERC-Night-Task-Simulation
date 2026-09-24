# Night-Task Navigation for Competition Rovers

Simulation environments, real-world experiment data, and processing pipeline for the paper *"Enhancing Night-Task Navigation for Competition Rovers: A Comparative Study of Gazebo Simulations and Sensor Performance"*, developed for the Artificial Life and Robotics journal.

> **Full write-up:** [Google Docs](https://docs.google.com/document/d/1aZhX8T2QJ3hH65VPwwKCBMxovXjShG7vIbJGwNSziSs/edit?usp=sharing)

## Overview

The European Rover Challenge (ERC) Night Task requires reliable perception in near-zero lux environments. This project implements a Sim-to-Real pipeline to evaluate ArUco marker detection under extreme low-light conditions using ROS2 Jazzy and Gazebo Harmonic. Virtual experiments were conducted in simulation, then validated against physical experiments at the University of West Bohemia (UWB) workshop.

**Key contributions:**

- Gazebo-Harmonic sensor profile optimized for low-light ArUco detection
- Comparative analysis of pose estimation between virtual and physical IR sensors
- Scalable Sim-to-Real framework for autonomous vision testing in space exploration contexts

### Tech Stack

| Component | Technology |
|---|---|
| Framework | ROS2 Jazzy |
| Simulator | Gazebo Harmonic (gz-sim 8) |
| Camera | Intel RealSense D435i |
| Vision | OpenCV ArUco (`DICT_7X7_50` in sim / `DICT_ARUCO_ORIGINAL` in real) |
| Visualization | RViz2 |

---

## Results

Five simulation runs and three real-world runs were performed per lighting condition. Tables report mean ± std dev across runs.

### Simulation (Gazebo)

| Scenario | Target Lux | Detection Rate | Mean Depth Error |
|---|---|---|---|
| Twilight 19:00 | 3.0 lx | 90.1% ± 6.4% | +1.430 ± 0.200 m |
| Evening 21:00  | 0.3 lx | 84.0% ± 15.3% | +1.370 ± 0.050 m |
| Midnight 00:00 | 0.1 lx | 80.8% ± 8.3% | +1.270 ± 0.070 m |

### Real-World (UWB Workshop)

Physical experiments used a D435i camera and synchronized spotlight, filmed at 1280×720 @ 15 fps. The physical ArUco marker is from `DICT_ARUCO_ORIGINAL` (ID 297).

| Scenario | Target Lux | Detection Rate | Mean Depth Error |
|---|---|---|---|
| Twilight 19:00 | 3.0 lx | 92.3% ± 0.4% | +0.520 ± 0.020 m |
| Evening 21:00  | 0.3 lx | 83.2% ± 4.2% | +0.420 ± 0.010 m |
| Midnight 00:00 | 0.1 lx | 37.8% ± 3.8% | +0.170 ± 0.030 m |

### Sim-to-Real Comparison

| Scenario | Target Lux | Source | Detection Rate | Mean Depth Error |
|---|---|---|---|---|
| Twilight 19:00 | 3.0 lx | Simulation | 90.1% ± 6.4% | +1.430 ± 0.200 m |
| Twilight 19:00 | 3.0 lx | Real-world | 92.3% ± 0.4% | +0.520 ± 0.020 m |
| Evening 21:00  | 0.3 lx | Simulation | 84.0% ± 15.3% | +1.370 ± 0.050 m |
| Evening 21:00  | 0.3 lx | Real-world | 83.2% ± 4.2% | +0.420 ± 0.010 m |
| Midnight 00:00 | 0.1 lx | Simulation | 80.8% ± 8.3% | +1.270 ± 0.070 m |
| Midnight 00:00 | 0.1 lx | Real-world | 37.8% ± 3.8% | +0.170 ± 0.030 m |

**Key observations:**

- **Twilight (3.0 lx):** Sim and real still match closely (~90–92%), confirming good fidelity at the highest light level tested.
- **Evening (0.3 lx):** Mean detection rate now tracks real-world closely (84.0% sim vs 83.2% real), but simulation variance jumped to ±15.3% (from ±3.8% with 3 runs) — driven by `sim_21h_run4`, which detected at only 55.0%, well below the other four runs (83–99%). That run's frames are also ~21% dimmer on average than the other Evening runs despite an identical world file, pointing to a rendering-pipeline difference (this run was recorded under CPU/software rendering as a workaround for a host GPU driver issue) rather than genuine scenario variance.
- **Midnight (0.1 lx):** Largest divergence — real-world detection collapses to 38% while simulation holds at ~81%. The Gazebo model overestimates sensor performance at extreme low-light; the physical spotlight has a narrower effective cone and the real sensor has higher noise than the model assumes.
- **Depth error:** Real-world errors are ~0.9 m smaller than simulation. The simulation bias is an artifact of the screencast pipeline: intrinsics for a 1920×1080 sensor were scaled to the ~440×250 screen recording. Real-world videos at 1280×720 do not carry this offset.
- **Run consistency:** Real-world variance is still tighter (e.g., ±0.4% at 19h) than simulation, reflecting more controlled physical conditions vs. GPU-dependent Gazebo frame rates — and, for Evening specifically, a mixed-rendering-backend artifact in the newest runs (see above).

<details>
<summary>Per-run breakdown</summary>

**Simulation**

| Scenario | Target Lux | Run | Detection Rate | Mean Depth Error |
|---|---|---|---|---|
| Twilight 19:00 | 3.0 lx | run1 | 81.2% | +1.264 m |
| Twilight 19:00 | 3.0 lx | run2 | 96.4% | +1.373 m |
| Twilight 19:00 | 3.0 lx | run3 | 97.6% | +1.365 m |
| Twilight 19:00 | 3.0 lx | run4 | 91.1% | +1.319 m |
| Twilight 19:00 | 3.0 lx | run5 | 84.4% | +1.824 m |
| Evening 21:00  | 0.3 lx | run1 | 90.2% | +1.379 m |
| Evening 21:00  | 0.3 lx | run2 | 93.1% | +1.389 m |
| Evening 21:00  | 0.3 lx | run3 | 98.7% | +1.408 m |
| Evening 21:00  | 0.3 lx | run4 | 55.0% ⚠️ | +1.272 m |
| Evening 21:00  | 0.3 lx | run5 | 83.0% | +1.410 m |
| Midnight 00:00 | 0.1 lx | run1 | 78.0% | +1.196 m |
| Midnight 00:00 | 0.1 lx | run2 | 97.0% | +1.388 m |
| Midnight 00:00 | 0.1 lx | run3 | 79.5% | +1.220 m |
| Midnight 00:00 | 0.1 lx | run4 | 74.1% | +1.283 m |
| Midnight 00:00 | 0.1 lx | run5 | 75.6% | +1.260 m |

⚠️ `sim_21h_run4` recorded ~21% dimmer than the other Evening runs (mean frame brightness 11.9 vs 15.2) despite the same world file — the run used CPU/software Gazebo rendering (a temporary workaround for a host GPU driver mismatch) instead of the GPU-accelerated path used for every other run. Treat this run's detection rate as a rendering-backend artifact, not a scenario result.

**Real-World**

| Scenario | Target Lux | Run | Detection Rate | Mean Depth Error |
|---|---|---|---|---|
| Twilight 19:00 | 3.0 lx | run1 | 92.8% | +0.541 m |
| Twilight 19:00 | 3.0 lx | run2 | 91.8% | +0.504 m |
| Twilight 19:00 | 3.0 lx | run3 | 92.4% | +0.526 m |
| Evening 21:00  | 0.3 lx | run1 | 89.1% | +0.403 m |
| Evening 21:00  | 0.3 lx | run2 | 80.6% | +0.426 m |
| Evening 21:00  | 0.3 lx | run3 | 80.0% | +0.432 m |
| Midnight 00:00 | 0.1 lx | run1 | 32.6% | +0.143 m |
| Midnight 00:00 | 0.1 lx | run2 | 39.4% | +0.210 m |
| Midnight 00:00 | 0.1 lx | run3 | 41.4% | +0.162 m |

</details>

> **Note on depth error:** Detection rate is the primary reliable metric. Depth error is included for reference but carries a systematic offset in the simulation (intrinsics scaled from 1920×1080 down to ~440×250 screencast resolution). For a precise pose comparison, record directly from the `/camera/image_raw` ROS topic at full resolution.

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

## Processing Recordings

To reprocess recordings or add new ones:

```bash
# Process a single video
python3 process_recordings.py --video recordings/real_19h_run1.mp4 --scenario 19h

# Process all videos in recordings/ (sim_* and real_*)
python3 process_recordings.py --all

# Rebuild the sim-vs-real comparison from existing summaries
python3 process_recordings.py --compare
```

Videos must be named `sim_<scenario>_run<N>.mp4` or `real_<scenario>_run<N>.mp4` where `<scenario>` is `19h`, `21h`, or `midnight`.

---

## Repository Structure

```
├── process_recordings.py              # Post-processes recordings → results/
├── recordings/
│   ├── sim_19h_run1.mp4 .. run3.mp4
│   ├── sim_21h_run1.mp4 .. run3.mp4
│   ├── sim_midnight_run1.mp4 .. run3.mp4
│   ├── real_19h_run1.mp4 .. run3.mp4
│   ├── real_21h_run1.mp4 .. run3.mp4
│   └── real_midnight_run1.mp4 .. run3.mp4
├── results/
│   ├── raw/                           # Per-frame CSVs (one row per frame)
│   └── summary/
│       ├── *_summary.csv              # Aggregated stats per run
│       ├── all_scenarios_summary.csv  # All runs combined
│       └── sim_vs_real_comparison.csv # Mean ± std per scenario × source
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
    ├── urdf/d435i.urdf
    └── worlds/
        ├── night_task_19h.sdf         # Twilight — 3.0 lx
        ├── night_task_21h.sdf         # Evening — 0.3 lx
        ├── night_task_midnight.sdf    # Midnight — 0.1 lx
        └── standard.sdf
```

---

## Acknowledgments

Research conducted as part of the visiting researcher program at the University of West Bohemia (UWB), in collaboration with the UWB Robotics Team and Professor Tomas Broum.

## License

MIT License — see [LICENSE](LICENSE).
