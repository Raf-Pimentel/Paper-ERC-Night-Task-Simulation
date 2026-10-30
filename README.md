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
| Vision | OpenCV ArUco (`DICT_7X7_50` ID 0 in the original sim runs / `DICT_ARUCO_ORIGINAL` ID 297 in real and in the same-marker sim runs) |
| Visualization | RViz2 |

---

## Results

Three conditions were compared per lighting level: **simulation with a different marker** (`DICT_7X7_50` ID 0, five runs), **simulation with the same marker as the real experiment** (`DICT_ARUCO_ORIGINAL` ID 297, five runs), and **real-world** (`DICT_ARUCO_ORIGINAL` ID 297, three runs). Tables report mean ± std dev across runs.

### Simulation (Gazebo)

| Scenario | Target Lux | Detection Rate | Mean Depth Error |
|---|---|---|---|
| Twilight 19:00 | 3.0 lx | 90.1% ± 6.4% | +1.430 ± 0.200 m |
| Evening 21:00  | 0.3 lx | 84.0% ± 15.3% | +1.370 ± 0.050 m |
| Midnight 00:00 | 0.1 lx | 80.8% ± 8.3% | +1.270 ± 0.070 m |

### Simulation, same marker as real-world (Gazebo, ID 297)

Same worlds, lighting, and trajectory as above (`worlds/night_task_*_id297.sdf`); only the marker texture changes to `DICT_ARUCO_ORIGINAL` ID 297, the marker used in the UWB recordings.

| Scenario | Target Lux | Detection Rate | Mean Depth Error |
|---|---|---|---|
| Twilight 19:00 | 3.0 lx | 95.3% ± 3.3% | +1.420 ± 0.040 m |
| Evening 21:00  | 0.3 lx | 99.7% ± 0.3% | +1.450 ± 0.030 m |
| Midnight 00:00 | 0.1 lx | 93.6% ± 5.0% | +1.430 ± 0.050 m |

### Real-World (UWB Workshop)

Physical experiments used a D435i camera and synchronized spotlight, filmed at 1280×720 @ 15 fps. The physical ArUco marker is from `DICT_ARUCO_ORIGINAL` (ID 297).

| Scenario | Target Lux | Detection Rate | Mean Depth Error |
|---|---|---|---|
| Twilight 19:00 | 3.0 lx | 92.3% ± 0.4% | +0.520 ± 0.020 m |
| Evening 21:00  | 0.3 lx | 83.2% ± 4.2% | +0.420 ± 0.010 m |
| Midnight 00:00 | 0.1 lx | 37.8% ± 3.8% | +0.170 ± 0.030 m |

### Three-Way Comparison

| Scenario | Target Lux | Source | Detection Rate | Mean Depth Error |
|---|---|---|---|---|
| Twilight 19:00 | 3.0 lx | Simulation (7x7 ID 0) | 90.1% ± 6.4% | +1.430 ± 0.200 m |
| Twilight 19:00 | 3.0 lx | Simulation (same marker, ID 297) | 95.3% ± 3.3% | +1.420 ± 0.040 m |
| Twilight 19:00 | 3.0 lx | Real-world (ID 297) | 92.3% ± 0.4% | +0.520 ± 0.020 m |
| Evening 21:00  | 0.3 lx | Simulation (7x7 ID 0) | 84.0% ± 15.3% | +1.370 ± 0.050 m |
| Evening 21:00  | 0.3 lx | Simulation (same marker, ID 297) | 99.7% ± 0.3% | +1.450 ± 0.030 m |
| Evening 21:00  | 0.3 lx | Real-world (ID 297) | 83.2% ± 4.2% | +0.420 ± 0.010 m |
| Midnight 00:00 | 0.1 lx | Simulation (7x7 ID 0) | 80.8% ± 8.3% | +1.270 ± 0.070 m |
| Midnight 00:00 | 0.1 lx | Simulation (same marker, ID 297) | 93.6% ± 5.0% | +1.430 ± 0.050 m |
| Midnight 00:00 | 0.1 lx | Real-world (ID 297) | 37.8% ± 3.8% | +0.170 ± 0.030 m |

Detection-rate differences in percentage points:

| Scenario | Marker effect in sim (ID 297 − 7x7) | Real − sim (7x7), original comparison | Real − sim (same marker) |
|---|---|---|---|
| Twilight 19:00 | +5.2 | +2.2 | −3.0 |
| Evening 21:00  | +15.7 | −0.8 | −16.4 |
| Midnight 00:00 | +12.8 | −43.0 | −55.8 |

**Key observations:**

- **The marker is not the cause of the midnight gap.** With the identical marker, simulation still detects it in 93.6% of frames at 0.1 lx versus 37.8% in the real recordings, so the gap widens from 43.0 to 55.8 points instead of closing. This points back to the missing sensor-noise model (no read noise, shot noise, or IR-projector speckle) rather than to the marker choice.
- **The ID 297 marker is easier to detect in simulation at every light level** (+5.2, +15.7, +12.8 points). It is a 5×5-bit marker, so each cell is larger at the same 0.20 m physical size, which helps in the low-resolution (~450×255) screencasts. The original 7x7 results therefore understated simulated detectability.
- **The close match at 0.3 lx was partly coincidental.** Simulation (7x7) and real agreed within 0.8 points, but with the same marker simulation reaches 99.7% against 83.2% real, a 16.4-point gap. The original 7x7 mean was also pulled down by the `sim_21h_run4` outlier (55.0%). Treat fidelity at 0.3 lx as less established than the first comparison suggested.
- **Twilight (3.0 lx)** is the only level where all three conditions agree within about 5 points.
- **Detection is not monotonic in light level for the ID 297 simulation:** 0.3 lx (99.7%) is higher than 3.0 lx (95.3%), driven by three 19h runs near 92.5%. This dataset does not explain why.
- **Depth error:** both simulation conditions sit around +1.3 to +1.5 m while the real recordings are 0.2 to 0.5 m. This is the screencast-intrinsics artifact described below, so it does not distinguish the two simulation conditions.
- **Run consistency:** real-world variance is tighter than simulation at 3.0 lx (±0.4%), reflecting controlled physical conditions vs. GPU-dependent Gazebo frame rates. With 5 and 3 runs per condition and large spreads in places, differences of a few points are not conclusive.

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
| Evening 21:00  | 0.3 lx | run4 | 55.0% | +1.272 m |
| Evening 21:00  | 0.3 lx | run5 | 83.0% | +1.410 m |
| Midnight 00:00 | 0.1 lx | run1 | 78.0% | +1.196 m |
| Midnight 00:00 | 0.1 lx | run2 | 97.0% | +1.388 m |
| Midnight 00:00 | 0.1 lx | run3 | 79.5% | +1.220 m |
| Midnight 00:00 | 0.1 lx | run4 | 74.1% | +1.283 m |
| Midnight 00:00 | 0.1 lx | run5 | 75.6% | +1.260 m |

**Simulation, same marker (ID 297)**

| Scenario | Target Lux | Run | Detection Rate | Mean Depth Error |
|---|---|---|---|---|
| Twilight 19:00 | 3.0 lx | run1 | 92.7% | +1.403 m |
| Twilight 19:00 | 3.0 lx | run2 | 99.9% | +1.450 m |
| Twilight 19:00 | 3.0 lx | run3 | 98.8% | +1.470 m |
| Twilight 19:00 | 3.0 lx | run4 | 92.6% | +1.390 m |
| Twilight 19:00 | 3.0 lx | run5 | 92.5% | +1.381 m |
| Evening 21:00  | 0.3 lx | run1 | 99.9% | +1.444 m |
| Evening 21:00  | 0.3 lx | run2 | 99.4% | +1.425 m |
| Evening 21:00  | 0.3 lx | run3 | 99.4% | +1.492 m |
| Evening 21:00  | 0.3 lx | run4 | 100.0% | +1.412 m |
| Evening 21:00  | 0.3 lx | run5 | 99.7% | +1.475 m |
| Midnight 00:00 | 0.1 lx | run1 | 84.2% | +1.348 m |
| Midnight 00:00 | 0.1 lx | run2 | 93.9% | +1.498 m |
| Midnight 00:00 | 0.1 lx | run3 | 95.7% | +1.430 m |
| Midnight 00:00 | 0.1 lx | run4 | 95.1% | +1.418 m |
| Midnight 00:00 | 0.1 lx | run5 | 99.3% | +1.454 m |

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

> **Note on depth error:** Detection rate is the primary reliable metric. Simulation depth error carries a systematic offset from the screencast pipeline (camera intrinsics scaled to a screen-recording crop that varies slightly run to run) — treat it as indicative, not precise. Recording directly from the `/camera/image_raw` ROS topic would remove this offset.

---

## Running the Simulation

The simulation runs inside a Docker container. You will need **four terminal tabs** — one for Gazebo, one for the ROS bridge, one for the movement script, and one for vision evaluation.

### 0. Prerequisites

```bash
# Clone the repository
git clone https://github.com/Raf-Pimentel/Paper-ERC-Night-Task-Simulation.git
cd Paper-ERC-Night-Task-Simulation

# Build the image (pinned base image + requirements.txt)
docker build -t erc-night-task .

# Allow Docker to access the display (host machine)
xhost +local:root

# Start the container, mounting the simulation source
docker run -dit --name erc_sim_container \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$(pwd)/src":/ros2_ws/src \
    erc-night-task bash
```

**Laptops with an NVIDIA GPU (recommended):** without GPU access Gazebo falls back to software rendering and the GUI may freeze ("not responding") or never open on Wayland. Requires the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html). Use this `docker run` instead:

```bash
docker run -dit --name erc_sim_container \
    --runtime nvidia --gpus all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e __NV_PRIME_RENDER_OFFLOAD=1 \
    -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$(pwd)/src":/ros2_ws/src \
    erc-night-task bash
```

Passing `-r` to `gz sim` (e.g. `gz sim -r <world>.sdf`) starts the simulation immediately, so you do not need to press Play.

### 1. First-Time Setup: Generate the ArUco Marker

Run this **once** inside the container after first start — the PNG depends on the installed OpenCV version:

```bash
python3 << 'EOF'
import cv2, numpy as np, os

d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_50)
m = cv2.aruco.generateImageMarker(d, 0, 400)
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
docker exec -it erc_sim_container bash
```

```bash
cd /ros2_ws
export GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins:${GZ_SIM_SYSTEM_PLUGIN_PATH}
export GZ_SIM_RESOURCE_PATH=/ros2_ws/src/uwb_erc_sim/models:${GZ_SIM_RESOURCE_PATH}
```

Launch a scenario (the `_id297` worlds are identical except that the marker is `DICT_ARUCO_ORIGINAL` ID 297, as in the real recordings):

| Scenario | Command |
|---|---|
| Twilight 19:00 (3.0 lx) | `gz sim /ros2_ws/src/uwb_erc_sim/worlds/night_task_19h.sdf` |
| Evening 21:00 (0.3 lx) | `gz sim /ros2_ws/src/uwb_erc_sim/worlds/night_task_21h.sdf` |
| Midnight 00:00 (0.1 lx) | `gz sim /ros2_ws/src/uwb_erc_sim/worlds/night_task_midnight.sdf` |
| Twilight 19:00, ID 297 marker | `gz sim /ros2_ws/src/uwb_erc_sim/worlds/night_task_19h_id297.sdf` |
| Evening 21:00, ID 297 marker | `gz sim /ros2_ws/src/uwb_erc_sim/worlds/night_task_21h_id297.sdf` |
| Midnight 00:00, ID 297 marker | `gz sim /ros2_ws/src/uwb_erc_sim/worlds/night_task_midnight_id297.sdf` |

After Gazebo opens: press **Play**, then click **⋮ menu (top-right)** → **Image Display** to see the camera feed.

### 3. Terminal 2 — ROS-Gazebo Bridge

```bash
docker exec -it erc_sim_container bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
    /camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
    /model/aruco_target/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist
```

### 4. Terminal 3 — Movement Script

```bash
docker exec -it erc_sim_container bash
cd /ros2_ws
python3 src/uwb_erc_sim/scripts/fly_square.py
```

### 5. Terminal 4 — Vision Evaluation

```bash
docker exec -it erc_sim_container bash
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
python3 src/uwb_erc_sim/scripts/erc_vision_eval.py --scenario <SCENARIO>
```

Replace `<SCENARIO>` with `19h`, `21h`, or `midnight`.

---

## Processing Recordings

Install the pinned dependencies, then reprocess recordings or add new ones:

```bash
pip install -r requirements.txt

# Process a single video
python3 process_recordings.py --video recordings/real_19h_run1.mp4 --scenario 19h

# Process all videos in recordings/ (sim_*, sim_id297_* and real_*)
python3 process_recordings.py --all

# Rebuild the sim-vs-real comparison from existing summaries
python3 process_recordings.py --compare
```

Videos must be named `sim_<scenario>_run<N>.mp4` (simulation, `DICT_7X7_50` ID 0), `sim_id297_<scenario>_run<N>.mp4` (simulation with the real-world marker, `DICT_ARUCO_ORIGINAL` ID 297), or `real_<scenario>_run<N>.mp4`, where `<scenario>` is `19h`, `21h`, or `midnight`.

---

## Repository Structure

```
├── Dockerfile                          # Pinned simulation environment (see requirements.txt)
├── requirements.txt                    # Pinned Python deps for process_recordings.py
├── process_recordings.py               # Post-processes recordings → results/
├── TECHNICAL_DETAILS.md                # Full reproducibility reference
├── recordings/
│   ├── sim_19h_run1.mp4 .. run5.mp4
│   ├── sim_21h_run1.mp4 .. run5.mp4
│   ├── sim_midnight_run1.mp4 .. run5.mp4
│   ├── sim_id297_19h_run1.mp4 .. run5.mp4      # same marker as real (also 21h, midnight)
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
        ├── night_task_*_id297.sdf     # Same three worlds with the ID 297 marker
        └── standard.sdf
```

---

## Acknowledgments

Research conducted as part of the visiting researcher program at the University of West Bohemia (UWB), in collaboration with the UWB Robotics Team and Professor Tomas Broum.

## License

MIT License — see [LICENSE](LICENSE).
