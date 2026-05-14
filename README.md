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

---

## Why the ArUco Marker Initially Did Not Appear — Root Cause Analysis

Getting the ArUco marker to display correctly required solving three independent problems that stacked on top of each other. Each one silently masked the next.

### Problem 1 — `Unable to find uri[model://aruco_target]`

**What happened:** The world SDF files referenced the ArUco model with `<uri>model://aruco_target</uri>`. This URI scheme is resolved by Gazebo at runtime using the `GZ_SIM_RESOURCE_PATH` environment variable. Because that variable was never set when calling `gz sim` directly (outside the launch file), Gazebo simply could not locate the model directory.

**Fix:** Changed the `<uri>` in all world SDF files from `model://aruco_target` to the absolute path `/ros2_ws/src/uwb_erc_sim/models/aruco_target`, which works unconditionally. Also added `GZ_SIM_RESOURCE_PATH` to `simulation.launch.py` for launch-based runs.

---

### Problem 2 — Model loads, but panel is plain white

Once the model was found, it appeared as a solid white rectangle. There were two sub-causes that looked identical from the outside:

**2a — Wrong material system for the render engine.**
The original `model.sdf` used an OGRE1 `<script>` material (a `.material` file referencing the texture by name). This works in Gazebo Classic but **does not work in Gazebo Harmonic with `ogre2`**, which uses the OgreNext HLMS (High-Level Material System) and ignores legacy `.material` scripts entirely. When the script is silently ignored, the object renders with the plain diffuse color `(1, 1, 1, 1)` — pure white.

**Fix:** Replaced the `<script>` block with a `<pbr><metal>` block pointing directly to the PNG via a `file://` absolute URI, which is the correct way to apply textures in Gazebo Harmonic with ogre2.

**2b — The PNG itself was a near-white placeholder.**
The original `NewArUco.png` (3 878 bytes) was never a valid ArUco marker. It was a small, mostly-white grayscale image committed as a placeholder. Even if the material system had loaded it correctly, the texture would still have appeared white because the image content was white. This made the two bugs invisible to each other — fixing the material system alone changed nothing visually.

**Fix:** Generated a valid `DICT_7X7_50` ID 0 marker (512×512 px, RGB, with white border) using OpenCV's ArUco API and saved it as `NewArUco.png`.

---

### Problem 3 — Gazebo crashes (`free(): invalid pointer`)

After the material system was corrected to use PBR but before the PNG was properly replaced, an attempt to transfer the PNG via a truncated base64 string produced a **corrupted file** (1 846 bytes instead of 7 456 bytes). OgreNext's `TextureGpuManager` attempted to decode this broken PNG, triggered an internal exception, and crashed during exception cleanup with `free(): invalid pointer` in `Ogre::Exception::~Exception()`. This is a known instability in OgreNext 2.3.3 when texture decoding fails mid-load.

**Fix:** Generated the PNG directly inside the container using the correct OpenCV API for the version installed there (`cv2.aruco.Dictionary_get` + `cv2.aruco.drawMarker`, which is the pre-4.7 API — newer versions use `getPredefinedDictionary` + `generateImageMarker`).

---

### Summary Table

| # | Symptom | Root Cause | Fix |
|---|---|---|---|
| 1 | `Unable to find uri[model://aruco_target]` | `GZ_SIM_RESOURCE_PATH` not set; `model://` URI unresolvable | Use absolute path in world SDF `<uri>`; set env var in launch file |
| 2a | White panel, no texture | OGRE1 `<script>` material silently ignored by ogre2/OgreNext | Replace `<script>` with `<pbr><metal><albedo_map>` |
| 2b | White panel, no texture | Placeholder PNG was a near-white image, not an ArUco marker | Generate valid DICT_7X7_50 ID=0 PNG with OpenCV |
| 3 | `free(): invalid pointer` crash | Corrupted PNG (truncated base64 transfer) caused OgreNext exception during texture load | Regenerate PNG inside container with `cv2.aruco.drawMarker` |

---

## Acknowledgments

This research was conducted as part of the visiting researcher program at the University of West Bohemia (UWB), collaborating with the UWB Robotics Team and Professor Tomas Broum.

## License

This software is provided "as is" under the MIT License. For full details, see the LICENSE file.
