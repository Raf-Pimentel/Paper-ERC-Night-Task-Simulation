# Technical Details & Reproducibility Guide

**Paper:** *Enhancing Night-Task Navigation for Competition Rovers: A Comparative Study of Gazebo Simulations and Sensor Performance*
**Journal:** Artificial Life and Robotics (Springer)
**Repository:** [github.com/Raf-Pimentel/Paper-ERC-Night-Task-Simulation](https://github.com/Raf-Pimentel/Paper-ERC-Night-Task-Simulation)
**Full write-up:** [Google Docs](https://docs.google.com/document/d/1aZhX8T2QJ3hH65VPwwKCBMxovXjShG7vIbJGwNSziSs/edit?usp=sharing)

This document records every parameter, configuration value, and design decision needed to independently reproduce the simulation, real-world experiments, and post-processing pipeline described in the paper.

---

## 1. Software Environment

| Component | Version / Value |
|---|---|
| Operating system | Ubuntu 24.04 (inside Docker container) |
| ROS | ROS 2 Jazzy |
| Simulator | Gazebo Harmonic (gz-sim 8) |
| Render engine | OGRE2 (via gz-rendering) |
| ROS–Gazebo bridge | ros_gz_bridge (Jazzy) |
| Python | 3.x (system) |
| OpenCV | 4.6.x (cv2) |
| NumPy | 1.x |

The simulation runs entirely inside Docker. The container must have GUI access for Gazebo (`xhost +local:root` on the host before starting).

---

## 2. Simulation Scene Setup

### 2.1 World Coordinate Frame

All positions use the standard Gazebo world frame: X forward, Y left, Z up.

| Entity | World position (x, y, z) m | Orientation (roll, pitch, yaw) rad |
|---|---|---|
| Camera (observer rig base) | (0.0, 0.0, 1.0) | (0, 0, 0) |
| Spotlight | (0.0, 0.0, 1.5) | (0, 0, 0) — offset +0.5 m in Z from rig |
| ArUco marker (initial) | (2.0, 0.0, 1.0) | (0, 0, 0) |
| Sun (directional light) | (0, 0, 10) — symbolic | direction: (−0.5, 0.1, −0.9) |

The camera and spotlight are static throughout each run. Only the ArUco marker moves.

### 2.2 Physical Dimensions

| Entity | Dimensions (x, y, z) m | Notes |
|---|---|---|
| Camera body | 0.1 × 0.1 × 0.1 | Visual only, does not affect sensor |
| ArUco marker panel | 0.01 × 0.20 × 0.20 | Thin flat box; 0.01 m depth (X-axis) |
| Spotlight | — | Dimensionless point source in SDF |

### 2.3 Camera Sensor Model (Intel RealSense D435i — Simulated)

The camera is defined as a Gazebo `<sensor type="camera">` inside the observer rig.

| Parameter | Value | SDF tag |
|---|---|---|
| Sensor name | `d435i_color` | `<sensor name=...>` |
| Image width | 1920 px | `<width>` |
| Image height | 1080 px | `<height>` |
| Pixel format | R8G8B8 | `<format>` |
| Horizontal FOV | 1.204 rad (≈ 69°) | `<horizontal_fov>` |
| Near clip | 0.1 m | `<near>` |
| Far clip | 100.0 m | `<far>` |
| Update rate | 30 Hz | `<update_rate>` |
| ROS topic | `/camera/image_raw` | `<topic>` |

Derived focal length at full resolution:

```
fx = (width / 2) / tan(H-FOV / 2)
fx = (1920 / 2) / tan(1.204 / 2) ≈ 1452 px
```

The principal point is at the image centre: `cx = 960 px`, `cy = 540 px`. Distortion coefficients are set to zero (`DIST_COEFFS = [0, 0, 0, 0]`).

### 2.4 ArUco Marker

| Parameter | Value |
|---|---|
| Dictionary | `DICT_7X7_50` (OpenCV `cv2.aruco.DICT_7X7_50`) |
| Marker ID | 0 |
| Physical size | 0.20 m × 0.20 m |
| Texture file | `models/aruco_target/materials/textures/NewArUco.png` |
| Texture generation | `cv2.aruco.drawMarker(DICT_7X7_50, id=0, sidePixels=400)` on a 512×512 white canvas with 56 px padding |

The texture must be generated inside the container after first launch because it depends on the installed OpenCV version (see README §1).

---

## 3. Lighting Model

### 3.1 Total Illuminance Decomposition

The illuminance at any point in the scene is:

```
I_total(d) = I_sun + I_spotlight(d)
```

**I_sun** is implemented as a Gazebo `<light type="directional">`. Directional lights in OGRE2 have no attenuation — intensity is constant regardless of distance. This models diffuse ambient sky light.

**I_spotlight(d)** is implemented as a Gazebo `<light type="spot">` and follows the standard attenuation formula:

```
I_spotlight(d) = 1 / (Kc + Kl·d + Kq·d²)

Kc = 0.2   (constant term  — baseline loss independent of d)
Kl = 0.05  (linear term    — dominant in 1–3 m range)
Kq = 0.01  (quadratic term — dominant beyond ~3 m)
```

Example attenuation values at key distances:

| Distance (m) | I_spotlight multiplier |
|---|---|
| 1.0 | 3.85× |
| 2.0 | 2.94× |
| 3.0 | 2.17× |
| 5.0 | 1.43× |
| 10.0 | 0.77× |

### 3.2 Spotlight Geometry Parameters

| Parameter | Value | SDF tag | Meaning |
|---|---|---|---|
| Range | 25.0 m | `<range>` | No illumination beyond this distance |
| Inner angle | 0.4 rad (≈ 22.9°) | `<inner_angle>` | Full-intensity cone half-angle |
| Outer angle | 1.2 rad (≈ 68.8°) | `<outer_angle>` | Beam edge half-angle (penumbra starts here) |
| Falloff | 1.0 | `<falloff>` | Linear intensity drop from inner to outer edge |
| Direction | +X (1, 0, 0) | `<direction>` | Points toward the ArUco marker |

### 3.3 Per-Scenario Lighting Parameters

Each scenario corresponds to a time of night measured at the UWB workshop with a lux meter. The three world files share the same spotlight geometry; only the directional sun and scene ambient differ.

#### UWB Workshop Lux Measurements

| # | Daytime | Outdoor ambient (lx) | Lab avg (lx) | Lab max (lx) |
|---|---|---|---|---|
| 1 | Twilight 19:00 | 3 – 10 | 0.1 | 4.5 |
| 2 | Evening 21:00 | 0.1 – 0.3 | 0.0 | 0.2 |
| 3 | Midnight 00:00 | max 0.1 | 0.0* | 0.0* |

\* Lux meter resolution limit: values below 0.05 lx read as 0.0. True midnight value estimated at ~0.01 lx.

#### SDF Parameters Per Scenario

| Scenario | World file | Target lux | Sun `<diffuse>` | Sun `<specular>` | Scene `<ambient>` | Scene `<background>` | Spotlight `<diffuse>` | Spotlight `<specular>` | Marker `<ambient>` | Marker `<diffuse>` |
|---|---|---|---|---|---|---|---|---|---|---|
| Twilight 19:00 | `night_task_19h.sdf` | 3.0 lx | 0.150 | 0.045 | 0.075 | 0.040 | 0.42 | 0.09 | 0.32 | 0.40 |
| Evening 21:00 | `night_task_21h.sdf` | 0.3 lx | 0.015 | 0.0045 | 0.008 | 0.005 | 0.28 | 0.06 | 0.05 | 0.19 |
| Midnight 00:00 | `night_task_midnight.sdf` | 0.1 lx | 0.005 | 0.0015 | 0.003 | 0.0016 | 0.23 | 0.05 | 0.03 | 0.19 |

**Design note:** The spotlight diffuse and marker material values are tuned progressively darker across scenarios to model the reduced effective reflectance of a paper marker under diminishing illumination. The midnight background was set to 0.0016 (not 0.01) to preserve the monotonic darkening progression: 19h > 21h > midnight.

The diffuse-to-lux mapping is a linear proportionality model calibrated with `light_calibration.py`. Validated mean frame brightnesses from the screencast pipeline: 19h ≈ 44.2, 21h ≈ 15.2, midnight ≈ 5.0.

### 3.4 Ground Plane Material

Identical across all three scenarios. Intentionally dark to avoid reflective floor contamination:

| Property | Value |
|---|---|
| Ambient | (0.05, 0.05, 0.05, 1) |
| Diffuse | (0.10, 0.10, 0.10, 1) |
| Specular | (0.01, 0.01, 0.01, 1) |

---

## 4. Marker Trajectory

The ArUco marker moves autonomously via velocity commands published to `/model/aruco_target/cmd_vel` by `scripts/fly_square.py`. The camera and spotlight are static.

### 4.1 Trajectory Design

The trajectory consists of two squares in two parallel planes (different X depths), traversed at different speeds. This tests detection at two distances and across the full spatial extent of the camera FOV.

| Phase | Plane X (m) | Speed v (m/s) | Time per segment (s) | Displacement s = v·t (m) | Square side (m) |
|---|---|---|---|---|---|
| Square 1 (near) | x = 2.0 | 0.3 | 1.3 | 0.39 | 0.39 |
| Transition: forward | — | 0.5 | 1.3 | 0.65 | — |
| Square 2 (far) | x = 2.65 | 0.5 | 1.3 | 0.65 | 0.65 |
| Transition: back | — | 0.5 | 1.3 | 0.65 | — |

Each square traverses: Left → Up → Right → Down, with a 1.3 s stop between each phase transition to allow Gazebo's velocity controller to stabilise before direction changes.

### 4.2 Movement Sequence

```
Phase 1 — Square in near plane (x = 2.0 m):
  (y: +0.3 m/s) for 1.3 s  → left  0.39 m
  (z: +0.3 m/s) for 1.3 s  → up    0.39 m
  (y: −0.3 m/s) for 1.3 s  → right 0.39 m
  (z: −0.3 m/s) for 1.3 s  → down  0.39 m

Stop (0 m/s) for 1.3 s
Transition (x: +0.5 m/s) for 1.3 s  → forward ~0.65 m  →  x ≈ 2.65 m
Stop (0 m/s) for 1.3 s

Phase 2 — Square in far plane (x ≈ 2.65 m):
  (y: +0.5 m/s) for 1.3 s  → left  0.65 m
  (z: +0.5 m/s) for 1.3 s  → up    0.65 m
  (y: −0.5 m/s) for 1.3 s  → right 0.65 m
  (z: −0.5 m/s) for 1.3 s  → down  0.65 m

Stop (0 m/s) for 1.3 s
Return  (x: −0.5 m/s) for 1.3 s  → back to x = 2.0 m
Stop (0 m/s) for 1.3 s

→ Loop repeats indefinitely (Ctrl+C to stop)
```

**Total cycle time:** 14 segments × 1.3 s = 18.2 s per loop.

---

## 5. Video Processing Pipeline

All post-processing is done by `process_recordings.py`. Videos must be placed in `recordings/` with the naming convention `<source>_<scenario>_run<N>.mp4`.

### 5.1 Input Naming Convention

| Source | Prefix | Example |
|---|---|---|
| Gazebo simulation (screencast) | `sim_` | `sim_19h_run1.mp4` |
| Real-world experiment | `real_` | `real_midnight_run2.mp4` |

Valid scenario keywords: `19h`, `21h`, `midnight`.

### 5.2 ArUco Detection Parameters

Different dictionaries are used per source because the physical and simulated markers are different:

| Source | ArUco dictionary | Marker ID | Detection function |
|---|---|---|---|
| Simulation | `DICT_7X7_50` | 0 | `cv2.aruco.detectMarkers` |
| Real-world | `DICT_ARUCO_ORIGINAL` | 297 | `cv2.aruco.detectMarkers` |

`DetectorParameters`: `DetectorParameters_create()` is used for OpenCV < 4.7 compatibility (the default `DetectorParameters()` constructor segfaults in OpenCV 4.6.x). All other detector parameters are at their OpenCV defaults.

Detection is performed on a grayscale conversion of each frame (`cv2.COLOR_BGR2GRAY`).

### 5.3 Camera Intrinsics

Base intrinsics match the physical D435i at 1920×1080:

```
fx_base = (1920 / 2) / tan(1.204 / 2) ≈ 1452 px
fy_base = fx_base (square pixels assumed)
cx_base = 960 px
cy_base = 540 px
DIST_COEFFS = [0, 0, 0, 0]
```

If the video resolution differs from 1920×1080, intrinsics are scaled linearly:

```
fx_scaled = fx_base × (frame_width  / 1920)
fy_scaled = fy_base × (frame_height / 1080)
cx_scaled = cx_base × (frame_width  / 1920)
cy_scaled = cy_base × (frame_height / 1080)
```

This was validated at the real-world video resolution (1280×720):
- `fx_scaled = 1452 × (1280/1920) ≈ 968 px`
- `cx_scaled = 640 px`, `cy_scaled = 360 px`

These match the known D435i specification at 1280×720.

### 5.4 Pose Estimation

Pose is estimated with `cv2.aruco.estimatePoseSingleMarkers` using:
- `markerLength = 0.20 m` (physical marker size)
- Scaled camera matrix (see §5.3)
- Zero distortion coefficients

The function returns a translation vector `tvec` in the camera frame. The camera's coordinate system maps to the world frame as:

```
camera Z  →  world X  (depth / forward)
camera X  →  world Y  (lateral)
camera Y  →  world Z  (vertical, inverted)
```

### 5.5 Ground Truth and Error Computation

Ground truth is the marker's initial world position, fixed for all scenarios:

```
GT = (x=2.0, y=0.0, z=1.0) m
```

Pose error is computed as the difference between the estimated position (in world frame) and ground truth:

```
err_x = tvec[2] − GT[0]   (depth error)
err_y = tvec[0] − GT[1]   (lateral error)
err_z = tvec[1] − GT[2]   (vertical error)
```

**Important limitation:** Ground truth is fixed at the initial marker position, but the marker moves during each run. Therefore, non-zero `err_y` and `err_z` values reflect the marker's actual displacement, not estimation inaccuracy. Only the mean `err_x` (depth) is meaningful as a systematic bias measure, averaged across all detected frames.

### 5.6 Output CSV Columns

**Per-frame (raw) CSV** — one row per video frame:

| Column | Type | Description |
|---|---|---|
| `frame` | int | Zero-indexed frame number |
| `time_s` | float | Frame timestamp in seconds (`frame / fps`) |
| `scenario` | str | Human-readable scenario label |
| `source` | str | `simulation` or `real-world` |
| `target_lux` | float | Target illuminance for this scenario |
| `detected` | int | 1 if marker found, 0 otherwise |
| `est_x` | float | Camera-frame tvec[0] (m) |
| `est_y` | float | Camera-frame tvec[1] (m) |
| `est_z` | float | Camera-frame tvec[2] — depth (m) |
| `err_x` | float | Depth error vs ground truth (m) |
| `err_y` | float | Lateral error (m) |
| `err_z` | float | Vertical error (m) |
| `rot_x` | float | Rotation vector rvec[0] (rad) |
| `rot_y` | float | Rotation vector rvec[1] (rad) |
| `rot_z` | float | Rotation vector rvec[2] (rad) |
| `marker_area_px` | float | Marker area in pixels (shoelace formula) |
| `frame_brightness` | float | Mean pixel intensity of the grayscale frame |

**Per-run summary CSV** — one row per video:

| Column | Description |
|---|---|
| `scenario` | Scenario label |
| `source` | `simulation` or `real-world` |
| `target_lux` | Target illuminance |
| `total_frames` | Total frames processed |
| `detected_frames` | Frames with a successful detection |
| `detection_rate_pct` | `(detected_frames / total_frames) × 100` |
| `mean_err_x` | Mean depth error across detected frames (m) |
| `std_err_x` | Std dev of depth error (m) |
| `mean_err_y` / `std_err_y` | Lateral error stats (m) |
| `mean_err_z` / `std_err_z` | Vertical error stats (m) |
| `mean_brightness` | Mean frame brightness across all frames |
| `source_video` | Path to the source video file |

### 5.7 Key Metric Definitions

**Detection Success Rate (DSR):**

```
DSR (%) = (detected_frames / total_frames) × 100
```

This is the primary metric. It is independent of camera intrinsics and is not affected by the screencast resolution bias described in §7.1.

**Mean Depth Error:**

```
mean_err_x = mean(tvec[2] − 2.0)  [over all detected frames]
```

A positive value means the camera overestimates the marker's distance.

---

## 6. Real-World Experiment Setup

Physical experiments were conducted at the University of West Bohemia (UWB) workshop in collaboration with the UWB Robotics Team and Professor Tomas Broum.

### 6.1 Hardware

| Component | Specification |
|---|---|
| Camera | Intel RealSense D435i (colour stream) |
| Spotlight | Synchronized, mounted ~10–15 cm below camera |
| Light measurement | Lux meter (resolution: 0.05 lx) |
| Recording device | Host laptop (screen-captured colour stream) |

### 6.2 Video Properties

| Property | Value |
|---|---|
| Resolution | 1280 × 720 px |
| Frame rate | 15 fps |
| Format | MP4 (H.264) |
| Stream used | Colour (RGB) — not IR |
| Duration per run | ~31–34 s |
| Frames per run | 443 – 514 frames |

### 6.3 ArUco Marker

The physical marker used in the UWB workshop is from the `DICT_ARUCO_ORIGINAL` dictionary (the original ArUco library format, distinct from the OpenCV DICT_*X*_50 family):

| Property | Value |
|---|---|
| Dictionary | `DICT_ARUCO_ORIGINAL` (`cv2.aruco.DICT_ARUCO_ORIGINAL`) |
| Marker ID | 297 |
| Physical size | 0.20 m × 0.20 m (same as simulation) |

### 6.4 Lighting Conditions

Ambient light levels were set using the lab's controllable lighting and verified with a lux meter before each run. Three conditions were tested, matching the simulation scenarios:

| Scenario | Target lux | Method |
|---|---|---|
| Twilight 19:00 | 3.0 lx | Dimmed ceiling lights |
| Evening 21:00 | 0.3 lx | Near-off ceiling lights |
| Midnight 00:00 | 0.1 lx | Complete darkness (lux meter near resolution limit) |

### 6.5 Experimental Protocol

- Three independent runs per lighting condition (9 runs total)
- Marker placed at 2.0 m from the camera, 1.0 m height — matching simulation ground truth
- Spotlight active during all runs
- Camera held static; marker moved manually to replicate the simulation trajectory pattern
- Each run lasted approximately 30 seconds

---

## 7. Known Limitations

### 7.1 Simulation Depth Error — Screencast Bias

The simulation videos are screen recordings of the Gazebo viewport at approximately 440×250 px resolution. The processing pipeline scales the D435i intrinsics from 1920×1080 down to this resolution. This introduces a systematic depth bias because:

1. The Gazebo viewport FOV differs from the sensor FOV defined in the SDF
2. UI chrome (title bars, padding) reduces the actual image area below the reported resolution
3. Non-standard aspect ratios cause per-axis scaling errors

The result is an underestimated focal length (`fx_scaled ≈ 340 px` instead of ~968 px), which inflates all depth estimates by approximately +0.8 m. **This is why simulation depth errors are ~0.8 m larger than real-world errors.**

Detection rate is not affected by this bias and remains the primary reliable metric from screencasts.

To obtain unbiased pose estimates from simulation, record directly from the `/camera/image_raw` ROS topic using `ros2 bag record` — this captures 1920×1080 frames with exact intrinsics.

### 7.2 Moving Ground Truth

The ground truth position is fixed at (2.0, 0.0, 1.0) m — the marker's starting position — but the marker moves throughout the trajectory. Reported `err_y` and `err_z` therefore reflect actual marker displacement, not estimation accuracy. Only `mean_err_x` (depth) is a pure measure of systematic estimation bias.

### 7.3 Gazebo Lighting vs. Real Physics

Gazebo's OGRE2 renderer models light as a rendering approximation. It does not simulate:
- **Photon shot noise** — at very low light levels, real sensors produce random pixel noise that degrades edge detection
- **ISO gain / sensor amplification** — the camera raises gain in low light, amplifying noise
- **Spectral response** — the real colour stream has poor sensitivity below 1 lx; Gazebo treats all light levels equally
- **Spotlight cone fall-off** — the real spotlight illuminates a narrower effective area than the SDF cone parameters model

These gaps explain the large sim-to-real divergence at midnight (84% sim vs. 38% real).

### 7.4 Real-World Trajectory

The real-world marker trajectory was performed manually, approximating the simulation's two-square pattern. Small deviations in speed, timing, and path introduce variability not present in the automated simulation. This partially explains the higher run-to-run variance in real-world evening and midnight runs.

---

## 8. Results Summary

### 8.1 Simulation

| Scenario | Target Lux | Detection Rate (mean ± std) | Mean Depth Error (mean ± std) | Mean Brightness |
|---|---|---|---|---|
| Twilight 19:00 | 3.0 lx | 91.7% ± 7.4% | +1.330 ± 0.050 m | 44.2 |
| Evening 21:00 | 0.3 lx | 93.6% ± 3.8% | +1.390 ± 0.020 m | 15.2 |
| Midnight 00:00 | 0.1 lx | 84.3% ± 8.3% | +1.260 ± 0.080 m | 5.0 |

### 8.2 Real-World

| Scenario | Target Lux | Detection Rate (mean ± std) | Mean Depth Error (mean ± std) | Mean Brightness |
|---|---|---|---|---|
| Twilight 19:00 | 3.0 lx | 91.1% ± 0.6% | +0.550 ± 0.030 m | 58.8 |
| Evening 21:00 | 0.3 lx | 83.2% ± 4.2% | +0.470 ± 0.030 m | 12.3 |
| Midnight 00:00 | 0.1 lx | 37.8% ± 3.8% | +0.170 ± 0.030 m | 6.1 |

### 8.3 Sim-to-Real Gap

| Scenario | Sim DSR | Real DSR | Gap (pp) | Interpretation |
|---|---|---|---|---|
| Twilight 19:00 | 91.7% | 91.1% | −0.6 | Excellent fidelity |
| Evening 21:00 | 93.6% | 83.2% | −10.4 | Moderate divergence |
| Midnight 00:00 | 84.3% | 37.8% | −46.5 | Simulator overoptimistic |

---

## 9. How to Reproduce

### 9.1 Reproduce the Simulation

1. Clone the repository and start the Docker container (see README)
2. Generate the ArUco texture (see README §1)
3. Open four terminal tabs and follow the run sequence in README §2–§5
4. Screen-record the Gazebo viewport for approximately 30 seconds per scenario
5. Save as `sim_<scenario>_run<N>.mp4` in `recordings/`
6. Run: `python3 process_recordings.py --all`

### 9.2 Reproduce the Real-World Experiment

1. Set up the D435i camera pointing at a physical ArUco ORIGINAL ID=297 marker at 2.0 m distance, 1.0 m height
2. Attach a spotlight ~10–15 cm below the camera
3. Set ambient light to the target lux level (verify with a lux meter)
4. Record the colour stream at 1280×720 @ 15 fps while moving the marker in a two-square pattern
5. Save as `real_<scenario>_run<N>.mp4` in `recordings/`
6. Run: `python3 process_recordings.py --all`

### 9.3 Reproduce Only the Comparison

If recordings are already in `results/summary/`:

```bash
python3 process_recordings.py --compare
```

Outputs `results/summary/sim_vs_real_comparison.csv`.

---

## 10. File Reference

| File | Purpose |
|---|---|
| `process_recordings.py` | Main post-processing pipeline |
| `src/uwb_erc_sim/worlds/night_task_19h.sdf` | Twilight scenario world (3.0 lx) |
| `src/uwb_erc_sim/worlds/night_task_21h.sdf` | Evening scenario world (0.3 lx) |
| `src/uwb_erc_sim/worlds/night_task_midnight.sdf` | Midnight scenario world (0.1 lx) |
| `src/uwb_erc_sim/scripts/fly_square.py` | Marker trajectory controller |
| `src/uwb_erc_sim/scripts/erc_vision_eval.py` | Live ROS detection + evaluation node |
| `src/uwb_erc_sim/scripts/light_calibration.py` | SDF diffuse ↔ lux calibration tool |
| `recordings/sim_*.mp4` | Gazebo screencast recordings (9 runs) |
| `recordings/real_*.mp4` | Real-world experiment recordings (9 runs) |
| `results/raw/*_raw.csv` | Per-frame detection data |
| `results/summary/*_summary.csv` | Per-run aggregated statistics |
| `results/summary/all_scenarios_summary.csv` | All 18 runs combined |
| `results/summary/sim_vs_real_comparison.csv` | Aggregated sim-vs-real table |
