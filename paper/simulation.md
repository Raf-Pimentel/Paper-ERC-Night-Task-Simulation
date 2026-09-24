# The Simulation

*Draft of the paper's "Simulation" section. Source of truth for every number here is the repository itself — [`TECHNICAL_DETAILS.md`](../TECHNICAL_DETAILS.md), the world files in `src/uwb_erc_sim/worlds/`, and the processed results in `results/summary/`. This draft supersedes earlier working notes on the same topic; see the "Superseded notes" callouts below for what changed and why.*

The simulation was developed in a Gazebo environment; all source code is in this [GitHub repository](https://github.com/Raf-Pimentel/Paper-ERC-Night-Task-Simulation). It runs inside a Docker container using Gazebo Harmonic to simulate the camera, spotlight, ambient lighting, and ArUco marker, with ROS 2 Jazzy bridging sensor and control messages between Gazebo and the detection/evaluation nodes.

> **Figures (to insert):**
> - **Fig. 1** — The ArUco marker sheet used for detection: five distinct 8×8 markers at different sizes.
> - **Fig. 2** — The same marker sheet as it appears rendered in the Gazebo simulation.
> - **Fig. 3** — Perspective view of the camera and ArUco marker in the scene.
>
> *(Image files aren't in the repository yet — add them under `paper/figures/` and update the links above.)*

---

## Coordinate System and Scene Layout

All positions are given in the Gazebo world frame (X forward, Y left, Z up), in meters.

| Entity | World position (x, y, z) | Notes |
|---|---|---|
| Camera (observer rig) | (0.0, 0.0, 1.0) | 1 m above the ground plane |
| Spotlight | (0.0, 0.0, 1.5) | +0.5 m in Z relative to the camera rig |
| ArUco marker (initial) | (2.0, 0.0, 1.0) | Same height as the camera |

Camera and spotlight direction: **(1, 0, 0)** — both point down the world +X axis, toward the marker.

The camera and spotlight are static for the whole run; only the marker moves.

> **Superseded note:** an earlier draft listed the camera at (0, 0, 0) and the spotlight at (0, 0, 0.5), and described the spotlight as offset "10 cm forward on the X-axis." Those were rig-local coordinates, not world coordinates, and don't match the marker's world-frame ground truth (2.0, 0.0, 1.0) used throughout `process_recordings.py`. The table above is the world-frame version and is what the rest of the pipeline assumes.

## Physical Dimensions

| Entity | Dimensions (x, y, z), m |
|---|---|
| Camera body | 0.1 × 0.1 × 0.1 (visual only) |
| ArUco marker panel | 0.01 × 0.20 × 0.20 |
| Spotlight | Negligible — dimensionless point source in SDF |

## Camera Sensor Model (Intel RealSense D435i, simulated)

| Parameter | Value |
|---|---|
| Resolution | 1920 × 1080 px |
| Pixel format | R8G8B8 |
| Horizontal FOV | 1.204 rad (≈ 69°) |
| Clip range | 0.1 m – 100.0 m |
| Update rate | 30 Hz |
| ROS topic | `/camera/image_raw` |

Derived focal length at full resolution:

```
fx = (width / 2) / tan(H-FOV / 2) = (1920 / 2) / tan(1.204 / 2) ≈ 1452 px
```

Principal point at image center: `cx = 960 px`, `cy = 540 px`. Distortion coefficients are zero.

---

## ArUco Marker Trajectory

The marker moves through two square paths in two parallel planes at different depths, so that detection is exercised at two distances and across the camera's full field of view. The trajectory is published as velocity commands from `src/uwb_erc_sim/scripts/fly_square.py`.

Given constant speed per segment (`s = v · t`, with `t = 1.3 s`):

- **Near plane**, x₁ = 2.0 m, v₁ = 0.3 m/s → segment length s₁ = 1.3 × 0.3 = **0.39 m** → a 0.39 m × 0.39 m square.
- **Far plane**, v₂ = 0.5 m/s → segment length s₂ = 1.3 × 0.5 = **0.65 m** → the transition moves the marker to x₂ = 2.0 + 0.65 = **2.65 m**, and traces a 0.65 m × 0.65 m square there.

The far square is intentionally larger, to compensate for the reduced apparent marker size at greater camera distance.

A 1.3 s stop is inserted before every direction change (including the transition between planes) — this was added because Gazebo showed timing inconsistencies delivering trajectory messages in real time; the pause gives the velocity controller time to settle before reversing direction.

```
Phase 1 — square in near plane (x = 2.0 m):
  y: +0.3 m/s for 1.3 s → left  0.39 m
  z: +0.3 m/s for 1.3 s → up    0.39 m
  y: −0.3 m/s for 1.3 s → right 0.39 m
  z: −0.3 m/s for 1.3 s → down  0.39 m
  stop 1.3 s

Transition: x: +0.5 m/s for 1.3 s → forward 0.65 m (x ≈ 2.65 m)
  stop 1.3 s

Phase 2 — square in far plane (x ≈ 2.65 m):
  y: +0.5 m/s for 1.3 s → left  0.65 m
  z: +0.5 m/s for 1.3 s → up    0.65 m
  y: −0.5 m/s for 1.3 s → right 0.65 m
  z: −0.5 m/s for 1.3 s → down  0.65 m
  stop 1.3 s

Return: x: −0.5 m/s for 1.3 s → back to x = 2.0 m
  stop 1.3 s

→ loop repeats (Ctrl+C to stop)
```

Total cycle: 14 segments × 1.3 s = **18.2 s per loop**.

> **Superseded note:** an earlier draft computed s₂ = 1.3 × 0.5 = 0.69 m and x₂ = 2.69 m — both arithmetic errors (1.3 × 0.5 = 0.65, not 0.69). Corrected above; matches `TECHNICAL_DETAILS.md` §4.

---

## Lighting Model

### How Gazebo Models Light

Gazebo's rendering engine (OGRE2, via `gz-rendering`) simulates how light interacts with scene surfaces, defined through SDF light sources of three kinds: **directional** (parallel rays, e.g. the sun), **point** (uniform emission from a coordinate), and **spot** (constrained to a cone). Interaction with materials is governed by `diffuse` (matte reflection) and `specular` (shiny highlight) properties, producing the shadows and gradients that make the scene usable for computer-vision testing.

Light intensity attenuation is modeled with constant/linear/quadratic falloff factors; spotlights additionally use inner and outer cone angles to create a soft penumbra at the beam edge. This level of control matters for Sim-to-Real work, since it lets the simulated sensor reproduce the non-linear falloff and photon-starvation behavior a real depth camera experiences in low light.

### Spotlight Configuration

The spotlight is attached to the camera rig, offset +0.5 m in Z (see coordinate table above), pointing in the same +X direction as the camera — mimicking a flashlight mounted just above a real D435i.

| Parameter | Value | SDF tag |
|---|---|---|
| Range | 25 m | `<range>` |
| Inner angle | 0.4 rad (≈ 22.9°) | `<inner_angle>` |
| Outer angle | 1.2 rad (≈ 68.8°) | `<outer_angle>` |
| Falloff | 1.0 (linear) | `<falloff>` |
| Direction | (1, 0, 0) | `<direction>` |
| Casts shadows | yes | `<cast_shadows>` |

Attenuation follows `I_spotlight(d) = 1 / (Kc + Kl·d + Kq·d²)`:

| Parameter | Value | SDF tag | Meaning |
|---|---|---|---|
| Kc | 0.2 | `<constant>` | Fixed baseline loss, independent of distance |
| Kl | 0.05 | `<linear>` | Dominant in the 1–3 m range |
| Kq | 0.01 | `<quadratic>` | Dominant beyond ~3 m |

Spotlight `diffuse`/`specular` intensity is scaled down per scenario (see table below) rather than held at a fixed high value, so the beam itself gets dimmer as the simulated time gets later — closer to how a real flashlight's usable throw looks against a darker background.

> **Superseded note:** an earlier draft described the spotlight as white (`diffuse`/`specular` = 1,1,1,1), 20 m range, `inner_angle` 0.1, `outer_angle` 0.5 — these were first-iteration values later tuned (see the diffuse/specular table below) and are no longer what's in the world files. The values above are current.

### Scenario Design: Lux Targets

The three scenarios are based on lux-meter readings taken at the UWB workshop under simulated night conditions:

| # | Time of day | Outdoor ambient (lx) | Lab avg (lx) | Lab max (lx) |
|---|---|---|---|---|
| 1 | Twilight 19:00 | 3 – 10 | 0.1 | 4.5 |
| 2 | Evening 21:00 | 0.1 – 0.3 | 0.0 | 0.2 |
| 3 | Midnight 00:00 | ≤ 0.1 | 0.0* | 0.0* |

\* Lux-meter resolution limit: values below 0.05 lx read as 0.0. True midnight value estimated at ~0.01 lx.

These map to the following SDF parameters (a linear proportionality model, validated with `scripts/light_calibration.py` against the mean rendered frame brightness of each scenario: Twilight ≈ 44.2, Evening ≈ 13.9, Midnight ≈ 5.0):

| # | Scenario | Target lux | World file | Sun `diffuse` | Scene `ambient` | Spotlight `diffuse` / `specular` |
|---|---|---|---|---|---|---|
| 1 | Twilight 19:00 | 3.0 lx | `night_task_19h.sdf` | 0.150 | 0.075 | 0.42 / 0.09 |
| 2 | Evening 21:00 | 0.3 lx | `night_task_21h.sdf` | 0.015 | 0.008 | 0.28 / 0.06 |
| 3 | Midnight 00:00 | 0.1 lx | `night_task_midnight.sdf` | 0.005 | 0.003 | 0.23 / 0.05 |

The ArUco marker's own material is also tuned per scenario (ambient/diffuse of 0.32/0.40 at twilight down to 0.03/0.19 at midnight), so it gets progressively less reflective as ambient light drops — at midnight the marker is visible almost entirely because of the spotlight, not ambient light, matching real nighttime detection conditions.

Total illuminance at any point in the scene decomposes as:

```
I_total(d) = I_sun + I_spotlight(d)
```

`I_sun` (a directional light) is rendered by OGRE2 without attenuation — constant, distance-independent, as required by the model. `I_spotlight(d)` follows the attenuation formula above. All three world files share the same spotlight geometry; only the sun `diffuse` and scene `ambient` differ between scenarios.

> **Superseded note:** an earlier draft flagged these diffuse/ambient values as "first-iteration estimates" still needing validation against physical lux-meter readings. That validation is done — the values above are the ones used for every published result in this repository.

---

## Detection and Evaluation Metrics

**Detection Success Rate (DSR)** is the primary metric. For each processed video, `process_recordings.py` writes a per-frame `detected` flag (1/0) to `results/raw/<video>_raw.csv`, and aggregates it per run in `results/summary/<video>_summary.csv`:

```
DSR (%) = (detected_frames / total_frames) × 100
```

Per-scenario, per-source means (± std dev) live in `results/summary/all_scenarios_summary.csv` and `results/summary/sim_vs_real_comparison.csv`.

> **Superseded note:** an earlier draft referenced a `night_task_results.csv` file; the pipeline writes per-video `*_raw.csv` / `*_summary.csv` files under `results/raw/` and `results/summary/` instead — see [`README.md`](../README.md#repository-structure).

---

## Results

*(5 simulation runs and 3 real-world runs per scenario; see [`README.md`](../README.md#results) for the full breakdown, including per-run numbers.)*

### Detection Success Rate

| Scenario | Simulation (mean ± std) | Real-world (mean ± std) |
|---|---|---|
| Twilight 19:00 (3.0 lx) | 90.1% ± 6.4% | 92.3% ± 0.4% |
| Evening 21:00 (0.3 lx) | 84.0% ± 15.3% | 83.2% ± 4.2% |
| Midnight 00:00 (0.1 lx) | 80.8% ± 8.3% | 37.8% ± 3.8% |

Detection Success Rate decreases from Twilight → Midnight in **both** simulation and the real world, confirming the expected trend: less ambient light means less contrast at the marker's corners, which is harder for the ArUco detector. The effect is mild in simulation (90.1% → 80.8%) but severe in the real world (92.3% → 37.8%) — the spotlight and simulated sensor noise model in Gazebo do not yet reproduce the physical sensor's low-light degradation, which is the paper's central Sim-to-Real gap finding.

> **Correction to an earlier "key talking point":** a prior working note claimed the *opposite* — that detection was best at midnight and worst at twilight, attributing this to spotlight dominance over ambient glare at 3 lx. That claim does not match the final dataset (twilight is the best-performing scenario in both sim and real, by a wide margin at midnight). If that note came from a different/earlier dataset, flag it — otherwise this section reflects the validated final results and the earlier talking point should not be used in the paper.

### Pose / Depth Error — Reliability Caveat

The fundamental limitation in trusting pose estimation from the simulation's screen-recorded videos is the dependency on camera intrinsics. OpenCV's `estimatePoseSingleMarkers` computes depth approximately as:

```
depth ≈ (marker_physical_size × fx) / marker_size_in_pixels
```

`fx` (focal length) is the critical parameter here — any error in it is a constant scaling-factor error on every depth estimate. At the D435i's native 1920×1080, `fx_base ≈ 1452 px`. The pipeline linearly scales this to whatever resolution the screencast video happens to be (roughly 440–456 px wide across runs) — e.g. `fx_scaled ≈ 1452 × (452/1920) ≈ 342 px`. That linear scaling assumes an identical field of view between the video crop and the sensor, which does not hold, for three reasons:

1. **Viewport vs. sensor mismatch** — the Gazebo 3D viewport and the virtual camera sensor (`/camera/image_raw`) are separate; the sensor FOV is fixed in the SDF, but the viewport (and therefore the screencast) is user-controlled and variable.
2. **UI chrome** — the Image Display panel includes borders/padding/title bars that the screencast captures, shrinking the actual image area below the reported video resolution.
3. **Inconsistent, non-standard aspect ratios** — the crop resolution varies slightly between recording sessions (e.g. ~440–456 px wide for the original three runs per scenario, vs. a uniform 448×267 for the two runs added later), each implying a different, uncalibrated `fx`.

Net effect: an underestimated focal length inflates every simulation depth estimate — a marker at a true 2.0 m distance is reported at roughly 3.3–3.4 m (matching the observed mean depth error of +1.27 to +1.43 m across scenarios). Real-world depth error, measured from the D435i's actual native 1280×720 stream with correctly scaled intrinsics, does not carry this bias and is the more trustworthy of the two (+0.17 to +0.52 m, tight std dev).

Two ways to remove this bias for future quantitative pose analysis:

1. **Record directly from the ROS topic** — `ros2 bag record /camera/image_raw` captures the true 1920×1080 sensor frames with exact, known intrinsics. This is the recommended fix.
2. **Post-hoc intrinsic correction** — measure the exact pixel bounds of the camera panel within each existing screencast and recompute `fx_corrected` per video. Salvages existing recordings but is more error-prone than (1).

**For the paper: Detection Success Rate is a reliable, publishable metric as-is. Simulation depth/pose error should be treated as indicative only, and ideally re-derived via `/camera/image_raw` recording before being reported as a precise number.**

---

## Open Items / Future Work

Status as of the last results update in this repository:

- [x] ~~Make the processing of the real-world recordings a solid baseline for comparison~~ — done: all real-world videos reprocessed with a corrected OpenCV/ArUco pipeline; results in `results/summary/`.
- [x] ~~Rewrite the results on GitHub and the docs for better understanding~~ — done: `README.md` and `TECHNICAL_DETAILS.md` are current as of the last push.
- [x] At least 5 simulation runs per scenario — done (`sim_*_run1..5.mp4`).
- [ ] 10 simulation runs per scenario (requested for a larger sample).
- [ ] Increase environment light ~5× and re-run, to test detection under a brighter baseline.
- [ ] Vary ground-plane material (not pure white/black) for the Twilight and Evening scenarios, and compare against the current baseline.
- [ ] Test a warm/yellow-tinted sun light instead of the current neutral sun.
- [ ] Sun/ground material combination matrix (3 runs each): normal-sun × normal-ground (done, current baseline), diffuse-sun × diffuse-ground, normal-sun × diffuse-ground, diffuse-sun × normal-ground.
