<!--
Finalized draft. Every number in this file is cross-checked against the repository:
world files (src/uwb_erc_sim/worlds/*.sdf), process_recordings.py, light_calibration.py,
fly_square.py, TECHNICAL_DETAILS.md, and results/summary/*.csv (5 sim runs / 3 real runs
per scenario). Remaining [PLACEHOLDER] markers are things this repository has no data
for (citations, funding, ethics) — do not fill these from guesswork.
-->

Robotica (2026), page 1 of N
doi:10.1017/xxxxx

# RESEARCH ARTICLE

## Sim-to-Real Validation of Low-Light Fiducial Marker Detection for Competition Rover Docking

Rafael Melo\*¹, Tomás Broum², Václav Mašek², and Dominik Fink²

¹ Department of Mechanical Engineering, Unicamp, Campinas, Brazil.
² Department of Mechanical Engineering, University of West Bohemia, Plzeň, Czech Republic.

\* Corresponding author. E-mail: r239356@dac.unicamp.br

Received: xx xxx xxx; Revised: xx xxx xxx; Accepted: xx xxx xxx

**Keywords:** ArUco Markers, Computer Vision, Gazebo, Low-Light Detection, Mars Rover, Sim-to-Real

---

## Abstract

The European Rover Challenge (ERC) Night Task demands reliable autonomous navigation and docking in near-zero illumination environments, specifically in the 0.1–0.5 lux range where standard RGB imaging is susceptible to photon starvation. Existing simulation tools commonly fail to replicate the non-linear noise characteristics of depth cameras under such conditions, creating a gap between virtual testing and physical deployment. This paper presents a Sim-to-Real methodology using ROS2 Jazzy and Gazebo Harmonic to evaluate ArUco fiducial marker detection for a mobile rover equipped with an Intel RealSense D435i camera. A parameterised Gazebo camera and spotlight model was used to approximate the sensor's photometric behaviour across three illuminance scenarios: Twilight (3.0 lx), Evening (0.3 lx), and Midnight (0.1 lx). Detection Success Rate and depth estimation error were measured in simulation (five runs per scenario) and compared against physical experiments conducted at the University of West Bohemia (three runs per scenario). The simulation tracks physical detection performance closely down to 0.3 lx (within ~2 percentage points), then diverges sharply at 0.1 lx, where simulated detection remains high (80.8%) while physical detection collapses (37.8%) — a gap attributable to the absence of any sensor-noise model, IR speckle included, in the current virtual camera. The validated pipeline is directly applicable to the UWB Robotics team's pre-competition preparation and provides a scalable framework for testing autonomous vision systems in logistically constrained space exploration scenarios.

---

## 1. Introduction

### 1.1 The ERC Night Task: Constraints and Failure Modes

The European Rover Challenge (ERC) is an annual international university competition in which student teams operate a semi-autonomous Mars-analogue rover across a simulated planetary surface. The Night Task is one of its most technically demanding modules: the rover must autonomously navigate to a docking station and detect ArUco fiducial markers under illuminances of 0.1–0.5 lux, with the human operator either blinded or absent. All visual processing must be performed on-board, using only the rover's own illumination source.

This near-zero illumination constraint exposes a specific failure mode of commodity depth cameras. At sub-lux illuminances, photon starvation reduces the number of photoelectrons reaching each pixel below the shot-noise floor, causing a sharp drop in signal-to-noise ratio that standard autogain cannot fully compensate. For the D435i in particular, the consequence is an irregular noise floor driven partly by the IR structured-light projector, whose speckle pattern leaks into the RGB channel at very low ambient levels. Marker detection algorithms such as OpenCV's ArUco pipeline are sensitive to this kind of localised intensity variation, which can produce false gradient responses that either suppress or corrupt the marker border. Beyond the photometric challenge, the Night Task imposes a docking geometry requirement: the rover must approach the station within a specified angular and translational tolerance, making pose estimation accuracy — not just binary detection — the operationally relevant metric.

### 1.2 Hardware Overview: The Intel RealSense D435i in Night-Task Conditions

The Intel RealSense D435i is a stereo depth camera combining a global-shutter RGB imager (1920×1080, up to 30 fps) with an active IR stereo pair and an IMU. Two properties are specifically relevant to the Night Task scenario. First, the global-shutter architecture ensures simultaneous exposure of all pixels, which eliminates the rolling-shutter motion distortion that would arise when the rover approaches the marker at 0.3–0.5 m/s. This matters because detection is computed from individual frames rather than long-exposure composites, and any intra-frame geometric distortion would corrupt the four-corner localisation on which the ArUco pose solver depends. Second, the D435i's IR stereo projector operates at 850 nm, a wavelength outside the visible range that can be paired with a narrowband LED or laser spotlight — providing active illumination that remains effectively invisible to human observers. In the UWB rover's configuration, a spotlight co-aligned with the camera provides the sole illumination source during the Night Task.

Sensor selection for the UWB rover was discussed in prior work [1], which compared time-of-flight sensors, monocular cameras, and stereo depth cameras and concluded that the D435i offered the best tradeoff between depth accuracy, frame rate, and ROS2 ecosystem integration. The present work builds directly on that selection, focusing on characterising how the D435i's detection performance in the Night Task degrades with illuminance and whether a Gazebo simulation can reliably predict that degradation.

### 1.3 Software Stack

The simulation and evaluation pipeline relies on three open-source frameworks. ROS2 (Jazzy distribution) provides the middleware layer: standardised topic definitions for `sensor_msgs/Image`, a unified coordinate frame system via `tf2`, and the `ros_gz_bridge` package for bidirectional topic translation between ROS2 and Gazebo. Gazebo Harmonic (`gz-sim` 8) serves as the physics and rendering backend, using the Ogre2 render engine to support physically-based materials, spotlight attenuation, and per-sensor camera plugins. RViz2 is used throughout for real-time visualisation of camera feeds and ArUco detection overlays. The full pipeline — world files, detection scripts, and post-processing — is built and version-pinned via a Docker image defined in the project repository, so that the software environment described here is reproducible independent of host configuration.

### 1.4 The Simulation-to-Reality Gap in Low-Light Vision

In university-level competition environments, physical testing time is limited by shared laboratory access and geographic distance from the ERC arena. Simulation thus plays a central role in the development cycle; however, its utility is bounded by how faithfully it can reproduce the conditions that determine detection success in the real system. Existing Sim-to-Real work in robotics vision has focused on domain randomisation [2] and photorealistic scene rendering [3], but comparatively little attention has been paid to calibrating the absolute illuminance levels in simulation against physical lux measurements. This is particularly relevant in the low-light regime, where the mapping from scene geometry to detection probability is non-linear and dominated by sensor-specific noise sources that differ across platforms.

This paper addresses that gap through two concrete contributions: (i) a Gazebo-Harmonic camera and spotlight parameter set — attenuation coefficients, spotlight cone geometry, and per-scenario ambient/diffuse settings — calibrated against physical lux-meter readings to approximate the D435i's photometric behaviour in the 0.1–3.0 lux range; and (ii) a comparative analysis of Detection Success Rate and depth estimation error between simulation and physical experiment, identifying the illuminance regime where the simulation provides a reliable proxy and where it systematically diverges.

---

## 2. Methodology

### 2.1 Virtual Environment Setup

#### 2.1.1 Scene Layout

The camera and spotlight are mounted on a static observer rig at world position (0.0, 0.0, 1.0) m, 1 m above the ground plane; the spotlight is offset a further +0.5 m in Z, at (0.0, 0.0, 1.5) m. Both point along the world +X axis. The ArUco marker's initial position is (2.0, 0.0, 1.0) m — level with the camera, 2 m along its viewing axis. The camera and spotlight remain static throughout each run; only the marker moves (§2.1.3).

#### 2.1.2 Camera Configuration Parameters

The D435i is modelled in Gazebo Harmonic as a camera sensor within the static observer rig. The plugin parameters approximate the physical camera's primary optical characteristics: a horizontal field of view of 1.204 rad (≈ 69.0°), a resolution of 1920×1080, and an update rate of 30 Hz. Clipping planes are set to 0.1 m (near) and 100 m (far), consistent with the D435i's rated depth range. The image format is R8G8B8, matching the colour output consumed by the ArUco detection pipeline.

**No explicit sensor-noise model is applied to the simulated camera** — the plugin definition used to generate every result in this paper contains no `<noise>` element, so the rendered frames carry only whatever numerical noise Ogre2's rendering pipeline introduces incidentally. This is a limitation, not a design choice, and is a leading candidate explanation for the Sim-to-Real divergence reported in §4.4: neither read noise, photon shot noise, nor the D435i's IR-projector speckle pattern (which leaks into the RGB channel at very low ambient levels) is currently reproduced in the virtual sensor.

#### 2.1.3 ArUco Marker and Trajectory

The ArUco marker (`DICT_7X7_50`, ID 0) is modelled as a 0.20 m × 0.20 m × 0.01 m thin box. It is actuated by publishing `geometry_msgs/Twist` commands to `/model/aruco_target/cmd_vel` from `scripts/fly_square.py`, bridged into Gazebo via `ros_gz_bridge`.

The trajectory sweeps two square paths in two parallel planes at different depths, exercising detection at two distances and across the camera's field of view. Each segment runs for a fixed duration of t = 1.3 s at a fixed speed, so displacement follows s = v·t:

- **Near plane**, x₁ = 2.0 m, v₁ = 0.3 m/s → segment length s₁ = 1.3 × 0.3 = 0.39 m, tracing a 0.39 m × 0.39 m square.
- **Far plane**, v₂ = 0.5 m/s → the transition moves the marker by s₂ = 1.3 × 0.5 = 0.65 m, to x₂ = 2.0 + 0.65 = **2.65 m**, where it traces a 0.65 m × 0.65 m square.

The far square is deliberately larger, compensating for the marker's reduced apparent size at greater camera distance. A 1.3 s stop is inserted before every direction change — including the transition between planes — because Gazebo showed timing inconsistencies delivering trajectory messages in real time; the pause gives the velocity controller time to settle before reversing direction. One full loop (both squares plus transitions and stops) takes 14 segments × 1.3 s = 18.2 s; each recorded evaluation run covers approximately 30 s, i.e. roughly 1.5 loops.

#### 2.1.4 Spotlight Model: Parametric Attenuation

A spot light is used in place of an omnidirectional point light to match the physical flashlight co-aligned with the camera; a point light would distribute energy uniformly, which does not represent a directional illuminator. The Gazebo spot light is parameterised by an inner cone angle (θ_inner = 0.4 rad ≈ 22.9°) within which intensity is uniform, and an outer cone angle (θ_outer = 1.2 rad ≈ 68.8°) defining the edge of a linear falloff (penumbra), with a 25 m range.

Distance attenuation follows the Ogre2 model:

```
att(d) = 1 / (Kc + Kl·d + Kq·d²)
```

with Kc = 0.2 (constant), Kl = 0.05 (linear), Kq = 0.01 (quadratic) — chosen to approximate the physical inverse-square falloff while avoiding a singularity at d = 0.

The spotlight's `diffuse`/`specular` channel values — rather than the attenuation coefficients — are what is tuned per scenario (Table 3, §2.1.5); the attenuation model itself is shared by all three world files. `scripts/light_calibration.py` provides a starting-point heuristic for that tuning, relating mean rendered frame brightness to target illuminance as `mean_brightness ≈ K_lux · target_lux` with an empirically chosen `K_lux = 12.75`, then iterating the SDF `diffuse` values by hand until the rendered scene's brightness looked right for that scenario relative to the others. This heuristic is not a tight fit at the low end — a strict `K_lux` prediction is off by roughly 3–4× at 0.1–0.3 lx against the brightness actually observed in the final recordings (≈ 44.2 at 3.0 lx, ≈ 13.9 at 0.3 lx, ≈ 5.0 at 0.1 lx) — so it should be read as a rough starting point for manual tuning, not a validated radiometric model; it does not account for spectral reflectance or the D435i's wavelength-dependent channel response either. The `diffuse`/`ambient` values in Table 2 are the final, hand-tuned parameters actually used to generate every result in this paper, independent of how well the linear heuristic that seeded them fits in hindsight.

#### 2.1.5 Scenario Design: Lux Targets

The three scenarios are based on lux-meter readings taken at the UWB workshop under simulated night conditions.

**Table 1.** UWB workshop lux measurements.

| # | Time of day | Outdoor ambient (lx) | Lab avg (lx) | Lab max (lx) |
|---|---|---|---|---|
| 1 | Twilight 19:00 | 3 – 10 | 0.1 | 4.5 |
| 2 | Evening 21:00 | 0.1 – 0.3 | 0.0 | 0.2 |
| 3 | Midnight 00:00 | ≤ 0.1 | 0.0\* | 0.0\* |

\* Lux-meter resolution limit: values below 0.05 lx read as 0.0; true midnight value estimated at ~0.01 lx.

**Table 2.** SDF parameters derived from Table 1 for each scenario.

| # | Scenario | Target lux | World file | Sun `diffuse` | Scene `ambient` | Spotlight `diffuse` / `specular` |
|---|---|---|---|---|---|---|
| 1 | Twilight 19:00 | 3.0 lx | `night_task_19h.sdf` | 0.150 | 0.075 | 0.42 / 0.09 |
| 2 | Evening 21:00 | 0.3 lx | `night_task_21h.sdf` | 0.015 | 0.008 | 0.28 / 0.06 |
| 3 | Midnight 00:00 | 0.1 lx | `night_task_midnight.sdf` | 0.005 | 0.003 | 0.23 / 0.05 |

The ArUco marker's own material is likewise tuned per scenario (ambient/diffuse from 0.32/0.40 at Twilight down to 0.03/0.19 at Midnight), so it becomes progressively less reflective as ambient light drops — at Midnight the marker is visible almost entirely because of the spotlight, not ambient light. Total illuminance at any point in the scene decomposes as I_total(d) = I_sun + I_spotlight(d), where I_sun (a directional light) is rendered without attenuation by Ogre2 — constant and distance-independent — and I_spotlight(d) follows the attenuation model in §2.1.4. All three world files share the same spotlight geometry; only the sun `diffuse` and scene `ambient` differ between scenarios.

> **Figures (to insert):**
> - **Fig. 1** — The ArUco marker sheet used for detection: five distinct 8×8 markers at different sizes.
> - **Fig. 2** — The same marker sheet as it appears rendered in the Gazebo simulation.
> - **Fig. 3** — Perspective view of the camera and ArUco marker in the scene, showing the D435i rig and co-aligned spotlight.
>
> *(Image files are not yet in the repository — add under `paper/figures/` and update these captions with real filenames.)*

### 2.2 Real-World Setup

Physical experiments were conducted at the University of West Bohemia mechanical engineering workshop, in collaboration with the UWB Robotics Team and Professor Tomáš Broum. The D435i was mounted on a stationary rig at a height of approximately 1.0 m — matching the simulation's ground-truth marker height — with a spotlight synchronised to the camera and mounted approximately 10–15 cm below it (a different physical offset from the simulation's +0.5 m vertical spotlight placement; see §4.1 for the implications of this difference).

An ArUco marker from the `DICT_ARUCO_ORIGINAL` dictionary, ID 297 — physically 0.20 m × 0.20 m, printed on matte paper — was placed on a stand at a fixed distance of 2.0 m from the camera, at the camera's height. Unlike the simulated trajectory (§2.1.3), the physical protocol used a single fixed distance rather than a near/far sweep; the marker was moved manually within that plane to approximate the simulation's lateral/vertical square pattern. Ambient illuminance was controlled by progressively covering the workshop's overhead lights and verified with a lux meter at the marker plane, yielding the same three target conditions as the simulation: Twilight (3.0 lx), Evening (0.3 lx), Midnight (0.1 lx).

Three independent ~30 s trials were recorded per illuminance condition (nine runs total; ≈ 4.7 minutes of footage), at 1280×720, 15 fps, H.264, colour stream only.

**Table 3.** Physical experiment configuration.

| Parameter | Value |
|---|---|
| Camera height | ≈ 1.0 m |
| Spotlight offset from camera | ≈ 10–15 cm (below) |
| Marker distance | 2.0 m (fixed) |
| Marker dictionary / ID | `DICT_ARUCO_ORIGINAL`, ID 297 |
| Illuminance levels | 3.0 / 0.3 / 0.1 lx |
| Runs per condition | 3 (9 total) |
| Recording | 1280×720 @ 15 fps, H.264, colour only |
| Total footage | ≈ 4.7 min |

### 2.3 ArUco Detection Pipeline

Detection in both environments uses a common post-processing pipeline (`process_recordings.py`) running OpenCV's `cv2.aruco.detectMarkers`, applied to a grayscale conversion of each frame, with the detector at its OpenCV default parameters. Because the simulated and physical markers use different dictionaries (§2.1.3, §2.2), detection uses `DICT_7X7_50` (ID 0) for simulation footage and `DICT_ARUCO_ORIGINAL` (ID 297) for physical footage. Simulated camera frames are nominally captured at 30 fps; the physical recordings were captured at 15 fps (§2.2), so detection statistics are reported as a per-frame success rate rather than an absolute frame count, to keep the two sources comparable.

Depth is estimated with `cv2.aruco.estimatePoseSingleMarkers`, using a marker side length of 0.20 m and a camera intrinsic matrix derived analytically from the D435i's rated horizontal field of view (§2.1.2) — not a factory calibration file — scaled to each video's resolution, with zero assumed distortion. Ground truth is a fixed constant, the marker's nominal position (2.0, 0.0, 1.0) m in simulation, or the fixed 2.0 m stand-off in the physical setup; it is **not** read from a live pose topic. Because the marker moves during each run (§2.1.3, §2.2), lateral and vertical error computed against this fixed reference reflect the marker's actual displacement as much as estimation inaccuracy — depth error is therefore the only pose component treated as a systematic-bias measure in this paper (see §4.3 for a full discussion of depth-error reliability in the simulated case).

---

## 3. Results

*(Five simulation runs and three real-world runs per scenario. Full per-run data in `results/summary/all_scenarios_summary.csv`.)*

### 3.1 Detection Success Rate

Detection Success Rate (DSR) is the fraction of processed frames in which `detectMarkers` returned the target marker:

```
DSR = (N_detected / N_total) × 100%
```

**Table 4.** Detection Success Rate (%), simulation vs. physical, mean ± std dev.

| Scenario | Target lux | Simulation DSR | Physical DSR | Gap (Physical − Sim, pp) |
|---|---|---|---|---|
| Twilight 19:00 | 3.0 lx | 90.1% ± 6.4% | 92.3% ± 0.4% | +2.2 |
| Evening 21:00 | 0.3 lx | 84.0% ± 15.3% | 83.2% ± 4.2% | −0.8 |
| Midnight 00:00 | 0.1 lx | 80.8% ± 8.3% | 37.8% ± 3.8% | −43.0 |

At the two brighter scenarios, simulation and physical detection agree closely — within ±2.2 percentage points at both 3.0 lx and 0.3 lx — despite an order-of-magnitude difference in target illuminance between them. This indicates the simulation is a reliable proxy for physical detection performance at least down to 0.3 lx. At 0.1 lx (Midnight), the two diverge sharply: physical DSR collapses to 37.8% while simulated DSR remains high at 80.8%, a 43.0 percentage-point gap. This is the paper's central Sim-to-Real finding — see §4.4.

*(An earlier working hypothesis assumed detection would be best at Midnight, reasoning that the spotlight dominates completely once ambient light is negligible, while ambient light at Twilight creates competing glare. The final dataset does not support this: Twilight is the best-performing scenario in both environments, and Midnight the worst, by a wide margin physically. This should not be treated as a valid claim going forward.)*

### 3.2 Depth Estimation Error

**Table 5.** Mean depth error (m), simulation vs. physical, mean ± std dev.

| Scenario | Simulation | Physical |
|---|---|---|
| Twilight 19:00 | +1.43 ± 0.20 m | +0.52 ± 0.02 m |
| Evening 21:00 | +1.37 ± 0.05 m | +0.42 ± 0.01 m |
| Midnight 00:00 | +1.27 ± 0.07 m | +0.17 ± 0.03 m |

Physical depth error is small, tightly distributed, and *decreases* toward Midnight — the opposite of the "harder to localise in the dark" intuition, and likely explained by the fact that at Midnight only the clearest, highest-contrast detections survive at all (physical DSR is 37.8%; the runs are dominated by close-range, high-confidence detections rather than a representative sample of the full trajectory). Simulated depth error is roughly three times larger and does not track illuminance in any consistent direction. As discussed in §4.3, this is a known artefact of how the simulation footage was recorded and should not be read as a genuine measure of the simulated sensor's pose accuracy. Rotational pose error is not reported: the pipeline has no ground-truth orientation reference, only a fixed-position ground truth, so no rotational error signal currently exists in this dataset.

---

## 4. Discussion

### 4.1 Illuminance Calibration and Residual Discrepancies

Total illuminance in the simulation was modelled as the sum of a distance-independent directional light (an ambient-light surrogate) and a distance-dependent spot light (the rover's own illumination). The directional light's `diffuse` parameter was calibrated against UWB workshop lux-meter readings — 0.150 (Twilight), 0.015 (Evening), 0.005 (Midnight) — and the spotlight's inverse-square falloff was fixed at Kc = 0.2, Kl = 0.05, Kq = 0.01, approximating the physical flashlight's intensity decay over the tested range.

This calibration aligns the *aggregate* illuminance level at the marker plane, not the spectral or spatial distribution of the light field, and not the physical rig geometry: the real spotlight sits ~10–15 cm below the camera, while the simulated one sits 0.5 m above it. Differences in reflectance angle, wall reflections, camera gain response, and this positional offset between the virtual and physical rigs introduce residual discrepancies that plausibly contribute to the detection divergence discussed in §4.4, on top of the absent noise model identified in §2.1.2.

### 4.2 Figure

> **Figure 1.** Screenshot of the Gazebo simulation environment showing the D435i rig, co-aligned spotlight, and ArUco marker target. *(To insert — see figure placeholders in §2.1.5.)*

### 4.3 Depth Error Reliability

The depth values reported in Table 5 for simulation carry a known, structural bias and should be treated as indicative only, not as a precise pose-accuracy measurement. Depth is computed by OpenCV as approximately:

```
depth ≈ (marker_physical_size × fx) / marker_size_in_pixels
```

`fx` (focal length) is the critical parameter: any error in it is a constant scaling-factor error on every depth estimate. At the D435i's native 1920×1080, `fx_base ≈ 1452 px`. Simulation footage, however, was captured as a screen recording of the Gazebo viewport rather than directly from the ROS image topic, at resolutions of roughly 440–456 px wide (and, for two of the five runs per scenario, a uniform 448×267 px from a later recording session) — not 1920×1080. The pipeline linearly rescales `fx` to match each video's resolution, e.g. `fx_scaled ≈ 1452 × (452/1920) ≈ 342 px`, but this scaling assumes an identical field of view between the screencast crop and the sensor, which does not hold, for three reasons: (i) the Gazebo 3D viewport and the virtual camera sensor (`/camera/image_raw`) are separate — the sensor's FOV is fixed in the SDF, but the viewport (and therefore what the screencast captures) is user-controlled and variable; (ii) the on-screen Image Display panel includes borders, padding, and title-bar chrome that the screencast captures, shrinking the true image area below the reported video resolution; (iii) the exact crop resolution is not even consistent between recording sessions, so each carries a slightly different, uncalibrated implied `fx`.

The net effect is a systematically underestimated focal length, which inflates every simulated depth estimate — a marker at a true 2.0 m distance is reported at roughly 3.3–3.4 m, consistent with the +1.27 to +1.43 m mean offsets in Table 5. Physical depth error, measured from the D435i's actual native 1280×720 stream with correctly scaled intrinsics, does not carry this bias and is the more trustworthy of the two figures.

Two options exist to remove this bias in future work: (1) record directly from the ROS topic with `ros2 bag record /camera/image_raw`, capturing true 1920×1080 frames with exact, known intrinsics — the recommended fix; or (2) apply a post-hoc intrinsic correction by measuring the exact pixel bounds of the camera panel within each existing screencast and recomputing `fx` per video, which salvages existing recordings but is more error-prone. **For this paper: Detection Success Rate (§3.1) is treated as the reliable, publishable metric; simulated depth error is reported for completeness but should not be over-interpreted.**

### 4.4 Sim-to-Real Gap Analysis

The results in Table 4 define two illuminance regimes for the simulation's predictive validity, bounded by what was actually tested. At 3.0 lx and 0.3 lx, DSR divergence between simulation and physical experiment stayed within ±2.2 percentage points — small enough that the simulation can be treated as a reliable substitute for physical testing during parameter optimisation and detector configuration at these light levels. At 0.1 lx, the simulation overestimates detection robustness by 43.0 percentage points. No data point between 0.3 lx and 0.1 lx was collected, so the exact threshold at which this divergence begins cannot be pinned down more precisely than "between 0.1 and 0.3 lx" from the current dataset.

The most direct candidate cause is the complete absence of a sensor-noise model in the current camera plugin (§2.1.2): no read noise, no photon shot noise, and critically, none of the D435i's IR-projector speckle pattern that leaks into the RGB channel at sub-lux illuminances in the physical sensor. That speckle is a spatially-correlated, burst-like noise source, distinct from the smoother Gaussian read-noise floor a simple noise model would add — it produces localised false gradients that can either suppress the ArUco corner detector or generate confounding responses near the marker edges, exactly the kind of failure that would explain physical detection collapsing at 0.1 lx while the (noise-free) simulated image remains clean enough to detect reliably. The rig geometry and lighting-model discrepancies noted in §4.1 are plausible secondary contributors but are unlikely to explain a 43-point gap on their own.

Future work should add an explicit noise model to the camera plugin — starting with a basic Gaussian read-noise term, and ideally extending to a speckle-pattern model parameterised from physical recordings at matched illuminance — and re-run the Midnight scenario to test whether it closes the observed gap.

---

## 5. Conclusion

This paper presented a Sim-to-Real methodology for evaluating ArUco marker detection under the extreme low-light conditions of the ERC Night Task. Using ROS2 Jazzy and Gazebo Harmonic with a calibrated Ogre2 rendering pipeline, a parameterised camera and spotlight model was developed to approximate the photometric behaviour of an Intel RealSense D435i across three illuminance scenarios (0.1–3.0 lx). Physical validation at the University of West Bohemia showed that the simulation is a reliable detection proxy at 3.0 lx and 0.3 lx, with DSR divergence within ±2.2 percentage points at both.

At 0.1 lx, simulation systematically overestimates DSR by 43.0 percentage points. The most direct explanation is that the current camera model applies no sensor-noise term at all — the physical camera's read noise, shot noise, and especially its IR-projector speckle pattern are all currently unmodelled — which is now a concrete, actionable target for future simulation work. Depth estimation error, by contrast, is only reliable from the physical dataset in its present form: the simulation's screen-recorded videos carry a resolution-dependent intrinsics bias (§4.3) that should be resolved by recording directly from the ROS image topic before depth error is reported as a precise number in future iterations of this work.

The validated pipeline, world files, and detection code are publicly available in the project repository and directly applicable to the UWB Robotics team's pre-competition preparation, reducing dependency on scarce physical testing time. The methodology generalises to other fiducial marker systems and camera platforms operating in photon-limited environments.

---

## Required Declarations

**Author Contributions.** Rafael Melo conceived and designed the simulation framework, implemented the ROS2/Gazebo pipeline, and led manuscript preparation. Tomáš Broum coordinated review iterations and contributed to methodology design. Václav Mašek and Dominik Fink conducted physical data collection and contributed to experimental validation at the UWB Robotics Office.

**Financial Support.** [PLACEHOLDER — author to complete; no funding information available in the project repository.]

**Conflicts of Interest.** [PLACEHOLDER — author to complete.]

**Ethical Approval.** Not applicable.

**Acknowledgements.** Research conducted as part of the visiting researcher program at the University of West Bohemia (UWB), in collaboration with the UWB Robotics Team and Professor Tomáš Broum.

---

## References

[1] [PLACEHOLDER — full citation needed for the prior UWB sensor-comparison study referenced in §1.2.]

[2] J. Tobin, R. Fong, A. Ray et al., "Domain randomization for transferring deep neural networks from simulation to the real world," in *Proc. IEEE/RSJ Int. Conf. Intelligent Robots and Systems (IROS)*, Vancouver, Canada, 2017, pp. 23–30.

[3] M. Müller, A. Dosovitskiy, B. Ghanem et al., "Driving in the matrix: Can virtual worlds replace human-generated annotations for real world tasks?" in *Proc. IEEE Int. Conf. Robotics and Automation (ICRA)*, Brisbane, Australia, 2018, pp. 746–753.

[4] [PLACEHOLDER — additional reference, not yet specified.]
