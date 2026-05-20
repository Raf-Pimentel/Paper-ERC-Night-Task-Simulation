#!/usr/bin/env python3
"""
process_recordings.py  —  ERC Night Task: Video Processing Pipeline
====================================================================
Processes screen recordings of Gazebo simulations, detects DICT_7X7_50
ArUco markers frame-by-frame, and saves structured CSV results.

Usage:
    python3 process_recordings.py --video recordings/sim_19h_run1.mp4 --scenario 19h
    python3 process_recordings.py --video recordings/sim_21h_run1.mp4 --scenario 21h
    python3 process_recordings.py --video recordings/sim_midnight_run1.mp4 --scenario midnight

    # Process all recordings at once:
    python3 process_recordings.py --all

Output:
    results/
    ├── raw/
    │   ├── sim_19h_run1_raw.csv        ← one row per frame
    │   ├── sim_21h_run1_raw.csv
    │   └── sim_midnight_run1_raw.csv
    └── summary/
        ├── sim_19h_run1_summary.csv    ← aggregated stats per scenario
        ├── sim_21h_run1_summary.csv
        └── all_scenarios_summary.csv   ← combined table for the paper

Raw CSV columns:
    frame, time_s, scenario, target_lux, detected,
    est_x, est_y, est_z,
    err_x, err_y, err_z,
    rot_x, rot_y, rot_z,
    marker_area_px, frame_brightness

Summary CSV columns:
    scenario, target_lux, total_frames, detected_frames,
    detection_rate_pct,
    mean_err_x, std_err_x,
    mean_err_y, std_err_y,
    mean_err_z, std_err_z,
    mean_brightness, source_video

Notes:
    - The ArUco dictionary used is DICT_7X7_50 (matches the simulation model).
    - Ground-truth marker pose matches the SDF: world (x=2.0, y=0.0, z=1.0).
    - Camera intrinsics match the D435i simulated profile (1920×1080, H-FOV=1.204 rad).
    - If the video resolution differs from 1920×1080, intrinsics are scaled automatically.
"""

import argparse
import csv
import os
import sys
from pathlib import Path

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

SCENARIOS = {
    "19h":      {"label": "Twilight 19:00",  "lux": 3.0},
    "21h":      {"label": "Evening 21:00",   "lux": 0.3},
    "midnight": {"label": "Midnight 00:00",  "lux": 0.1},
}

# Ground-truth marker world position (from model.sdf: <pose>2.0 0 1.0 0 0 0</pose>)
GT_TRANSLATION = np.array([2.0, 0.0, 1.0])

# Marker physical size in metres (matches model.sdf geometry 0.20 x 0.20)
MARKER_SIZE_M = 0.20

# D435i base intrinsics at 1920×1080 (H-FOV = 1.204 rad)
_BASE_FX = (1920 / 2) / np.tan(1.204 / 2)   # ≈ 1452 px
BASE_CAMERA_MATRIX = np.array([
    [_BASE_FX,       0,  960],
    [      0,  _BASE_FX,  540],
    [      0,        0,    1],
], dtype=np.float64)
DIST_COEFFS = np.zeros((4, 1), dtype=np.float64)

# ArUco setup — DICT_7X7_50 to match the simulation model
ARUCO_DICT   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()

# Output directory structure
RESULTS_DIR  = Path("results")
RAW_DIR      = RESULTS_DIR / "raw"
SUMMARY_DIR  = RESULTS_DIR / "summary"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def scale_camera_matrix(base_matrix: np.ndarray,
                         frame_w: int, frame_h: int) -> np.ndarray:
    """Scale intrinsics if video resolution differs from the base 1920×1080."""
    sx = frame_w / 1920
    sy = frame_h / 1080
    m = base_matrix.copy()
    m[0, 0] *= sx   # fx
    m[1, 1] *= sy   # fy
    m[0, 2] *= sx   # cx
    m[1, 2] *= sy   # cy
    return m


def compute_pose_error(tvec: np.ndarray) -> tuple[float, float, float]:
    """
    Map camera-frame translation to world-frame error vs. GT_TRANSLATION.

    The camera looks along +X in world frame:
      camera Z  ≈  world X  (depth / forward)
      camera X  ≈  world Y  (lateral)
      camera Y  ≈  world Z  (vertical, inverted)
    """
    err_x = float(tvec[2]) - float(GT_TRANSLATION[0])   # depth error
    err_y = float(tvec[0]) - float(GT_TRANSLATION[1])   # lateral error
    err_z = float(tvec[1]) - float(GT_TRANSLATION[2])   # vertical error
    return err_x, err_y, err_z


def marker_area(corners) -> float:
    """Return the pixel area of the detected marker (shoelace formula)."""
    pts = corners[0][0]
    x, y = pts[:, 0], pts[:, 1]
    return float(0.5 * abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1))))


# ---------------------------------------------------------------------------
# Core processing
# ---------------------------------------------------------------------------

def process_video(video_path: str, scenario: str) -> dict:
    """
    Process a single video file.

    Returns a dict with:
        - 'raw_csv'     : path to the per-frame CSV
        - 'summary'     : dict of aggregated stats
    """
    if scenario not in SCENARIOS:
        raise ValueError(f"Unknown scenario '{scenario}'. "
                         f"Choose from: {list(SCENARIOS.keys())}")

    scen_info    = SCENARIOS[scenario]
    target_lux   = scen_info["lux"]
    scen_label   = scen_info["label"]
    video_stem   = Path(video_path).stem

    # Ensure output dirs exist
    RAW_DIR.mkdir(parents=True, exist_ok=True)
    SUMMARY_DIR.mkdir(parents=True, exist_ok=True)

    raw_csv_path = RAW_DIR / f"{video_stem}_raw.csv"

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise FileNotFoundError(f"Cannot open video: {video_path}")

    fps        = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total_frm  = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    frame_w    = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_h    = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    camera_matrix = scale_camera_matrix(BASE_CAMERA_MATRIX, frame_w, frame_h)

    print(f"\n{'='*60}")
    print(f"Video      : {video_path}")
    print(f"Scenario   : {scen_label}  ({target_lux} lx)")
    print(f"Resolution : {frame_w}×{frame_h}  @  {fps:.1f} fps")
    print(f"Frames     : {total_frm}")
    print(f"Output CSV : {raw_csv_path}")
    print(f"{'='*60}")

    # Accumulation for summary
    detected_count = 0
    total_count    = 0
    err_x_list, err_y_list, err_z_list = [], [], []
    brightness_list = []

    with open(raw_csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "frame", "time_s", "scenario", "target_lux", "detected",
            "est_x", "est_y", "est_z",
            "err_x", "err_y", "err_z",
            "rot_x", "rot_y", "rot_z",
            "marker_area_px", "frame_brightness",
        ])

        frame_idx = 0
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            time_s      = round(frame_idx / fps, 4)
            gray        = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            brightness  = round(float(np.mean(gray)), 3)

            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, ARUCO_DICT, parameters=ARUCO_PARAMS)

            detected = 1 if (ids is not None and len(ids) > 0) else 0

            est_x = est_y = est_z = 0.0
            err_x = err_y = err_z = 0.0
            rot_x = rot_y = rot_z = 0.0
            area  = 0.0

            if detected:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, MARKER_SIZE_M, camera_matrix, DIST_COEFFS)

                tvec = tvecs[0][0]
                rvec = rvecs[0][0]

                est_x, est_y, est_z = float(tvec[0]), float(tvec[1]), float(tvec[2])
                rot_x, rot_y, rot_z = float(rvec[0]), float(rvec[1]), float(rvec[2])
                err_x, err_y, err_z = compute_pose_error(tvec)
                area = marker_area(corners)

                err_x_list.append(err_x)
                err_y_list.append(err_y)
                err_z_list.append(err_z)
                detected_count += 1

            brightness_list.append(brightness)
            total_count += 1

            writer.writerow([
                frame_idx,
                time_s,
                scen_label,
                target_lux,
                detected,
                round(est_x, 5), round(est_y, 5), round(est_z, 5),
                round(err_x, 5), round(err_y, 5), round(err_z, 5),
                round(rot_x, 5), round(rot_y, 5), round(rot_z, 5),
                round(area, 2),
                brightness,
            ])

            frame_idx += 1

            # Progress every 5 seconds of video
            if frame_idx % int(fps * 5) == 0:
                pct = 100 * frame_idx / total_frm if total_frm > 0 else 0
                det_rate = 100 * detected_count / total_count if total_count else 0
                print(f"  [{pct:5.1f}%]  frame {frame_idx}/{total_frm}  |  "
                      f"detection rate so far: {det_rate:.1f}%")

    cap.release()

    # Build summary
    detection_rate = 100 * detected_count / total_count if total_count else 0.0

    def safe_stats(lst):
        if not lst:
            return 0.0, 0.0
        return round(float(np.mean(lst)), 5), round(float(np.std(lst)), 5)

    mean_ex, std_ex = safe_stats(err_x_list)
    mean_ey, std_ey = safe_stats(err_y_list)
    mean_ez, std_ez = safe_stats(err_z_list)
    mean_br, _      = safe_stats(brightness_list)

    summary = {
        "scenario":           scen_label,
        "target_lux":         target_lux,
        "total_frames":       total_count,
        "detected_frames":    detected_count,
        "detection_rate_pct": round(detection_rate, 2),
        "mean_err_x":         mean_ex,
        "std_err_x":          std_ex,
        "mean_err_y":         mean_ey,
        "std_err_y":          std_ey,
        "mean_err_z":         mean_ez,
        "std_err_z":          std_ez,
        "mean_brightness":    round(mean_br, 2),
        "source_video":       video_path,
    }

    # Write individual summary
    summary_path = SUMMARY_DIR / f"{video_stem}_summary.csv"
    with open(summary_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(summary.keys()))
        writer.writeheader()
        writer.writerow(summary)

    print(f"\n  Detection rate : {detection_rate:.1f}%  "
          f"({detected_count}/{total_count} frames)")
    print(f"  Mean depth err : {mean_ex:+.4f} m  (std {std_ex:.4f})")
    print(f"  Saved raw CSV  : {raw_csv_path}")
    print(f"  Saved summary  : {summary_path}")

    return {"raw_csv": str(raw_csv_path), "summary": summary}


# ---------------------------------------------------------------------------
# Batch processing
# ---------------------------------------------------------------------------

def process_all():
    """
    Scan the recordings/ folder and process all videos whose names match
    the pattern sim_<scenario>_*.mp4.
    """
    recordings_dir = Path("recordings")
    if not recordings_dir.exists():
        print("ERROR: 'recordings/' directory not found. "
              "Create it and place your .mp4 files there.")
        sys.exit(1)

    # Map filename keywords to scenarios
    keyword_map = {"19h": "19h", "21h": "21h", "midnight": "midnight"}

    videos_found = sorted(recordings_dir.glob("*.mp4"))
    if not videos_found:
        print("No .mp4 files found in recordings/.")
        sys.exit(1)

    results = []
    for video_path in videos_found:
        scenario = None
        for kw, sc in keyword_map.items():
            if kw in video_path.stem:
                scenario = sc
                break
        if scenario is None:
            print(f"  Skipping {video_path.name} — "
                  f"no scenario keyword (19h/21h/midnight) in filename.")
            continue

        result = process_video(str(video_path), scenario)
        results.append(result["summary"])

    # Write combined summary
    if results:
        combined_path = SUMMARY_DIR / "all_scenarios_summary.csv"
        with open(combined_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(results[0].keys()))
            writer.writeheader()
            writer.writerows(results)
        print(f"\n{'='*60}")
        print(f"All done. Combined summary → {combined_path}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Process Gazebo simulation recordings for ArUco detection analysis")
    parser.add_argument("--video",    type=str, default=None,
                        help="Path to a single .mp4 recording")
    parser.add_argument("--scenario", type=str, default=None,
                        choices=list(SCENARIOS.keys()),
                        help="Scenario for this recording: 19h | 21h | midnight")
    parser.add_argument("--all",      action="store_true",
                        help="Process all .mp4 files in recordings/ automatically")
    args = parser.parse_args()

    if args.all:
        process_all()
    elif args.video and args.scenario:
        process_video(args.video, args.scenario)
    else:
        parser.print_help()
        print("\nExamples:")
        print("  python3 process_recordings.py --video recordings/sim_19h_run1.mp4 --scenario 19h")
        print("  python3 process_recordings.py --all")
        sys.exit(1)


if __name__ == "__main__":
    main()
