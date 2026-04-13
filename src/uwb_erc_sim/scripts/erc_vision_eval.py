#!/usr/bin/env python3
"""
erc_vision_eval.py  —  ERC Night Task Vision Evaluation Node
=============================================================
Measures Detection Success Rate and Pose Estimation Error over a
30-second trajectory, for a given illumination scenario.

Illumination scenarios (from UWB workshop measurements, April 2026):
  --scenario 19h       Twilight 19:00   target 3.0 lx
  --scenario 21h       Evening  21:00   target 0.3 lx
  --scenario midnight  Midnight 00:00   target 0.1 lx

Usage:
  python3 erc_vision_eval.py --scenario 19h
  python3 erc_vision_eval.py --scenario 21h
  python3 erc_vision_eval.py --scenario midnight

Output CSV:  night_task_results_<scenario>.csv
Columns:
  Time_s, Scenario, Target_Lux, Detected,
  Est_X, Est_Y, Est_Z,   (estimated translation, camera frame)
  Err_X, Err_Y, Err_Z,   (error vs ground-truth SDF position)
  Rot_X, Rot_Y, Rot_Z    (rotation vector)
"""

import argparse
import csv
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# ---------------------------------------------------------------------------
# Illumination scenarios from UWB measurements
# ---------------------------------------------------------------------------
SCENARIOS = {
    "19h":      {"label": "Twilight 19:00",  "lux": 3.0},
    "21h":      {"label": "Evening 21:00",   "lux": 0.3},
    "midnight": {"label": "Midnight 00:00",  "lux": 0.1},
}

# ---------------------------------------------------------------------------
# Known ground-truth marker pose (from night_task_*.sdf)
# ---------------------------------------------------------------------------
GT_TRANSLATION = np.array([2.0, 0.0, 1.0])   # world (x, y, z) metres

# ArUco marker side length (metres) — matches model.sdf geometry
MARKER_SIZE_M = 0.20

# D435i simulated intrinsics (1920×1080, H-FOV=1.204 rad)
_FX = (1920 / 2) / np.tan(1.204 / 2)         # ≈ 1452 px
CAMERA_MATRIX = np.array([
    [_FX,   0,   960],
    [  0, _FX,   540],
    [  0,   0,     1],
], dtype=np.float64)
DIST_COEFFS = np.zeros((4, 1), dtype=np.float64)


class ERCVisionEval(Node):
    def __init__(self, scenario: str):
        super().__init__('erc_vision_eval')

        if scenario not in SCENARIOS:
            raise ValueError(
                f"Unknown scenario '{scenario}'. "
                f"Choose from: {list(SCENARIOS.keys())}")

        self.scenario    = scenario
        self.scen_info   = SCENARIOS[scenario]
        self.target_lux  = self.scen_info["lux"]
        self.scen_label  = self.scen_info["label"]

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_cb, 1)

        self.aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()

        self.start_time    = None
        self.test_duration = 30.0
        self.test_running  = True

        # Per-scenario output CSV
        outfile = f"night_task_results_{scenario}.csv"
        self.file   = open(outfile, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'Time_s', 'Scenario', 'Target_Lux', 'Detected',
            'Est_X', 'Est_Y', 'Est_Z',
            'Err_X', 'Err_Y', 'Err_Z',
            'Rot_X', 'Rot_Y', 'Rot_Z',
        ])

        self.get_logger().info(
            f"ERC Vision Eval | scenario: {self.scen_label} "
            f"({self.target_lux} lx) | output: {outfile}")
        self.get_logger().info("Waiting for first camera frame...")

    # ------------------------------------------------------------------
    def image_cb(self, msg):
        if not self.test_running:
            return

        if self.start_time is None:
            self.start_time = time.time()
            self.get_logger().info(
                f"Camera connected! Starting 30-second test "
                f"[{self.scen_label}]...")
            return

        elapsed = time.time() - self.start_time
        if elapsed >= self.test_duration:
            self.get_logger().info("Test complete.")
            self.test_running = False
            rclpy.shutdown()
            return

        try:
            data     = np.frombuffer(msg.data, dtype=np.uint8)
            cv_image = data.reshape((msg.height, msg.width, 3)).copy()
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

            corners, ids, _ = cv2.aruco.detectMarkers(
                cv_image, self.aruco_dict, parameters=self.aruco_params)

            detected = 1 if (ids is not None and len(ids) > 0) else 0

            est_x = est_y = est_z = 0.0
            err_x = err_y = err_z = 0.0
            rot_x = rot_y = rot_z = 0.0

            if detected:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, MARKER_SIZE_M, CAMERA_MATRIX, DIST_COEFFS)

                tvec = tvecs[0][0]
                rvec = rvecs[0][0]

                est_x, est_y, est_z = float(tvec[0]), float(tvec[1]), float(tvec[2])
                rot_x, rot_y, rot_z = float(rvec[0]), float(rvec[1]), float(rvec[2])

                # Depth error: camera Z ≈ world X for forward-facing camera
                err_x = float(tvec[2]) - float(GT_TRANSLATION[0])
                err_y = float(tvec[0]) - float(GT_TRANSLATION[1])
                err_z = float(tvec[1]) - float(GT_TRANSLATION[2])

            self.writer.writerow([
                round(elapsed, 3),
                self.scen_label,
                self.target_lux,
                detected,
                round(est_x, 4), round(est_y, 4), round(est_z, 4),
                round(err_x, 4), round(err_y, 4), round(err_z, 4),
                round(rot_x, 4), round(rot_y, 4), round(rot_z, 4),
            ])

            if int(elapsed * 10) % 10 == 0:
                self.get_logger().info(
                    f"[{self.scen_label}] t={elapsed:.1f}s | "
                    f"detected={bool(detected)} | depth_err={err_x:.3f}m")

        except Exception as e:
            self.get_logger().error(f"Frame error: {e}")


def main():
    parser = argparse.ArgumentParser(
        description='ERC Vision Evaluation — select illumination scenario')
    parser.add_argument(
        '--scenario', type=str, default='midnight',
        choices=list(SCENARIOS.keys()),
        help='Illumination scenario: 19h | 21h | midnight')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = ERCVisionEval(scenario=args.scenario)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.file.close()
        node.destroy_node()


if __name__ == '__main__':
    main()
