#!/usr/bin/env python3
"""
light_calibration.py  —  Luminosity Calibration Tool
=====================================================
Maps Gazebo pixel brightness to real-world lux values (measured at UWB workshop)
and recommends SDF <diffuse> adjustments.

Illumination scenarios (from UWB workshop measurements, April 2026):
  --scenario 19h       Twilight 19:00   target 3.0 lx
  --scenario 21h       Evening  21:00   target 0.3 lx
  --scenario midnight  Midnight 00:00   target 0.1 lx

Usage:
  python3 light_calibration.py --scenario 19h
  python3 light_calibration.py --scenario 21h
  python3 light_calibration.py --scenario midnight

  # Or provide a custom lux value:
  python3 light_calibration.py --target-lux 2.5

Output CSV:  luminosity_test_results_<scenario>.csv
Columns:
  Time_s, Mean_Brightness, Target_Lux, Estimated_Lux, Ratio, Recommendation

How to calibrate
-----------------
1. Measure ambient lux at UWB workshop with lux meter (camera position, facing marker).
2. Run this script with --scenario matching the condition being tested.
3. Read the Recommendation column — that is the <diffuse> r=g=b value for the SDF.
4. Update the corresponding worlds/night_task_<scenario>.sdf.
5. Repeat until Estimated_Lux ≈ Target_Lux (within ±10%).
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
# Illumination scenarios
# ---------------------------------------------------------------------------
SCENARIOS = {
    "19h":      {"label": "Twilight 19:00",  "lux": 3.0,  "diffuse_init": 0.150},
    "21h":      {"label": "Evening 21:00",   "lux": 0.3,  "diffuse_init": 0.015},
    "midnight": {"label": "Midnight 00:00",  "lux": 0.1,  "diffuse_init": 0.005},
}

# Empirical calibration constant: mean_brightness (0–255) ≈ K_LUX * lux
# Validate during first physical session at UWB by plotting brightness vs lux meter.
K_LUX = 12.75


class LightCalibration(Node):
    def __init__(self, target_lux: float, scenario_label: str,
                 current_diffuse: float, duration: float, output: str):
        super().__init__('light_calibration')

        self.target_lux      = target_lux
        self.scenario_label  = scenario_label
        self.current_diffuse = current_diffuse
        self.duration        = duration
        self.output          = output

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_cb, 1)

        self.start_time = None
        self.running    = True

        self.file   = open(output, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'Time_s', 'Scenario', 'Mean_Brightness', 'Target_Lux',
            'Estimated_Lux', 'Ratio', 'Current_Diffuse', 'Recommendation'])

        self.get_logger().info(
            f"Light calibration | {scenario_label} | "
            f"target: {target_lux:.2f} lx | "
            f"current diffuse: {current_diffuse} | output: {output}")

    def image_cb(self, msg):
        if not self.running:
            return

        if self.start_time is None:
            self.start_time = time.time()
            self.get_logger().info("Camera frame received — starting calibration...")
            return

        elapsed = time.time() - self.start_time
        if elapsed >= self.duration:
            self._finish()
            return

        try:
            data  = np.frombuffer(msg.data, dtype=np.uint8)
            frame = data.reshape((msg.height, msg.width, 3)).copy()
            gray  = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

            mean_brightness = float(np.mean(gray))
            estimated_lux   = mean_brightness / K_LUX
            ratio           = estimated_lux / self.target_lux if self.target_lux > 0 else 0.0
            recommended     = round(self.current_diffuse / ratio, 5) if ratio > 0 else self.current_diffuse

            self.writer.writerow([
                round(elapsed, 2),
                self.scenario_label,
                round(mean_brightness, 2),
                self.target_lux,
                round(estimated_lux, 3),
                round(ratio, 4),
                self.current_diffuse,
                recommended,
            ])

            if int(elapsed * 10) % 20 == 0:
                self.get_logger().info(
                    f"[{self.scenario_label}] t={elapsed:.1f}s | "
                    f"brightness={mean_brightness:.1f} | "
                    f"est={estimated_lux:.3f} lx / target={self.target_lux:.2f} lx | "
                    f"ratio={ratio:.3f} → recommended diffuse={recommended}")

        except Exception as e:
            self.get_logger().error(f"Frame error: {e}")

    def _finish(self):
        self.running = False
        self.file.close()
        self.get_logger().info(
            f"Calibration complete → {self.output}\n"
            f"Update <diffuse> in worlds/night_task_{self.scenario_label.split()[0].lower()}.sdf "
            f"using the last Recommendation value, then re-run.")
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Gazebo light calibration per scenario')
    parser.add_argument('--scenario',    type=str,   default=None,
                        choices=list(SCENARIOS.keys()),
                        help='Scenario preset: 19h | 21h | midnight')
    parser.add_argument('--target-lux', type=float, default=None,
                        help='Override target lux (ignores --scenario lux)')
    parser.add_argument('--duration',   type=float, default=10.0)
    parser.add_argument('--output',     type=str,   default=None)
    args, _ = parser.parse_known_args()

    # Resolve target lux and label
    if args.scenario and args.scenario in SCENARIOS:
        info           = SCENARIOS[args.scenario]
        target_lux     = args.target_lux or info["lux"]
        scenario_label = info["label"]
        diffuse_init   = info["diffuse_init"]
        outfile        = args.output or f"luminosity_test_results_{args.scenario}.csv"
    else:
        target_lux     = args.target_lux or 1.0
        scenario_label = f"custom_{target_lux}lx"
        diffuse_init   = 0.04
        outfile        = args.output or f"luminosity_test_results_custom.csv"

    rclpy.init()
    node = LightCalibration(
        target_lux=target_lux,
        scenario_label=scenario_label,
        current_diffuse=diffuse_init,
        duration=args.duration,
        output=outfile)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.file.closed:
            node.file.close()
        node.destroy_node()


if __name__ == '__main__':
    main()
