"""Part C (Raspberry Pi sensors): reproduce every refined output from 00_raw.

Usage:  python pipeline\\3_pi_gps_imu_camera\\run.py

Writes 01_refined/run2/{gps,imu,pose,pose_landmarks,camera_frames}.csv (+ json
companions), 01_refined/run1/{gps,imu}_fragments.csv and the QC stats/plots in
02_qc/3_pi_gps_imu_camera/.  02_qc/3_pi_gps_imu_camera/report.md is hand-written from those stats.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import gps_imu       # noqa: E402  GPS + IMU (run2) and run-1 fragments
import pose_camera   # noqa: E402  camera frame index + pose landmarks/features

if __name__ == "__main__":
    gps_imu.main()
    pose_camera.main()
    print("Part C done.")
