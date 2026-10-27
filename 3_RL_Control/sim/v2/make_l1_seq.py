"""Refined car run -> replay sequence for env_v2 (scenario l1_replay) and the Pi runtime replay.

    python sim/v2/make_l1_seq.py [--run refined_drive/01_refined/run2] [--out sim/v2/l1_refined_seq.csv]

Columns (beamng/convert.py format): t [s], ax, ay [m/s^2, vehicle frame +x fwd, +y LEFT], yaw [rad/s],
v [m/s, GPS], sw [rad, OBD steering wheel]. Residual static a_x/a_y bias (mount + slope) removed
using the standstill samples (v < 0.3 m/s).
"""
import argparse, os
import numpy as np, pandas as pd

H = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.join(H, '..', '..')
ap = argparse.ArgumentParser()
ap.add_argument('--run', default=os.path.join(ROOT, 'refined_drive', '01_refined', 'run2'))
ap.add_argument('--out', default=os.path.join(H, 'l1_refined_seq.csv'))
a = ap.parse_args()
d = pd.read_csv(os.path.join(a.run, 'synced_10hz.csv'))
v = (d.gps_speed_kmh / 3.6).interpolate().bfill()
out = pd.DataFrame({'t': d.t_run_s - d.t_run_s.iloc[0], 'ax': d.imu_veh_acc_x_mps2.interpolate().bfill(),
                    'ay': d.imu_veh_acc_y_mps2.interpolate().bfill(), 'yaw': d.imu_veh_gyro_z_radps.interpolate().bfill(),
                    'v': v, 'sw': np.deg2rad(d.obd_steering_angle_deg.interpolate().bfill())})
st = v < 0.3
for c in ('ax', 'ay'): out[c] -= out[c][st].mean()
out.to_csv(a.out, index=False); print(out.shape, '->', a.out)
