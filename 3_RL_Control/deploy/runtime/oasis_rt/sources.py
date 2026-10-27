"""Inputs to the control loop other than the STM32 link: vehicle state and pose.

Every source returns (sample: dict | None, age_s). The controller treats age > its timeout as stale.

  vehicle sample : t, ay_mps2, ax_mps2, yaw_rate_radps, speed_mps, steer_rad  (vehicle frame +x fwd, +y LEFT)
  pose sample    : torso_lean_deg (+ = toward vehicle +y after `lean_sign`), lean_score
                   (shoulder shift / neutral shoulder width, DAQ pose/features.csv), pose_confidence, neutral_valid

  UdpJsonSource   : one JSON object per datagram (DAQ adapter on the Pi, or the BeamNG bridge on the PC)
  ReplayVehicle   : recorded / BeamNG-converted sequence (t, ax, ay, yaw, v, sw) played on the control clock
  NoPose          : no camera -> pose slots hold 0 with age 1.0 s (the policy saw pose dropouts in DR)
"""
import json, socket
import numpy as np


class UdpJsonSource:
    def __init__(self, port, clock, host='127.0.0.1', rename=None):
        self.clock, self.rename = clock, rename or {}
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); self.s.bind((host, port)); self.s.setblocking(False)
        self.last, self.at, self.bad = None, -1e9, 0
    def latest(self):
        while True:
            try:
                b = self.s.recv(4096)
            except (BlockingIOError, OSError):
                break
            try:
                d = json.loads(b)
                self.last = {self.rename.get(k, k): v for k, v in d.items()}; self.at = self.clock()
            except ValueError:
                self.bad += 1
        return self.last, self.clock() - self.at


class ReplayVehicle:
    def __init__(self, path, clock, t0=None, loop=False):
        import pandas as pd
        d = pd.read_csv(path); self.clock, self.loop = clock, loop
        self.t = d.t.values; self.cols = {k: d[k].values for k in ('ax', 'ay', 'yaw', 'v', 'sw')}
        self.t0 = clock() if t0 is None else t0
    def time(self):
        tt = self.clock() - self.t0
        return tt % self.t[-1] if self.loop else tt
    def latest(self):
        tt = self.time()
        if tt > self.t[-1]: return None, 1e9
        f = {k: float(np.interp(tt, self.t, v)) for k, v in self.cols.items()}
        return dict(t=tt, ay_mps2=f['ay'], ax_mps2=f['ax'], yaw_rate_radps=f['yaw'], speed_mps=f['v'], steer_rad=f['sw']), 0.0


class NoPose:
    def latest(self): return None, 1e9
