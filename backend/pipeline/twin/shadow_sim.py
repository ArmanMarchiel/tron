"""Shadow simulation: physics-based expected state.

A second MuJoCo instance of the same composed scene predicts the joint torques the *expected* motion
should require (inverse dynamics of the expected position, velocity and acceleration, including gravity
and the tool's mass).  ``divergence.torque_residual`` = observed effort - predicted effort; a sustained
residual is an external load or contact the software does not know about (UNEXPECTED_LOAD).  Works from
reported effort alone, so it also applies to replayed field logs without any contact sensor.
"""
from __future__ import annotations

import threading

import mujoco
import numpy as np


class ShadowSim:
    def __init__(self, scene_path, robot):
        self.m = mujoco.MjModel.from_xml_path(str(scene_path))
        # expected torques exclude contact: an unexpected contact is exactly what the residual should reveal
        self.m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
        self.d = mujoco.MjData(self.m)
        self.qadr = [self.m.jnt_qposadr[self.m.joint(j).id] for j in robot.joints]
        self.dadr = [self.m.jnt_dofadr[self.m.joint(j).id] for j in robot.joints]
        self.n = robot.n
        self._lock = threading.Lock()
        self.enabled = True

    def predict(self, twin: dict) -> list[float] | None:
        e = twin["expected"]
        if not self.enabled or not e.get("valid") or not e.get("position"):
            return None
        q, qd, qdd = e["position"], e.get("velocity") or [0.0] * self.n, e.get("acceleration") or [0.0] * self.n
        with self._lock:
            self.d.qvel[:] = 0.0
            self.d.qacc[:] = 0.0
            for i, (a, da) in enumerate(zip(self.qadr, self.dadr)):
                self.d.qpos[a] = q[i]
                self.d.qvel[da] = qd[i]
                self.d.qacc[da] = qdd[i]
            mujoco.mj_inverse(self.m, self.d)
            tau = [float(self.d.qfrc_inverse[da]) for da in self.dadr]
        return [round(t, 3) for t in tau]
