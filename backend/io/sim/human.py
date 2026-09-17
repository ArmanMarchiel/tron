"""Kinematic articulated operator (pelvis root on a mocap body, posed limb joints).

Behaviours
  operator_patrol    walk between patrol points along the operator lane, pause at each end
  operator_intrusion walk into the nearest scanner field toward the cell, reach toward it, walk back
  idle               stand still

Ground truth helpers: ``position()``, ``feet_in_field()`` (any foot inside a scanner field), geom ids
for distance queries.  Joint angles are written every step; velocities zeroed, so physics never moves
the body (its geoms are non-colliding anyway).
"""
from __future__ import annotations

import math

import mujoco
import numpy as np

JOINTS = ["hum_spine", "hum_shoulder_l", "hum_elbow_l", "hum_shoulder_r", "hum_elbow_r",
          "hum_hip_l", "hum_knee_l", "hum_hip_r", "hum_knee_r"]


def _yaw_quat(yaw: float) -> np.ndarray:
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


class HumanOperator:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, spec, scanner_fields: list[tuple[list, list]]):
        self.m, self.d = model, data
        self.id = spec.id
        self.mocap = model.body("human-1").mocapid[0]
        self.body = model.body("human-1").id
        self.qadr = {j: model.jnt_qposadr[model.joint(j).id] for j in JOINTS}
        self.dadr = {j: model.jnt_dofadr[model.joint(j).id] for j in JOINTS}
        self.geoms = [g for g in range(model.ngeom) if model.geom(g).name.startswith("hum_")]
        self.foot_l, self.foot_r = model.geom("hum_foot_l").id, model.geom("hum_foot_r").id
        self.z = spec.pose[2]
        self.x, self.y = spec.pose[0], spec.pose[1]
        self.yaw = 0.0
        self.speed = spec.speed
        self.patrol = [list(p) for p in (spec.patrol or [[spec.pose[0], spec.pose[1]]])]
        self.fields = scanner_fields
        self.behaviour = spec.behaviour
        self._wp = 1 % max(1, len(self.patrol))
        self._pause_until = 0.0
        self._phase = 0.0
        self._intrusion_stage = 0
        self._stage_t = 0.0
        self._intrusion_target = self._pick_intrusion_target()
        self._return_to = [self.x, self.y]
        self._reach = 0.0
        self._pose_joints(0.0, 0.0)

    # ------------------------------------------------------------------ behaviours
    def set_behaviour(self, name: str, now: float = 0.0) -> None:
        if name == self.behaviour:
            return
        if name == "operator_intrusion":
            self._return_to = [self.x, self.y]
            self._intrusion_stage, self._stage_t = 0, now
        self.behaviour = name

    def _pick_intrusion_target(self) -> list[float]:
        if not self.fields:
            return [0.6, 0.8]
        lo, hi = self.fields[0]
        return [(lo[0] + hi[0]) / 2, lo[1] + 0.12]   # inside the field, on the cell side

    def _walk_toward(self, tx: float, ty: float, dt: float) -> bool:
        dx, dy = tx - self.x, ty - self.y
        dist = math.hypot(dx, dy)
        if dist < 0.02:
            return True
        step = min(dist, self.speed * dt)
        self.x += dx / dist * step
        self.y += dy / dist * step
        self.yaw = math.atan2(dy, dx)
        self._phase += step * 6.0     # gait cycle
        return False

    def step(self, dt: float, now: float) -> None:
        walking = False
        if self.behaviour == "operator_patrol" and len(self.patrol) > 1:
            if now >= self._pause_until:
                tx, ty = self.patrol[self._wp]
                if self._walk_toward(tx, ty, dt):
                    self._wp = (self._wp + 1) % len(self.patrol)
                    self._pause_until = now + 2.5
                else:
                    walking = True
            else:
                # face the cell while paused
                self.yaw = -math.pi / 2 if self.y > 0 else math.pi / 2
        elif self.behaviour == "operator_intrusion":
            if self._intrusion_stage == 0:
                walking = not self._walk_toward(*self._intrusion_target, dt)
                if not walking:
                    self._intrusion_stage, self._stage_t = 1, now
                    self.yaw = -math.pi / 2 if self.y > 0 else math.pi / 2
            elif self._intrusion_stage == 1:
                self._reach = min(1.0, self._reach + dt * 2.0)
                if now - self._stage_t > 5.0:
                    self._intrusion_stage = 2
            else:
                self._reach = max(0.0, self._reach - dt * 2.0)
                walking = not self._walk_toward(*self._return_to, dt)
                if not walking and self._reach <= 0.0:
                    self.behaviour = "operator_patrol"
                    self._pause_until = now + 1.0
        # write pose
        self.d.mocap_pos[self.mocap] = [self.x, self.y, self.z]
        self.d.mocap_quat[self.mocap] = _yaw_quat(self.yaw)
        self._pose_joints(self._phase if walking else 0.0, self._reach)

    def _pose_joints(self, phase: float, reach: float) -> None:
        swing = 0.45 * math.sin(phase) if phase else 0.0
        knee = 0.6 * max(0.0, math.sin(phase + math.pi / 2)) if phase else 0.0
        q = {
            "hum_spine": 0.05 + 0.25 * reach,
            "hum_hip_l": swing, "hum_hip_r": -swing,
            "hum_knee_l": knee, "hum_knee_r": 0.6 * max(0.0, math.sin(phase - math.pi / 2)) if phase else 0.0,
            "hum_shoulder_l": -0.5 * swing + 0.2, "hum_elbow_l": -0.4,
            "hum_shoulder_r": 0.5 * swing + 0.2 + 1.5 * reach, "hum_elbow_r": -0.4 - 0.4 * reach,
        }
        for j, v in q.items():
            self.d.qpos[self.qadr[j]] = v
            self.d.qvel[self.dadr[j]] = 0.0

    # ------------------------------------------------------------------ ground truth
    def position(self) -> list[float]:
        return [round(self.x, 3), round(self.y, 3), round(self.z, 3)]

    def feet_in_field(self) -> bool:
        for g in (self.foot_l, self.foot_r):
            p = self.d.geom_xpos[g]
            for lo, hi in self.fields:
                if lo[0] <= p[0] <= hi[0] and lo[1] <= p[1] <= hi[1]:   # floor scanner: xy only
                    return True
        return False
