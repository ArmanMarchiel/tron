"""Damped-least-squares IK on the composed MuJoCo model, used to bake joint-space targets for a scenario.

Targets are end-effector positions with the tool pointing straight down (a fixed target rotation), which
is what pick/place and machine loading need.  Results are cached per (robot, scenario) under backend/data/.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

from backend.app.config import DATA_DIR

DATA_WP = DATA_DIR / "waypoints"
R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])   # tool z down, x forward


class IKSolver:
    def __init__(self, model: mujoco.MjModel, joints: list[str], home: list[float], site: str = "ee_site"):
        self.m = model
        self.d = mujoco.MjData(model)
        self.qadr = [model.jnt_qposadr[model.joint(j).id] for j in joints]
        self.dadr = [model.jnt_dofadr[model.joint(j).id] for j in joints]
        self.lo = np.array([model.jnt_range[model.joint(j).id][0] for j in joints])
        self.hi = np.array([model.jnt_range[model.joint(j).id][1] for j in joints])
        # unlimited joints (range 0 0) -> wide bounds
        unl = (self.hi - self.lo) < 1e-9
        self.lo[unl], self.hi[unl] = -2 * np.pi, 2 * np.pi
        self.home = np.array(home, dtype=float)
        self.site = model.site(site).id
        self.n = len(joints)

    def _set(self, q):
        for a, v in zip(self.qadr, q):
            self.d.qpos[a] = v
        mujoco.mj_kinematics(self.m, self.d)
        mujoco.mj_comPos(self.m, self.d)

    def fk(self, q) -> np.ndarray:
        self._set(q)
        return self.d.site_xpos[self.site].copy()

    def solve(self, target, q0=None, iters=400, rot_weight=0.5) -> tuple[np.ndarray, float]:
        q = np.array(q0 if q0 is not None else self.home, dtype=float)
        jacp, jacr = np.zeros((3, self.m.nv)), np.zeros((3, self.m.nv))
        err_norm = 1e9
        for _ in range(iters):
            self._set(q)
            err_p = np.asarray(target) - self.d.site_xpos[self.site]
            xmat = self.d.site_xmat[self.site].reshape(3, 3)
            rot_err = 0.5 * sum(np.cross(xmat[:, k], R_DOWN[:, k]) for k in range(3))
            mujoco.mj_jacSite(self.m, self.d, jacp, jacr, self.site)
            J = np.vstack([jacp[:, self.dadr], rot_weight * jacr[:, self.dadr]])
            e = np.concatenate([err_p, rot_weight * rot_err])
            err_norm = float(np.linalg.norm(err_p))
            if err_norm < 1e-4 and np.linalg.norm(rot_err) < 2e-3:
                break
            dq = J.T @ np.linalg.solve(J @ J.T + 0.01 * np.eye(6), e)
            N = np.eye(self.n) - np.linalg.pinv(J) @ J
            dq += 0.1 * (N @ (self.home - q))
            q = np.clip(q + 0.5 * dq, self.lo + 0.03, self.hi - 0.03)
        return q, err_norm


def bake(scene_path: Path, robot, targets: dict[str, list[float]], cache_key: str) -> dict[str, dict]:
    """targets: name -> ee xyz.  Returns name -> {q, ee, err}. Cached by scene+targets hash."""
    DATA_WP.mkdir(parents=True, exist_ok=True)
    h = hashlib.sha1((scene_path.read_text() + json.dumps(targets, sort_keys=True) + json.dumps(robot.home)).encode()).hexdigest()[:12]
    cache = DATA_WP / f"{cache_key}_{h}.json"
    if cache.exists():
        return json.loads(cache.read_text())
    m = mujoco.MjModel.from_xml_path(str(scene_path))
    ik = IKSolver(m, robot.joints, robot.home)
    out: dict[str, dict] = {}
    prev = ik.home
    for name, tgt in targets.items():
        q, err = ik.solve(tgt, q0=prev)
        ee = ik.fk(q)
        out[name] = {"q": [round(float(x), 5) for x in q], "ee": [round(float(x), 4) for x in ee], "err": round(err, 5)}
        prev = q
    out["home"] = {"q": [float(x) for x in robot.home], "ee": [round(float(x), 4) for x in ik.fk(robot.home)], "err": 0.0}
    cache.write_text(json.dumps(out, indent=1))
    return out
