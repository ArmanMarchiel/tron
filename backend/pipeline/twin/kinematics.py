"""Forward kinematics for the platform side (expected / believed end-effector position).

A robot description is needed to turn joint angles into an end-effector position on *any*
robot, real or simulated; here the MuJoCo model is used purely as a kinematics library.
Nothing in the simulator's dynamic state is read.  If MuJoCo is unavailable, FK returns
None and joint-space comparison still works.
"""
from __future__ import annotations

import threading

from backend.app.config import JOINTS, SIM

_lock = threading.Lock()
_model = None
_data = None
_qadr: list[int] = []
_site = None
_failed = False
_scene: str = SIM["scene"]
_joints: list[str] = list(JOINTS)


def configure(scene_path, joints: list[str]) -> None:
    """Point FK at the session's composed scene and joint list."""
    global _model, _data, _qadr, _site, _failed, _scene, _joints
    with _lock:
        _scene, _joints = str(scene_path), list(joints)
        _model = _data = None
        _failed = False


def _load() -> bool:
    global _model, _data, _qadr, _site, _failed
    if _model is not None:
        return True
    if _failed:
        return False
    try:
        import mujoco
        _model = mujoco.MjModel.from_xml_path(_scene)
        _data = mujoco.MjData(_model)
        _qadr = [_model.jnt_qposadr[_model.joint(j).id] for j in _joints]
        _site = _model.site("ee_site").id
        return True
    except Exception:
        _failed = True
        return False


def fk(q: list[float] | None) -> dict | None:
    """End-effector position {x,y,z} for joint angles q (7)."""
    if q is None or len(q) < len(_joints) or not _load():
        return None
    import mujoco
    with _lock:
        for a, v in zip(_qadr, q):
            _data.qpos[a] = v
        mujoco.mj_kinematics(_model, _data)
        p = _data.site_xpos[_site]
        return {"x": round(float(p[0]), 4), "y": round(float(p[1]), 4), "z": round(float(p[2]), 4)}
