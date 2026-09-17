"""Bake joint-space waypoints for the pick/place cycle (and the fault-scenario targets) with a
damped-least-squares IK on the MuJoCo model.  Output: backend/sim/assets/waypoints.json.

    .venv/bin/python -m backend.io.sim.bake_waypoints
"""
from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
SCENE = HERE / "assets" / "scene.xml"
OUT = HERE / "assets" / "waypoints.json"

TARGETS = [  # name, ee xyz, gripper (0 closed .. 255 open), duration s
    ("home", None, 255, 4.0),
    ("above_pick", (0.55, -0.35, 0.30), 255, 4.0),
    ("pick", (0.55, -0.35, 0.09), 255, 3.0),
    ("grasp", (0.55, -0.35, 0.09), 0, 1.0),
    ("lift", (0.55, -0.35, 0.42), 0, 3.0),
    ("above_place", (0.55, 0.35, 0.42), 0, 4.0),
    ("place", (0.55, 0.35, 0.09), 0, 3.0),
    ("release", (0.55, 0.35, 0.09), 255, 1.0),
    ("retreat", (0.55, 0.35, 0.42), 255, 3.0),
]
SCENARIO_TARGETS = {
    "rogue": (0.30, 0.72, 0.45),      # scenario B: swing toward the human / restricted zone
    "obstacle": (0.55, 0.00, 0.11),   # scenario E: drive the gripper into obstacle-1
    "obstacle_retreat": (0.55, 0.00, 0.50),  # after E: lift straight up before resuming the cycle
}


def solve_ik(m, d, site, target, q0, hold_rot, iters=300):
    q = q0.copy()
    jacp, jacr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
    for _ in range(iters):
        d.qpos[:7] = q; mujoco.mj_kinematics(m, d); mujoco.mj_comPos(m, d)
        err_p = np.array(target) - d.site(site).xpos
        xmat = d.site(site).xmat.reshape(3, 3)
        rot_err = 0.5 * (np.cross(xmat[:, 0], hold_rot[:, 0]) + np.cross(xmat[:, 1], hold_rot[:, 1]) + np.cross(xmat[:, 2], hold_rot[:, 2]))
        mujoco.mj_jacSite(m, d, jacp, jacr, m.site(site).id)
        J = np.vstack([jacp[:, :7], 0.5 * jacr[:, :7]])
        e = np.concatenate([err_p, 0.5 * rot_err])
        if np.linalg.norm(err_p) < 1e-4 and np.linalg.norm(rot_err) < 1e-3:
            break
        dq = J.T @ np.linalg.solve(J @ J.T + 0.01 * np.eye(6), e)
        # nullspace pull toward q0 keeps the elbow up and away from the table
        N = np.eye(7) - np.linalg.pinv(J) @ J
        dq += 0.1 * (N @ (q0 - q))
        q = np.clip(q + 0.5 * dq, m.jnt_range[:7, 0] + 0.05, m.jnt_range[:7, 1] - 0.05)
    d.qpos[:7] = q; mujoco.mj_kinematics(m, d)
    return q, float(np.linalg.norm(np.array(target) - d.site(site).xpos))


def main():
    m = mujoco.MjModel.from_xml_path(str(SCENE)); d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0); mujoco.mj_kinematics(m, d)
    home = d.qpos[:7].copy(); hold_rot = d.site("ee_site").xmat.reshape(3, 3).copy()
    out, prev = [], home
    for name, tgt, grip, dur in TARGETS:
        if tgt is None:
            q, err = home, 0.0
        else:
            q, err = solve_ik(m, d, "ee_site", tgt, prev, hold_rot)
        d.qpos[:7] = q; mujoco.mj_kinematics(m, d)
        ee = d.site("ee_site").xpos.round(4).tolist()
        print(f"{name:12} err {err:.4f} m  ee {ee}  q {np.round(q, 3).tolist()}")
        out.append({"name": name, "q": [round(float(x), 5) for x in q], "gripper": grip, "duration_s": dur, "ee": ee})
        prev = q
    sc = {}
    for name, tgt in SCENARIO_TARGETS.items():
        q, err = solve_ik(m, d, "ee_site", tgt, home, hold_rot)
        d.qpos[:7] = q; mujoco.mj_kinematics(m, d)
        sc[name] = {"name": name, "q": [round(float(x), 5) for x in q], "gripper": 255, "duration_s": 2.0, "ee": d.site("ee_site").xpos.round(4).tolist()}
        print(f"{name:12} err {err:.4f} m  ee {sc[name]['ee']}")
    # sanity: min EE height along linear joint interpolation of consecutive waypoints
    for a, b in zip(out, out[1:]):
        zs = []
        for s in np.linspace(0, 1, 25):
            d.qpos[:7] = np.array(a["q"]) + (np.array(b["q"]) - np.array(a["q"])) * s; mujoco.mj_kinematics(m, d); zs.append(d.site("ee_site").xpos[2])
        print(f"  {a['name']:>11} -> {b['name']:<11} min EE z {min(zs):.3f}  max joint delta {np.max(np.abs(np.array(b['q']) - np.array(a['q']))):.2f} rad over {b['duration_s']} s")
    OUT.write_text(json.dumps({"cycle": out, "scenarios": sc}, indent=1))
    print("wrote", OUT)


if __name__ == "__main__":
    main()
