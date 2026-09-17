"""Scene composer: registry robot + scenario assets -> one MJCF file per session.

Text-level composition (no MjSpec dependency): the robot's Menagerie file is read, its ``<compiler>``
and ``<keyframe>`` elements are dropped (the composed scene has more DoF than the robot alone, so the
robot's keyframes would not fit; the home pose comes from the registry), every mesh/texture ``file``
attribute is rewritten to an absolute path, and the tool-centre-point site, wrist camera and, for arms
without a hand, the built-in gripper are injected into the tool body.  Scenario assets (trays, blocks,
CNC, humanoid, zones, scanner fields) are appended to the world body.
"""
from __future__ import annotations

import re
from pathlib import Path

from backend.app.config import DATA_DIR
from backend.conf.machines import get_machine
from backend.conf.registry import ASSETS, RobotProfile

DATA_SCENES = DATA_DIR / "scenes"


def _absolutise_files(xml: str, base: Path) -> str:
    def repl(m):
        f = m.group(1)
        return f'file="{(base / f).resolve()}"' if not f.startswith("/") else m.group(0)
    return re.sub(r'file="([^"]+)"', repl, xml)


def _strip(xml: str) -> str:
    xml = re.sub(r"<compiler[^>]*/>", "", xml)
    xml = re.sub(r"<keyframe>.*?</keyframe>", "", xml, flags=re.S)
    xml = re.sub(r"<mujoco[^>]*>", "", xml, count=1)
    xml = xml.replace("</mujoco>", "")
    return xml


def _extract_balanced(xml: str, tag: str) -> tuple[list[str], str]:
    """Cut every top-level <tag ...>...</tag> block out of xml, honouring nested same-name tags
    (MuJoCo <default> blocks nest)."""
    open_re = re.compile(rf"<{tag}(?:\s[^>]*)?>")
    close_re = re.compile(rf"</{tag}>")
    out, pos, kept = [], 0, []
    while True:
        m = open_re.search(xml, pos)
        if not m:
            kept.append(xml[pos:]); break
        kept.append(xml[pos:m.start()])
        depth, i = 0, m.start()
        while True:
            mo, mc = open_re.search(xml, i), close_re.search(xml, i)
            if mc is None:
                raise ValueError(f"unbalanced <{tag}> in robot MJCF")
            if mo is not None and mo.start() < mc.start():
                depth += 1; i = mo.end()
            else:
                depth -= 1; i = mc.end()
                if depth == 0:
                    break
        out.append(xml[m.start():i])
        pos = i
    return out, "".join(kept)


def _q(v) -> str:
    return " ".join(f"{float(x):g}" for x in v)


def _robot_block(robot: RobotProfile) -> str:
    src = robot.mjcf_path
    xml = _strip(src.read_text())
    meshdir = re.search(r'meshdir="([^"]+)"', src.read_text())
    base = src.parent / (meshdir.group(1) if meshdir else "assets")
    xml = _absolutise_files(xml, base)
    # tool injections into the ee body
    if robot.gripper.get("type") == "builtin":
        g = (ASSETS / "gripper_builtin.xml").read_text().replace("__POS__", _q(robot.ee_offset)).replace("__QUAT__", _q(robot.ee_quat))
        inject = g
    else:
        cam = robot.wrist_camera
        inject = (f'<site name="ee_site" pos="{_q(robot.ee_offset)}" size="0.004" rgba="0 0 0 0"/>\n'
                  f'<camera name="wrist_cam" pos="{_q(cam["pos"])}" quat="{_q(cam["quat"])}" fovy="{cam.get("fovy", 75)}"/>')
    pat = re.compile(rf'(<body name="{re.escape(robot.ee_body)}"[^>]*>)')
    if not pat.search(xml):
        raise ValueError(f"tool body '{robot.ee_body}' not found in {src}")
    xml = pat.sub(lambda m: m.group(1) + "\n" + inject, xml, count=1)
    # gripper actuator/tendon for builtin grippers
    extra = ""
    if robot.gripper.get("type") == "builtin":
        extra = '''
  <tendon><fixed name="tron_split"><joint joint="tron_finger_joint1" coef="0.5"/><joint joint="tron_finger_joint2" coef="0.5"/></fixed></tendon>
  <equality><joint joint1="tron_finger_joint1" joint2="tron_finger_joint2" polycoef="0 1 0 0 0" solimp="0.95 0.99 0.001" solref="0.005 1"/></equality>
  <actuator><general name="tron_gripper" tendon="tron_split" forcerange="-100 100" ctrlrange="0 255" gainprm="0.01568627451 0 0" biasprm="0 -100 -10"/></actuator>'''
    return xml + extra


def _floor_markings(scenario, robot, floor_z: float) -> str:
    """Painted yellow boundary around the work cell, as a real shop floor is marked.

    The rectangle encloses everything inside the cell -- the machine's footprint and the robot's
    range of motion -- with a walking margin, so the operator aisle outside it is genuinely clear of
    both.  It is derived, not authored, so it follows a machine swap or a different arm.
    """
    x0, x1, y0, y1 = cell_bounds(scenario, robot)
    z = floor_z + 0.004                        # paint sits on the slab
    w = 0.05                                   # 100 mm painted line
    out = []
    for name, cx, cy, sx, sy in (("n", (x0 + x1) / 2, y1, (x1 - x0) / 2 + w, w),
                                 ("s", (x0 + x1) / 2, y0, (x1 - x0) / 2 + w, w),
                                 ("e", x1, (y0 + y1) / 2, w, (y1 - y0) / 2),
                                 ("w", x0, (y0 + y1) / 2, w, (y1 - y0) / 2)):
        out.append(f'<geom name="floor_line_{name}" type="box" size="{sx:g} {sy:g} 0.002" '
                   f'pos="{cx:g} {cy:g} {z:g}" material="floor_line" contype="0" conaffinity="0"/>')
    return "\n".join(out)


def cell_bounds(scenario, robot, pad: float = 0.35) -> tuple[float, float, float, float]:
    """(x0, x1, y0, y1) of the marked work cell: machine footprint + robot envelope + margin."""
    rom = robot.reach_m + next((z.margin_m for z in scenario.zones if z.type == "reach_envelope"), 0.0)
    xs, ys = [-rom, rom], [-rom, rom]
    if scenario.machine:
        prof = get_machine(scenario.machine.ref)
        mp = scenario.machine.pose
        xs += [mp[0] + prof.front_offset, mp[0] - prof.front_offset]
        ys += [mp[1] - prof.width / 2, mp[1] + prof.width / 2]
    return min(xs) - pad, max(xs) + pad, min(ys) - pad, max(ys) + pad


def _scenario_bodies(scenario, robot, floor_z: float = -0.75) -> str:
    """World bodies for the scenario's props (trays, blocks, CNC, human, zones, scanner)."""
    out = [_floor_markings(scenario, robot, floor_z)]
    for tray in scenario.trays:
        x, y, z = tray.pose
        out.append(f'<body name="{tray.id}" pos="{x} {y} {z}"><geom name="{tray.id}_geom" type="box" size="0.17 0.12 0.01" material="tray_mat"/></body>')
        # a stand from the tray underside down to the slab: a tray never floats
        top = z - 0.01
        h = (top - floor_z) / 2
        if h > 0.01:
            out.append(f'<geom name="{tray.id}_stand" type="box" size="0.15 0.10 {h:g}" '
                       f'pos="{x} {y} {floor_z + h:g}" material="haas_base"/>')
    for b in scenario.blocks:
        x, y, z = b.pose
        mat = "block_finished" if b.finished else "block_raw"
        # a cylinder reads as a turned part, so a finished piece is recognisable at a glance
        size = f"{b.size} {b.height if b.height is not None else b.size}" if b.shape == "cylinder" \
            else f"{b.size} {b.size} {b.size}"
        out.append(f'<body name="{b.id}" pos="{x} {y} {z}"><freejoint name="{b.id}_free"/>'
                   f'<geom name="{b.id}_geom" type="{b.shape}" size="{size}" material="{mat}" mass="{b.mass}" friction="1.2 0.02 0.001" contype="3" conaffinity="3"/></body>')
    if scenario.machine:
        x, y, z = scenario.machine.pose
        prof = get_machine(scenario.machine.ref)
        out.append(prof.mjcf_path.read_text().replace("__POS__", f"{x} {y} {z}")
                   .replace("__SHELL_GEOMS__", prof.shell_geoms()))
    if scenario.human:
        x, y, z = scenario.human.pose
        out.append((ASSETS / "humanoid_operator.xml").read_text().replace("__POS__", f"{x} {y} {z}"))
    for zone in scenario.zones:
        if zone.type == "reach_envelope":
            # the volume the arm can physically sweep: a cylinder centred on the robot base at the origin
            r = zone.radius or (robot.reach_m + zone.margin_m)
            h = zone.height or r
            out.append(f'<geom name="zone_{zone.id}" type="cylinder" size="{r:g} {h / 2:g}" pos="0 0 {h / 2:g}" '
                       f'material="zone_reach" contype="0" conaffinity="0"/>')
            continue
        lo, hi = zone.min, zone.max
        if not lo and scenario.machine:      # bounds omitted -> the machine's working volume
            lo, hi = get_machine(scenario.machine.ref).interior_bounds(scenario.machine.pose)
        if not lo:
            continue
        c = [(lo[i] + hi[i]) / 2 for i in range(3)]
        sz = [(hi[i] - lo[i]) / 2 for i in range(3)]
        mat = "zone_mat" if zone.type.startswith("restricted") else "zone_speed" if zone.type == "reduced_speed" else "scanner_mat"
        out.append(f'<geom name="zone_{zone.id}" type="box" size="{_q(sz)}" pos="{_q(c)}" material="{mat}" contype="0" conaffinity="0"/>')
    for sc in scenario.sensors:
        if sc.type == "area_scanner":
            lo, hi = sc.field_min, sc.field_max
            c = [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, floor_z + 0.006]
            s = [(hi[0] - lo[0]) / 2, (hi[1] - lo[1]) / 2, 0.004]
            out.append(f'<geom name="scanner_{sc.id}" type="box" size="{_q(s)}" pos="{_q(c)}" material="scanner_mat" contype="0" conaffinity="0"/>')
    return "\n".join(out)


def _grasp_welds(robot: RobotProfile, scenario) -> str:
    """One inactive weld per block between the tool body and the block; the adapter activates it on grasp."""
    tool = "tron_gripper" if robot.gripper.get("type") == "builtin" else robot.ee_body
    if not scenario.blocks:
        return ""
    welds = "\n".join(f'    <weld name="grasp_{b.id}" body1="{tool}" body2="{b.id}" active="false" solref="0.004 1"/>' for b in scenario.blocks)
    return f"  <equality>\n{welds}\n  </equality>"


def compose(robot: RobotProfile, scenario, session_id: str) -> Path:
    tpl = (ASSETS / "scene_template.xml").read_text()
    ped = robot.pedestal_height
    body = _scenario_bodies(scenario, robot, floor_z=-ped)
    # a machine built from CAD ships meshes; they must be declared in the top-level <asset> block
    machine_assets = get_machine(scenario.machine.ref).mesh_assets() if scenario.machine else ""
    scene = (tpl.replace("__PEDESTAL_HALF__", f"{ped / 2:g}").replace("__PEDESTAL_FOOT__", f"{ped - 0.03:g}")
             .replace("__PEDESTAL__", f"{ped:g}")
             .replace("__MACHINE_ASSETS__", machine_assets)
             .replace("__SCENE_BODIES__", body))
    robot_xml = _robot_block(robot)
    # robot goes inside the worldbody; its <asset>/<default>/<actuator>/... sections must be top-level
    sections = {}
    for tag in ("asset", "default", "actuator", "tendon", "equality", "contact", "sensor"):
        parts, robot_xml = _extract_balanced(robot_xml, tag)
        sections[tag] = "\n".join(parts)
    wb = re.search(r"<worldbody>(.*?)</worldbody>", robot_xml, flags=re.S)
    robot_world = wb.group(1) if wb else ""
    leftovers = re.sub(r"<worldbody>.*?</worldbody>", "", robot_xml, flags=re.S).strip()
    scene = scene.replace("<worldbody>", "<worldbody>\n" + robot_world, 1)
    top = "\n".join(v for v in sections.values() if v) + "\n" + leftovers + "\n" + _grasp_welds(robot, scenario)
    scene = scene.replace("</mujoco>", top + "\n</mujoco>")
    DATA_SCENES.mkdir(parents=True, exist_ok=True)
    out = DATA_SCENES / f"{session_id}.xml"
    out.write_text(scene)
    return out
