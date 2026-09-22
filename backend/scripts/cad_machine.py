#!/usr/bin/env python3
"""Turn a CAD machine (STEP) into a coloured MuJoCo asset set.

``cad_import.py`` produces a single untextured mesh, which is enough to check dimensions but renders
as a flat grey block.  A STEP file from SolidWorks carries a colour per face, and OpenCascade batches
those into one mesh per colour, so this keeps that grouping: each colour becomes its own MuJoCo mesh
plus a matching material.  The machine then looks like the CAD -- which matters when the renders are
training data for a vision model, not just a diagram.

    python backend/scripts/cad_machine.py backend/cad/haas-st10/ST-10.STEP --name haas_st10

Output lands in ``backend/io/sim/assets/<name>/``: one ``.msh`` per colour group plus a
``materials.xml`` fragment naming them.  Meshes are decimated to a face budget shared out in
proportion to each group's size, so the detail goes where the geometry is.  MuJoCo's own limit is
200k faces per mesh; the default budget sits well under it for the whole machine.

Orientation: SolidWorks is Y-up and MuJoCo is Z-up, so the model is rotated unless ``--no-rotate``.
The result is centred in plan with its base at z=0, which is what the machine XMLs expect.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ASSETS = Path(__file__).resolve().parent.parent / "io" / "sim" / "assets"


def _up_axis(mesh) -> int:
    """Which axis the machine stands on, found from its own geometry.

    Caution: this is sensitive to tessellation.  Re-importing the VF-2 at ``--tol 0.020`` instead of
    0.004 flipped the machine 180 degrees about z -- same bounding box, mirrored contents -- because
    the coarser mesh shifts which faces clear the flatness and height tests below.  Changing ``--tol``
    on a machine that already looks right means re-checking its orientation, and pinning ``--yaw``.

    A machine tool rests on a base plate: a large flat surface at the bottom of the model.  For each
    axis, sum the area of faces whose normal points along it *and* that sit within 120 mm of that
    axis' minimum.  The winner is the base, so that axis is "up".  This beats guessing a rotation,
    because CAD assemblies do not agree on which way is up.
    """
    import numpy as np

    tri = mesh.vertices[mesh.faces]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    area = np.linalg.norm(n, axis=1) / 2
    keep = area > 1e-5
    if not keep.any():
        return 2
    normals = n[keep] / np.linalg.norm(n[keep], axis=1, keepdims=True)
    area, centre = area[keep], tri[keep].mean(axis=1)
    scores = []
    for ax in range(3):
        flat = np.abs(normals[:, ax]) > 0.98
        if not flat.any():
            scores.append(0.0)
            continue
        near_min = (centre[flat][:, ax] - mesh.vertices[:, ax].min()) < 0.12
        scores.append(float(area[flat][near_min].sum()))
    return int(np.argmax(scores))


def convert(src: Path, out_dir: Path, name: str, tol: float, budget: int, rotate: bool, yaw: float) -> dict:
    try:
        import cascadio
        import fast_simplification
        import numpy as np
        import trimesh
    except ImportError:
        sys.exit("needs cascadio, trimesh, fast-simplification:  .venv/bin/pip install -r requirements-cad.txt")

    out_dir.mkdir(parents=True, exist_ok=True)
    glb = out_dir / f"{name}.glb"
    cascadio.step_to_glb(str(src), str(glb), tol_linear=tol, include_materials=True)
    scene = trimesh.load(glb)
    glb.unlink(missing_ok=True)

    # merge everything that shares a colour: a machine with 36 near-identical greys does not need
    # 36 MuJoCo geoms, and one mesh per distinct colour is what actually drives the look
    # A glTF scene places each part with a node transform.  Reading scene.geometry alone returns the
    # meshes in their own local frames, so concatenating them drops that placement and the machine
    # comes apart -- every panel at the origin's orientation.  Walk the scene graph instead and bake
    # each node's transform into its copy of the mesh.
    by_colour: dict[tuple, list] = {}
    if hasattr(scene, "graph") and hasattr(scene, "geometry"):
        for node in scene.graph.nodes_geometry:
            transform, geom_name = scene.graph[node]
            mesh = scene.geometry[geom_name].copy()
            mesh.apply_transform(transform)
            mat = getattr(mesh.visual, "material", None)
            rgba = tuple(int(c) for c in getattr(mat, "baseColorFactor", (200, 200, 200, 255)))
            by_colour.setdefault(rgba, []).append(mesh)
    else:
        mesh = scene
        mat = getattr(mesh.visual, "material", None)
        rgba = tuple(int(c) for c in getattr(mat, "baseColorFactor", (200, 200, 200, 255)))
        by_colour[rgba] = [mesh]
    groups = {}
    for rgba, meshes in by_colour.items():
        merged = trimesh.util.concatenate(meshes) if len(meshes) > 1 else meshes[0]
        merged._tron_rgba = [round(c / 255.0, 4) for c in rgba]
        groups[f"c{len(groups)}"] = merged
    total_faces = sum(len(g.faces) for g in groups.values())
    print(f"  {len(groups)} colour groups, {total_faces:,} faces -> budget {budget:,}")

    # one combined mesh first, to get the machine's overall frame, then place every group in it
    # SolidWorks is Y-up; MuJoCo is Z-up. The yaw then turns the machine's front (its door side)
    # toward -x, which is where the robot stands in the cell.
    # Stand the machine on the axis its own base plate sits on, rather than assuming the source
    # used Y-up: the three Haas assemblies disagree about which axis is vertical.
    probe = trimesh.util.concatenate(list(groups.values()))
    up = _up_axis(probe) if rotate else 2
    if up == 0:                         # x is up -> bring it to z
        xf = trimesh.transformations.rotation_matrix(-np.pi / 2, [0, 1, 0])
    elif up == 1:                       # y is up (the SolidWorks default)
        xf = trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0])
    else:
        xf = np.eye(4)
    print(f"  base plate found on the {'xyz'[up]} axis -> standing the machine on z")
    if yaw:
        xf = trimesh.transformations.rotation_matrix(np.radians(yaw), [0, 0, 1]) @ xf
    combined = trimesh.util.concatenate(list(groups.values()))
    combined.apply_transform(xf)
    lo, hi = combined.bounds
    shift = np.array([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from obj_to_msh import write_msh

    out: dict[str, dict] = {}
    for i, (key, mesh) in enumerate(sorted(groups.items())):
        mesh = mesh.copy()
        mesh.apply_transform(xf)
        mesh.apply_translation(shift)

        # Decimation is off unless a budget is given.  These colour groups are collections of
        # disconnected shells, and simplifying them collapses thin sheet metal into fragments --
        # the machine visibly shatters.  Tessellation tolerance is the right dial for size here.
        share = max(400, int(budget * len(mesh.faces) / total_faces)) if budget else len(mesh.faces)
        if budget and len(mesh.faces) > share:
            v, f = fast_simplification.simplify(np.asarray(mesh.vertices, np.float32),
                                                np.asarray(mesh.faces, np.int32),
                                                target_reduction=1.0 - share / len(mesh.faces))
            mesh = trimesh.Trimesh(v, f, process=False)

        rgba = getattr(groups[key], "_tron_rgba", [0.78, 0.79, 0.80, 1.0])

        part = f"{name}_{i}"
        path = out_dir / f"{part}.msh"
        write_msh(path, np.asarray(mesh.vertices, np.float32),
                  np.zeros((0, 3), np.float32), np.asarray(mesh.faces, np.int32))
        out[part] = {"file": f"{out_dir.name}/{path.name}", "rgba": rgba, "faces": len(mesh.faces)}
        print(f"    {part:16s} {len(mesh.faces):7,} faces  rgba {rgba}  {path.stat().st_size / 1e6:.1f} MB")

    ext = combined.extents
    print(f"  extents {ext[0]:.3f} x {ext[1]:.3f} x {ext[2]:.3f} m")
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("source", type=Path)
    ap.add_argument("--name", required=True, help="asset basename, e.g. haas_st10")
    ap.add_argument("--tol", type=float, default=0.004, help="tessellation deflection in metres")
    ap.add_argument("--budget", type=int, default=0,
                help="total face budget; 0 (default) keeps full detail -- decimation fragments these meshes")
    ap.add_argument("--no-rotate", action="store_true", help="skip the SolidWorks Y-up -> MuJoCo Z-up rotation")
    ap.add_argument("--yaw", type=float, default=0.0,
                    help="degrees about z after the Z-up rotation, to face the door toward -x")
    args = ap.parse_args()

    if not args.source.exists():
        sys.exit(f"no such file: {args.source}")
    out_dir = ASSETS / args.name
    print(f"{args.source.name} -> {out_dir}")
    parts = convert(args.source, out_dir, args.name, args.tol, args.budget, not args.no_rotate, args.yaw)

    print("\nregistry `meshes:` block —")
    print("    meshes: {" + ", ".join(f"{k}: {v['file']}" for k, v in parts.items()) + "}")
    print("\nmaterials for scene_template.xml —")
    for k, v in parts.items():
        r, g, b, a = v["rgba"]
        print(f'    <material name="{k}_mat" rgba="{r} {g} {b} {a}"/>')
    print("\ngeoms for the machine XML —")
    for k in parts:
        print(f'  <geom name="cnc_{k}" type="mesh" mesh="{k}" material="{k}_mat" contype="0" conaffinity="0"/>')
    return 0


if __name__ == "__main__":
    sys.exit(main())
