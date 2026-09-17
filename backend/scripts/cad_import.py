#!/usr/bin/env python3
"""Convert a CAD file (STEP / IGES) into meshes MuJoCo can load.

MuJoCo has no CAD decoder: it reads OBJ, STL and its own binary MSH, and nothing else.  This script
tessellates a B-rep through OpenCascade and writes the pieces the platform actually needs.

    python backend/scripts/cad_import.py machine.step --name haas_st10
    python backend/scripts/cad_import.py machine.step --name haas_st10 --tol 0.002 --split

Two things about MuJoCo meshes decide how a converted part behaves:

*Collision is convex.*  Every mesh geom collides as its convex hull, so a hollow enclosure imported
as one mesh is a solid block the robot can never reach into.  Use the mesh for *visual* geometry and
give the part hand-authored primitive boxes for collision (this is how ``haas_vf2.xml`` is built), or
pass ``--split`` to emit one mesh per CAD solid so each piece hulls separately.

*Tessellation is a trade.*  ``--tol`` is the linear deflection in metres: smaller is rounder and
heavier.  0.002 (2 mm) suits a machine enclosure; drop to 0.0005 for something the robot grasps.

Units: CAD is normally millimetres and OpenCascade converts to metres on the way out, which is what
MuJoCo wants.  The script prints the resulting bounding box -- check it against the real machine
before trusting the model.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ASSETS = Path(__file__).resolve().parent.parent / "io" / "sim" / "assets"


def convert(src: Path, out_dir: Path, name: str, tol: float, split: bool, binary: bool) -> list[Path]:
    try:
        import cascadio
        import trimesh
    except ImportError:
        sys.exit("needs cascadio and trimesh:  .venv/bin/pip install cascadio trimesh")

    glb = out_dir / f"{name}.glb"
    out_dir.mkdir(parents=True, exist_ok=True)
    cascadio.step_to_glb(str(src), str(glb), tol_linear=tol, merge_primitives=not split)

    scene = trimesh.load(glb)
    parts: list[tuple[str, "trimesh.Trimesh"]] = []
    if split and hasattr(scene, "geometry") and len(scene.geometry) > 1:
        for i, (key, geom) in enumerate(sorted(scene.geometry.items())):
            parts.append((f"{name}_{i:02d}", geom))
    else:
        parts.append((name, scene.to_mesh() if hasattr(scene, "to_mesh") else scene))

    written = []
    for part_name, mesh in parts:
        mesh.process(validate=True)               # drop degenerate faces CAD tessellation leaves behind
        path = out_dir / f"{part_name}.stl"
        mesh.export(path)
        written.append(path)
        ext = mesh.extents
        print(f"  {path.name:28s} {len(mesh.vertices):7d} verts {len(mesh.faces):7d} faces  "
              f"{ext[0]:.3f} x {ext[1]:.3f} x {ext[2]:.3f} m  {path.stat().st_size / 1e6:.1f} MB")
    glb.unlink(missing_ok=True)

    if binary:                                     # same 2.9x saving the Menagerie meshes get
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from obj_to_msh import write_msh
        import numpy as np
        for path in list(written):
            mesh = trimesh.load(path)
            write_msh(path.with_suffix(".msh"),
                      np.asarray(mesh.vertices, np.float32),
                      np.zeros((0, 3), np.float32),
                      np.asarray(mesh.faces, np.int32))
    return written


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("source", type=Path, help="a .step / .stp / .iges / .igs file")
    ap.add_argument("--name", help="output basename (default: the source stem)")
    ap.add_argument("--out", type=Path, help=f"output directory (default: {ASSETS})")
    ap.add_argument("--tol", type=float, default=0.002, help="linear deflection in metres (default 0.002)")
    ap.add_argument("--split", action="store_true", help="one mesh per CAD solid, so each hulls separately")
    ap.add_argument("--msh", action="store_true", help="also write MuJoCo binary .msh")
    args = ap.parse_args()

    if not args.source.exists():
        sys.exit(f"no such file: {args.source}")
    if args.source.suffix.lower() not in (".step", ".stp", ".iges", ".igs"):
        sys.exit(f"expected STEP or IGES, got '{args.source.suffix}'")

    name = args.name or args.source.stem.lower().replace(" ", "_")
    out = args.out or (ASSETS / name)
    print(f"{args.source.name} -> {out}  (deflection {args.tol * 1000:g} mm)")
    written = convert(args.source, out, name, args.tol, args.split, args.msh)

    print(f"\n{len(written)} mesh(es) written. Reference one from an MJCF fragment:")
    print(f'  <asset><mesh name="{name}" file="{name}/{written[0].name}"/></asset>')
    print(f'  <geom type="mesh" mesh="{name}" contype="0" conaffinity="0"/>   <!-- visual only -->')
    print("\nCollision is the convex hull of each mesh, so give a hollow part primitive box geoms")
    print("for its collision shape rather than relying on the imported mesh.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
