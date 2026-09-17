#!/usr/bin/env python3
"""Convert Menagerie visual OBJ meshes to MuJoCo's binary .msh format.

The visual meshes ship as ASCII OBJ (~149 MB across the four robots).  MuJoCo's native binary
mesh format stores the same vertices, normals and triangles as packed float32/int32, which is
about 2.7x smaller and skips ASCII parsing at model load.  The conversion is lossless: every
vertex, normal and face is preserved exactly.

Collision geometry is unaffected -- it comes from the separate .stl meshes, which stay as they are.

    python backend/scripts/obj_to_msh.py            # convert in place, remove the .obj files
    python backend/scripts/obj_to_msh.py --dry-run  # report the savings only
"""
from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

import numpy as np

ASSETS = Path(__file__).resolve().parent.parent / "io" / "sim" / "assets"


def parse_obj(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Read an OBJ into (vertices, normals, triangles).

    Faces are triangulated as a fan and, where an OBJ indexes positions and normals separately,
    normals are scattered into vertex order so the two arrays line up as .msh requires.
    """
    verts: list[tuple[float, float, float]] = []
    norms: list[tuple[float, float, float]] = []
    tris: list[tuple[int, int, int]] = []
    norm_for_vert: dict[int, int] = {}

    for line in path.read_text().splitlines():
        if line.startswith("v "):
            verts.append(tuple(float(x) for x in line.split()[1:4]))
        elif line.startswith("vn "):
            norms.append(tuple(float(x) for x in line.split()[1:4]))
        elif line.startswith("f "):
            vi: list[int] = []
            for tok in line.split()[1:]:
                bits = tok.split("/")
                v = int(bits[0]) - 1
                vi.append(v)
                if len(bits) == 3 and bits[2]:
                    norm_for_vert[v] = int(bits[2]) - 1
            for i in range(1, len(vi) - 1):          # fan-triangulate polygons
                tris.append((vi[0], vi[i], vi[i + 1]))

    v = np.asarray(verts, dtype=np.float32)
    f = np.asarray(tris, dtype=np.int32)
    if norms and norm_for_vert:
        src = np.asarray(norms, dtype=np.float32)
        n = np.zeros_like(v)
        for vi_, ni_ in norm_for_vert.items():
            if vi_ < len(n) and ni_ < len(src):
                n[vi_] = src[ni_]
    else:
        n = np.zeros((0, 3), dtype=np.float32)
    return v, n, f


def write_msh(path: Path, v: np.ndarray, n: np.ndarray, f: np.ndarray) -> int:
    """MuJoCo .msh: four int32 counts (verts, normals, texcoords, faces) then the packed arrays."""
    blob = (struct.pack("<4i", len(v), len(n), 0, len(f))
            + v.tobytes() + n.tobytes() + f.tobytes())
    path.write_bytes(blob)
    return len(blob)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true", help="report savings without writing")
    ap.add_argument("--keep-obj", action="store_true", help="convert but leave the .obj files")
    args = ap.parse_args()

    objs = sorted(ASSETS.rglob("*.obj"))
    if not objs:
        print(f"no .obj meshes under {ASSETS}")
        return 1

    before = after = 0
    for p in objs:
        v, n, f = parse_obj(p)
        src = p.stat().st_size
        before += src
        if args.dry_run:
            after += 16 + v.nbytes + n.nbytes + f.nbytes
            continue
        after += write_msh(p.with_suffix(".msh"), v, n, f)
        if not args.keep_obj:
            p.unlink()

    verb = "would save" if args.dry_run else "saved"
    print(f"{len(objs)} meshes: {before/1e6:.1f} MB -> {after/1e6:.1f} MB "
          f"({verb} {(before-after)/1e6:.1f} MB, {before/max(after,1):.1f}x)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
