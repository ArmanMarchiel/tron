#!/usr/bin/env python3
"""Turn a CAD machine (STEP) into a coloured MuJoCo asset set, one solid at a time.

``cad_machine.py`` hands the whole assembly to ``cascadio.step_to_glb``, which tessellates every
solid inside a single OpenCascade call and keeps all of it -- B-rep plus triangulation plus the
glTF buffers -- resident until that call returns.  On the VF-1 (50 MB, 138 solids) that call
reached 5.5 GB of RSS in 24 seconds on an 18 GB machine and then swapped rather than finished:
no output, no progress, no way to interrupt it.

This script reads the same file through OpenCascade directly (``STEPCAFControl_Reader``, the same
library cascadio wraps) and drives the loop itself, which changes the memory profile:

  * the B-rep is loaded once -- 12 s and 0.8 GB for the VF-1, the whole file;
  * each solid is meshed, its triangles copied into numpy, and its triangulation then dropped
    (``BRepTools.Clean``), so peak memory tracks the *largest single solid*, not the assembly;
  * progress prints per solid, and a partially finished run leaves usable meshes behind.

Colour and part names come from the STEP's XDE attributes, so the output matches what
``cad_machine.py`` produced: one ``.msh`` per colour group, one material per group, and a
``meshes:`` block to paste into ``backend/conf/machines/machines.yaml``.

    python backend/scripts/cad_step.py backend/cad/haas-vf1/VF-1_STEP_11_2021.STEP --name haas_vf1

Orientation: the machine is stood on the axis its own base plate sits on (pin it with ``--up`` when
that vote is wrong), centred in plan with its base at z=0, and ``--yaw`` turns the door toward -x
where the robot stands.  For all five Haas files so far that is ``--yaw 270``, with ``--up y`` needed explicitly
on the VF-2 and VF-2TR.

Do not try to verify the yaw by inspecting the geometry or a render.  Vertex density does not
identify the front (the back of these enclosures is just as sparse), and an untextured VMC looks
equally plausible from all four sides, so every such check during this import produced a confident
wrong answer.  Re-import with the rotation the operator asks for and let them look at the cell.

Note ``--tol`` is in the source model's units -- millimetres for these Haas exports -- unlike
cad_machine.py, where it was metres.  See the flag's own comment in ``main``.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ASSETS = Path(__file__).resolve().parent.parent / "io" / "sim" / "assets"

# Solids whose colour differs only in the last bit or two of a channel are the same paint as far as
# a render is concerned; quantising here keeps the group count (and so the MuJoCo geom count) near
# the number of colours a person would actually name in the model.
_COLOUR_STEPS = 64

# Machine-tool grey, for solids no style in the file reaches.  Already sRGB, like MuJoCo wants.
_DEFAULT_RGBA = (0.78, 0.79, 0.80, 1.0)

# MuJoCo rejects a mesh above 200k faces; stay clear of the edge so a group that lands right on it
# still compiles.  Colour usually splits a machine finely enough that this never binds.
_MAX_MESH_FACES = 150_000


def _to_srgb(c: float) -> float:
    """Linear light -> sRGB, the space MuJoCo's ``rgba`` is interpreted in.

    OpenCascade stores ``Quantity_Color`` in linear space and converts on the way in, so a part
    the CAD author picked as sRGB 202,209,238 comes back as 0.591,0.638,0.855.  Writing that
    straight into MuJoCo paints it as sRGB 0.591... -- far too bright, and the error grows toward
    the dark end (0.41 linear is sRGB 0.67).  That is why the whole VF-1 rendered as one pale mass
    with no contrast between its panels, its castings and its base.
    """
    c = min(max(c, 0.0), 1.0)
    return 12.92 * c if c <= 0.0031308 else 1.055 * c ** (1 / 2.4) - 0.055


def _quantise(rgba: tuple[float, ...]) -> tuple[int, ...]:
    return tuple(min(_COLOUR_STEPS - 1, int(c * _COLOUR_STEPS)) for c in rgba)


def _step_colour_table(src: Path) -> dict:
    """Colours parsed straight out of the STEP text, keyed by vertex count and bounding box.

    XDE only reports a colour for a solid when the STEP styles the solid itself.  SolidWorks writes
    either form, and the two Haas files differ: the VF-2 styles all 49 of its MANIFOLD_SOLID_BREPs
    (XDE finds every one), while the VF-1 styles 2758 individual ADVANCED_FACEs and only 122 solids,
    so XDE leaves 107 of its 138 solids grey.  Those face styles are present in the file -- they
    just never reach XDE, because OpenCascade records a transfer result per solid, not per face,
    and the styled face entity has nothing to bind to.

    So read the styles directly.  STYLED_ITEM -> PRESENTATION_STYLE_ASSIGNMENT -> ... -> COLOUR_RGB
    is walked by following entity references until an RGB turns up, which avoids hard-coding the
    half-dozen intermediate entity types.  A solid's colour is its own style if it has one, else the
    majority colour of its faces.

    COLOUR_RGB in the file is already sRGB, which is what MuJoCo wants, so these values are used
    as they are -- unlike the linear ones OpenCascade hands back through XDE (see ``_to_srgb``).

    Matching these back to OpenCascade's solids cannot use entity ids or coordinates: OCC applies
    each component's assembly placement, and normalises this file's inches to millimetres, so
    nothing lines up numerically.  Vertex *count* survives both, so that is the key; the bounding-box
    dimensions (scaled to mm, rounded) disambiguate the 10 counts shared by differently coloured
    solids.  Those are all small fittings -- bolts, brackets -- while the large panels that dominate
    the render have unique counts, so a tie there costs little and never fails loudly.
    """
    import collections
    import re

    txt = src.read_text(errors="replace")
    ent: dict[str, tuple[str, str]] = {}
    for m in re.finditer(r"^#(\d+)\s*=\s*\(?\s*([A-Z_0-9]+)(.*)$", txt, re.M):
        ent[m.group(1)] = (m.group(2), m.group(3))

    def refs(body: str) -> list[str]:
        return re.findall(r"#(\d+)", body)

    def floats(body: str) -> list[float]:
        return [float(x) for x in re.findall(r"-?\d+\.?\d*(?:[eE][-+]?\d+)?", body)]

    pts, rgb = {}, {}
    for eid, (kind, body) in ent.items():
        if kind == "CARTESIAN_POINT":
            n = floats(body)
            if len(n) >= 3:
                pts[eid] = tuple(n[:3])
        elif kind == "COLOUR_RGB":
            n = floats(body)
            if len(n) >= 3:
                rgb[eid] = tuple(round(c, 4) for c in n[-3:])

    def colour_under(eid: str):
        """First COLOUR_RGB reachable from a style entity."""
        seen, stack = set(), [eid]
        while stack:
            cur = stack.pop()
            if cur in seen or cur not in ent:
                continue
            seen.add(cur)
            if cur in rgb:
                return rgb[cur]
            stack.extend(refs(ent[cur][1]))
        return None

    solid_col, face_col = {}, {}
    for m in re.finditer(r"#(\d+)\s*=\s*STYLED_ITEM\s*\(\s*'[^']*'\s*,"
                         r"\s*\(\s*#(\d+)\s*\)\s*,\s*#(\d+)\s*\)", txt):
        style, target = m.group(2), m.group(3)
        kind = ent.get(target, ("", ""))[0]
        col = colour_under(style)
        if col is None:
            continue
        if kind in ("MANIFOLD_SOLID_BREP", "BREP_WITH_VOIDS"):
            solid_col[target] = col
        elif kind == "ADVANCED_FACE":
            face_col[target] = col

    def walk(eid: str):
        """Vertices and faces reachable from a solid."""
        seen, stack, verts, faces = set(), [eid], set(), []
        while stack:
            cur = stack.pop()
            if cur in seen or cur not in ent:
                continue
            seen.add(cur)
            kind, body = ent[cur]
            if kind == "ADVANCED_FACE":
                faces.append(cur)
            if kind == "VERTEX_POINT":
                for r in refs(body):
                    if r in pts:
                        verts.add(tuple(round(c, 6) for c in pts[r]))
                continue
            if kind in ("CARTESIAN_POINT", "DIRECTION", "COLOUR_RGB"):
                continue
            stack.extend(refs(body))
        return verts, faces

    # The file's own length unit vs the millimetres OpenCascade hands back.
    unit = 25.4 if re.search(r"CONVERSION_BASED_UNIT\s*\(\s*'INCH'", txt, re.I) else 1.0

    by_box: dict[tuple, set] = {}
    by_count: dict[int, set] = {}
    for m in re.finditer(r"^#(\d+)\s*=\s*(?:MANIFOLD_SOLID_BREP|BREP_WITH_VOIDS)", txt, re.M):
        sid = m.group(1)
        verts, faces = walk(sid)
        if not verts:
            continue
        col = solid_col.get(sid)
        if col is None:
            seen_cols = [face_col[f] for f in faces if f in face_col]
            if seen_cols:
                col = collections.Counter(seen_cols).most_common(1)[0][0]
        if col is None:
            continue
        xs, ys, zs = zip(*verts)
        dims = tuple(sorted(round((hi - lo) * unit) for lo, hi in
                            ((min(xs), max(xs)), (min(ys), max(ys)), (min(zs), max(zs)))))
        by_box.setdefault((len(verts), dims), set()).add(col)
        by_count.setdefault(len(verts), set()).add(col)
    return {"by_box": by_box, "by_count": by_count}


def _table_colour(table: dict, verts: set):
    """The STEP colour for an OpenCascade solid, by vertex count then bounding box."""
    if not table or not verts:
        return None
    xs, ys, zs = zip(*verts)
    dims = tuple(sorted(round(hi - lo) for lo, hi in
                        ((min(xs), max(xs)), (min(ys), max(ys)), (min(zs), max(zs)))))
    cand = table["by_count"].get(len(verts))
    if cand and len(cand) > 1:                     # count is shared: try the box too
        cand = table["by_box"].get((len(verts), dims)) or cand
    if cand:
        return sorted(cand)[0] if len(cand) > 1 else next(iter(cand))
    return None


def _solid_vertices(solid) -> set:
    """Distinct vertex coordinates of a solid, for matching against the STEP table."""
    from OCP.BRep import BRep_Tool
    from OCP.TopAbs import TopAbs_ShapeEnum
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS

    out = set()
    exp = TopExp_Explorer(solid, TopAbs_ShapeEnum.TopAbs_VERTEX)
    while exp.More():
        p = BRep_Tool.Pnt_s(TopoDS.Vertex(exp.Current()))
        out.add((round(p.X(), 6), round(p.Y(), 6), round(p.Z(), 6)))
        exp.Next()
    return out


def _read_document(src: Path):
    """Load the STEP into an XDE document, keeping colours and part names."""
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDocStd import TDocStd_Document

    doc = TDocStd_Document(TCollection_ExtendedString("tron"))
    reader = STEPCAFControl_Reader()
    reader.SetColorMode(True)
    reader.SetNameMode(True)
    reader.SetLayerMode(True)
    if reader.ReadFile(str(src)) != IFSelect_RetDone:
        sys.exit(f"OpenCascade could not read {src}")
    reader.Transfer(doc)
    return doc


def _solid_colour(shape, label, colour_tool):
    """The solid's XDE colour -- its own, else its assembly label's -- or None if XDE has neither.

    XDE stores colour against whichever label carried it in the source CAD -- sometimes the solid,
    sometimes the component or the part above it -- so a miss on the shape is normal and means
    "ask the parent", not "no colour".  None means XDE genuinely has nothing, and the caller then
    falls back to the STEP's own style table (see ``_step_colour_table``).
    """
    from OCP.Quantity import Quantity_Color
    from OCP.XCAFDoc import XCAFDoc_ColorType

    col = Quantity_Color()
    for kind in (XCAFDoc_ColorType.XCAFDoc_ColorSurf,
                 XCAFDoc_ColorType.XCAFDoc_ColorGen,
                 XCAFDoc_ColorType.XCAFDoc_ColorCurv):
        if colour_tool.GetColor(shape, kind, col):
            return (_to_srgb(col.Red()), _to_srgb(col.Green()), _to_srgb(col.Blue()), 1.0)
    # The instance method only accepts a shape; the label fallback goes through the static
    # overload, which is the one that takes a TDF_Label.
    if label is not None:
        from OCP.XCAFDoc import XCAFDoc_ColorTool

        for kind in (XCAFDoc_ColorType.XCAFDoc_ColorSurf,
                     XCAFDoc_ColorType.XCAFDoc_ColorGen):
            if XCAFDoc_ColorTool.GetColor_s(label, kind, col):
                return (_to_srgb(col.Red()), _to_srgb(col.Green()), _to_srgb(col.Blue()), 1.0)
    return None


def _triangulate(solid, tol: float, angular: float):
    """Mesh one solid and return (vertices, faces) in numpy, then free the triangulation.

    The copy into numpy matters: the triangulation lives in OpenCascade's own allocator, and
    ``BRepTools.Clean`` right after is what keeps a 138-solid assembly from accumulating all of it.
    Face orientation is honoured (reversed faces get their winding flipped) so normals point out
    and the machine does not render inside-out.
    """
    import numpy as np
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.BRepTools import BRepTools
    from OCP.TopAbs import TopAbs_Orientation, TopAbs_ShapeEnum
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS

    BRepMesh_IncrementalMesh(solid, tol, False, angular, True)

    chunks_v: list = []
    chunks_f: list = []
    offset = 0
    exp = TopExp_Explorer(solid, TopAbs_ShapeEnum.TopAbs_FACE)
    while exp.More():
        # the explorer yields TopoDS_Shape; Triangulation_s needs the TopoDS_Face subtype
        face = TopoDS.Face(exp.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            trsf = loc.Transformation()
            n_v = tri.NbNodes()
            verts = np.empty((n_v, 3), np.float64)
            for i in range(1, n_v + 1):
                p = tri.Node(i).Transformed(trsf)
                verts[i - 1] = (p.X(), p.Y(), p.Z())
            n_f = tri.NbTriangles()
            faces = np.empty((n_f, 3), np.int64)
            for i in range(1, n_f + 1):
                a, b, c = tri.Triangle(i).Get()
                faces[i - 1] = (a - 1, b - 1, c - 1)
            if face.Orientation() == TopAbs_Orientation.TopAbs_REVERSED:
                faces = faces[:, ::-1]
            chunks_v.append(verts)
            chunks_f.append(faces + offset)
            offset += n_v
        exp.Next()

    BRepTools.Clean_s(solid)
    if not chunks_v:
        return None, None
    return np.vstack(chunks_v), np.vstack(chunks_f)


def _up_axis(vertices, faces) -> int:
    """Which axis the machine stands on, from the largest flat area near an axis minimum.

    Same test as cad_machine.py: a machine tool rests on a base plate, so for each axis sum the
    area of faces whose normal points along it and that sit within 120 mm of that axis' minimum.
    """
    import numpy as np

    tri = vertices[faces]
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
        near_min = (centre[flat][:, ax] - vertices[:, ax].min()) < 0.12
        scores.append(float(area[flat][near_min].sum()))
    return int(np.argmax(scores))


def convert(src: Path, out_dir: Path, name: str, tol: float, angular: float,
            rotate: bool, yaw: float, scale: float, up_axis: int | None = None) -> dict:
    try:
        import numpy as np
        import OCP  # noqa: F401  -- import error surfaces here with the install hint
    except ImportError:
        sys.exit("needs cadquery-ocp and numpy:  .venv/bin/pip install -r requirements-cad.txt")

    from OCP.TopAbs import TopAbs_ShapeEnum
    from OCP.TopExp import TopExp_Explorer
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    from OCP.collections import Sequence_TDF_Label

    out_dir.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    print(f"  reading {src.name} ({src.stat().st_size / 1e6:.0f} MB) ...", flush=True)
    doc = _read_document(src)
    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    colour_tool = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
    print(f"  B-rep loaded in {time.time() - t0:.1f}s", flush=True)

    roots = Sequence_TDF_Label()
    shape_tool.GetFreeShapes(roots)
    solids: list[tuple] = []
    for i in range(1, roots.Length() + 1):
        label = roots.Value(i)
        exp = TopExp_Explorer(shape_tool.GetShape_s(label), TopAbs_ShapeEnum.TopAbs_SOLID)
        while exp.More():
            solids.append((exp.Current(), label))
            exp.Next()
    if not solids:
        sys.exit("no solids found in this STEP")
    print(f"  {len(solids)} solids", flush=True)

    # XDE misses colours that the STEP attaches to faces rather than solids, so keep the file's own
    # style table to fall back on.  Parsing it costs a few seconds and is skipped when XDE already
    # has a colour for every solid (the VF-2 case).
    xde_missing = sum(1 for solid, label in solids
                      if _solid_colour(solid, label, colour_tool) is None)
    table: dict = {}
    if xde_missing:
        print(f"  XDE has no colour for {xde_missing}/{len(solids)} solids; "
              f"reading the STEP's own styles ...", flush=True)
        table = _step_colour_table(src)
        print(f"  style table: {len(table['by_count'])} vertex-count keys "
              f"({time.time() - t0:.1f}s)", flush=True)

    # Tessellate one solid at a time, accumulating triangles per colour.  Peak memory is the
    # largest single solid, which is what makes a 50 MB assembly finish on a laptop.
    by_colour: dict[tuple, list] = {}
    skipped = 0
    fallback = 0
    for idx, (solid, label) in enumerate(solids, 1):
        rgba = _solid_colour(solid, label, colour_tool)
        if rgba is None and table:
            col = _table_colour(table, _solid_vertices(solid))
            if col is not None:
                rgba = (col[0], col[1], col[2], 1.0)
        if rgba is None:
            rgba = _DEFAULT_RGBA
            fallback += 1
        verts, faces = _triangulate(solid, tol, angular)
        if verts is None:
            skipped += 1
            continue
        by_colour.setdefault(_quantise(rgba), []).append((verts, faces, rgba))
        if idx % 10 == 0 or idx == len(solids):
            tris = sum(len(f) for g in by_colour.values() for _, f, _ in g)
            print(f"    {idx:4d}/{len(solids)}  {tris:9,} tris  {time.time() - t0:6.1f}s", flush=True)
    if skipped:
        print(f"  {skipped} solids produced no triangles (skipped)", flush=True)
    if fallback:
        print(f"  {fallback}/{len(solids)} solids fell back to default grey", flush=True)

    # Merge each colour group into one vertex/face array, splitting any group that would exceed
    # MuJoCo's per-mesh face limit.  Normally colour does the splitting for us, but a STEP with no
    # styles at all (the VF-4 is AP203, which carries none) collapses into a single group -- 469k
    # faces in one mesh, well over the limit.  Solids are kept whole and packed into chunks.
    groups: dict[str, tuple] = {}
    split_note = []
    for key in sorted(by_colour):
        parts = by_colour[key]
        chunks, cur, cur_faces = [], [], 0
        for verts, faces, _ in parts:
            if cur and cur_faces + len(faces) > _MAX_MESH_FACES:
                chunks.append(cur)
                cur, cur_faces = [], 0
            cur.append((verts, faces))
            cur_faces += len(faces)
        if cur:
            chunks.append(cur)
        if len(chunks) > 1:
            split_note.append((parts[0][2], len(chunks)))
        for chunk in chunks:
            vs, fs, off = [], [], 0
            for verts, faces in chunk:
                vs.append(verts)
                fs.append(faces + off)
                off += len(verts)
            groups[f"c{len(groups)}"] = (np.vstack(vs), np.vstack(fs), parts[0][2])
    total_faces = sum(len(f) for _, f, _ in groups.values())
    print(f"  {len(groups)} colour groups, {total_faces:,} faces", flush=True)
    for rgba, n in split_note:
        print(f"    colour {[round(c, 3) for c in rgba]} exceeded "
              f"{_MAX_MESH_FACES:,} faces -> split into {n} meshes", flush=True)

    # Units: STEP length unit is applied by OpenCascade, but SolidWorks exports are usually mm
    # while MuJoCo works in metres, so infer from the assembly's own size unless told otherwise.
    # Stack every group into one vertex/face array, re-indexing faces as we go, so the unit guess
    # and the up-axis vote both see the whole machine.
    stack_v, stack_f, run = [], [], 0
    for v, f, _ in groups.values():
        stack_v.append(v)
        stack_f.append(f + run)
        run += len(v)
    all_v, all_f = np.vstack(stack_v), np.vstack(stack_f)

    span = float((all_v.max(axis=0) - all_v.min(axis=0)).max())
    if scale <= 0:
        scale = 0.001 if span > 50 else 1.0
        print(f"  span {span:.1f} -> treating source as {'mm' if scale == 0.001 else 'm'}", flush=True)
    all_v = all_v * scale

    # ``up`` may be forced, because the flat-area vote is tessellation-sensitive and does get this
    # wrong: of the five Haas files, the VF-1/3/4 vote y (correct) while the VF-2 voted x and the
    # VF-2TR voted z, both of which lay the machine on its side.  When the result looks wrong, pin
    # it with --up rather than chasing it with --tol.
    if not rotate:
        up = 2
    elif up_axis is not None:
        up = up_axis
        print(f"  up axis pinned to {'xyz'[up]} (--up)", flush=True)
    else:
        up = _up_axis(all_v, all_f)
        print(f"  base plate found on the {'xyz'[up]} axis", flush=True)
        # A machine tool is normally tallest along its up axis, so disagreement here usually means
        # the flat-area vote latched onto a side panel and the machine will come out lying down.
        # This is not fatal -- a wide machine with a coolant tank can be genuinely wider than tall --
        # so warn and let the operator confirm with a render rather than overriding the vote.
        span = all_v.max(axis=0) - all_v.min(axis=0)
        if up != int(np.argmax(span)):
            print(f"  WARNING: the {'xyz'[int(np.argmax(span))]} axis is longer "
                  f"({span[int(np.argmax(span))]:.0f}) than the chosen up axis "
                  f"({span[up]:.0f}); render it from azimuth 180 and pass --up if it is on its side",
                  flush=True)

    xf = np.eye(4)
    if rotate:
        if up == 0:      # x is up -> bring it to z
            xf = _rot(-np.pi / 2, [0, 1, 0])
        elif up == 1:    # y is up (the SolidWorks default)
            xf = _rot(np.pi / 2, [1, 0, 0])
        print(f"  standing the machine on z", flush=True)
    if yaw:
        xf = _rot(np.radians(yaw), [0, 0, 1]) @ xf

    moved = (xf[:3, :3] @ all_v.T).T + xf[:3, 3]
    lo, hi = moved.min(axis=0), moved.max(axis=0)
    shift = np.array([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from obj_to_msh import write_msh

    out: dict[str, dict] = {}
    for i, key in enumerate(groups):
        verts, faces, rgba = groups[key]
        v = (xf[:3, :3] @ (verts * scale).T).T + xf[:3, 3] + shift
        part = f"{name}_{i}"
        path = out_dir / f"{part}.msh"
        write_msh(path, np.asarray(v, np.float32),
                  np.zeros((0, 3), np.float32), np.asarray(faces, np.int32))
        rgba_r = [round(c, 4) for c in rgba]
        out[part] = {"file": f"{out_dir.name}/{path.name}", "rgba": rgba_r, "faces": len(faces)}
        print(f"    {part:16s} {len(faces):8,} faces  rgba {rgba_r}  "
              f"{path.stat().st_size / 1e6:5.1f} MB", flush=True)

    ext = hi - lo
    print(f"  extents {ext[0]:.3f} x {ext[1]:.3f} x {ext[2]:.3f} m   "
          f"total {time.time() - t0:.1f}s", flush=True)
    return out


def _rot(angle: float, axis) -> "object":
    import numpy as np

    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    c, s = np.cos(angle), np.sin(angle)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    m = np.eye(4)
    m[:3, :3] = np.eye(3) * c + k * s + np.outer(a, a) * (1 - c)
    return m


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("source", type=Path)
    ap.add_argument("--name", required=True, help="asset basename, e.g. haas_vf1")
    # OpenCascade's deflection is in the model's own units, and these Haas exports are in mm, so
    # this is millimetres -- not the metres cad_machine.py's --tol took.  0.004 here would be 4
    # microns and buries a machine in triangles (1.8M faces for the VF-2); 0.5 mm is invisible at
    # render scale and keeps each colour group inside MuJoCo's 200k-face-per-mesh limit.
    ap.add_argument("--tol", type=float, default=0.5,
                    help="linear deflection in mm (default 0.5)")
    ap.add_argument("--angular", type=float, default=0.5,
                    help="angular deflection in radians (default 0.5)")
    ap.add_argument("--scale", type=float, default=0.0,
                    help="source->metre scale; 0 (default) infers mm vs m from the model size")
    ap.add_argument("--up", choices=("x", "y", "z"), default=None,
                    help="pin which source axis is up, when the base-plate vote gets it wrong")
    ap.add_argument("--no-rotate", action="store_true",
                    help="skip the base-plate up-axis rotation")
    ap.add_argument("--yaw", type=float, default=0.0,
                    help="degrees about z after the Z-up rotation, to face the door toward -x")
    args = ap.parse_args()

    if not args.source.exists():
        sys.exit(f"no such file: {args.source}")
    out_dir = ASSETS / args.name
    print(f"{args.source.name} -> {out_dir}")
    parts = convert(args.source, out_dir, args.name, args.tol, args.angular,
                    not args.no_rotate, args.yaw, args.scale,
                    None if args.up is None else "xyz".index(args.up))

    print("\nregistry `meshes:` block —")
    inner = ", ".join(f"{k}: {{file: {v['file']}, rgba: {v['rgba']}}}" for k, v in parts.items())
    print("    meshes: {" + inner + "}")
    print("\ngeoms for the machine XML —")
    for k in parts:
        print(f'  <geom name="cnc_{k}" type="mesh" mesh="{k}" material="{k}_mat" '
              f'contype="0" conaffinity="0"/>')
    return 0


if __name__ == "__main__":
    sys.exit(main())
