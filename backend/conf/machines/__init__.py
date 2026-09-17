"""Machine registry: the machine tools a cell can be built around.

A ``MachineProfile`` is everything the platform needs about a machine that is not derivable from the
scene: which MJCF fragment draws it, where the part sits relative to its origin, the real-world
dimensions the scenario zones are derived from, and its process capabilities.  A scenario picks one
with ``machine: {ref: <id>}``; swapping machines is a registry entry plus an MJCF file.

Every model must expose the same contract the adapter drives: a ``cnc_machine`` body placed at the
scenario's machine pose, a ``cnc_door_joint``, ``cnc_jaw_l_joint`` / ``cnc_jaw_r_joint``, and a
``cnc_chuck_site``.  Geoms are named ``cnc_*`` so the obstacle scan picks them up.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml

ASSETS = Path(__file__).resolve().parent.parent.parent / "io" / "sim" / "assets"
REGISTRY_FILE = Path(__file__).resolve().parent / "machines.yaml"


@dataclass
class MachineProfile:
    id: str
    name: str
    mjcf: str
    dimensions: dict                 # width / depth / height / enclosure_height (m)
    chuck_offset: list[float]        # part position relative to the machine origin
    front_offset: float              # machine origin -> front face along x (negative = toward the robot)
    door: dict = field(default_factory=dict)
    table: dict = field(default_factory=dict)
    travels: dict = field(default_factory=dict)
    spindle: dict = field(default_factory=dict)
    tool_changer: dict = field(default_factory=dict)
    work_volume: dict = field(default_factory=dict)   # enclosed working volume, machine-relative
    meshes: dict = field(default_factory=dict)        # mesh name -> file, relative to the assets dir
    max_table_load_kg: float | None = None

    @property
    def jaw_clamped(self) -> float | None:
        """How far the jaws travel to grip a part; None keeps the platform default."""
        return self.table.get("jaw_clamped")

    @property
    def mjcf_path(self) -> Path:
        return ASSETS / self.mjcf

    def mesh_assets(self) -> str:
        """<mesh> and <material> declarations for this machine.

        A CAD machine ships one mesh per colour in the source model, so its look survives the import;
        the matching material is declared here rather than in the shared scene template."""
        out = [f'<mesh name="{name}" inertia="shell" file="{(ASSETS / spec["file"] if isinstance(spec, dict) else ASSETS / spec).resolve()}"/>'
               for name, spec in self.meshes.items()]
        for name, spec in self.meshes.items():
            if isinstance(spec, dict) and spec.get("rgba"):
                out.append(f'<material name="{name}_mat" rgba="{" ".join(str(c) for c in spec["rgba"])}"/>')
        return "\n".join(out)

    def shell_geoms(self, prefix: str = "cnc_shell") -> str:
        """Visual-only geoms for the imported shell, one per colour group."""
        return "\n  ".join(
            f'<geom name="{prefix}_{i}" type="mesh" mesh="{name}" '
            f'material="{name}_mat" contype="0" conaffinity="0"/>'
            for i, name in enumerate(self.meshes))

    @property
    def width(self) -> float:
        return float(self.dimensions.get("width", 0.0))

    @property
    def depth(self) -> float:
        return float(self.dimensions.get("depth", 0.0))

    @property
    def enclosure_height(self) -> float:
        return float(self.dimensions.get("enclosure_height", self.dimensions.get("height", 0.0)))

    def interior_bounds(self, pose: list[float]) -> tuple[list[float], list[float]]:
        """World-space AABB of the enclosed working volume, for the restricted interior zone.

        This is the space reachable through the door -- where the spindle moves -- not the machine's
        whole footprint, so standing beside the machine is not a zone breach."""
        lo = self.work_volume.get("min") or [self.front_offset, -self.width / 2, 0.0]
        hi = self.work_volume.get("max") or [-self.front_offset, self.width / 2, self.enclosure_height]
        return ([pose[i] + lo[i] for i in range(3)], [pose[i] + hi[i] for i in range(3)])

    def summary(self) -> dict:
        return {"id": self.id, "name": self.name, "dimensions": dict(self.dimensions),
                "travels": dict(self.travels), "spindle": dict(self.spindle),
                "tool_changer": dict(self.tool_changer), "max_table_load_kg": self.max_table_load_kg}


_cache: dict[str, MachineProfile] | None = None


def load_machines() -> dict[str, MachineProfile]:
    global _cache
    if _cache is None:
        raw = yaml.safe_load(REGISTRY_FILE.read_text())["machines"]
        _cache = {mid: MachineProfile(id=mid, **spec) for mid, spec in raw.items()}
    return _cache


def get_machine(machine_id: str) -> MachineProfile:
    machines = load_machines()
    if machine_id not in machines:
        raise KeyError(f"unknown machine '{machine_id}'; known: {', '.join(machines)}")
    return machines[machine_id]


def list_machines() -> list[dict]:
    return [m.summary() for m in load_machines().values()]
