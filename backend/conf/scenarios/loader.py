"""Load / list / save scenario YAML files (backend/scenarios/defs/*.yaml)."""
from __future__ import annotations

from pathlib import Path

import yaml

from backend.conf.scenarios.schema import Scenario

SCENARIO_DIR = Path(__file__).resolve().parent / "defs"


def list_scenarios() -> list[dict]:
    out = []
    for f in sorted(SCENARIO_DIR.glob("*.yaml")):
        try:
            sc = load_scenario(f.stem)
            out.append({"id": sc.id, "name": sc.name, "description": sc.description, "steps": len(sc.task.steps),
                        "machine": bool(sc.machine), "human": bool(sc.human), "faults": list(sc.faults)})
        except Exception as e:  # keep listing even if one file is broken
            out.append({"id": f.stem, "name": f.stem, "error": str(e)})
    return out


def load_scenario(scenario_id: str) -> Scenario:
    path = SCENARIO_DIR / f"{scenario_id}.yaml"
    if not path.exists():
        raise KeyError(f"unknown scenario '{scenario_id}'")
    return Scenario.model_validate(yaml.safe_load(path.read_text()))


def save_scenario(scenario_id: str, data: dict) -> Scenario:
    sc = Scenario.model_validate(data)          # validate before touching the file
    if sc.id != scenario_id:
        raise ValueError("scenario id in body must match the path")
    path = SCENARIO_DIR / f"{scenario_id}.yaml"
    path.write_text(yaml.safe_dump(sc.model_dump(exclude_none=True), sort_keys=False, allow_unicode=True))
    return sc
