"""Scenario schema (pydantic). A scenario binds assets, layout, zones, sensors, a task and faults."""
from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, Field, model_validator


class Tray(BaseModel):
    id: str
    pose: list[float]
    slots: list[list[float]] = Field(default_factory=list)   # xy offsets of slots relative to tray pose


class Block(BaseModel):
    id: str
    pose: list[float]
    size: float = 0.03          # half-size (m)
    mass: float = 0.5
    finished: bool = False


class Machine(BaseModel):
    id: str = "cnc-1"
    ref: str = "cnc_vmc_small"
    pose: list[float]
    cycle_time_s: float = 12.0
    chuck_offset: list[float] = Field(default_factory=lambda: [-0.06, 0.0, 0.16])   # part position rel. to machine pose
    protocol: Literal["simulated", "opcua", "modbus"] = "simulated"
    endpoint: str | None = None
    tags: dict[str, str] = Field(default_factory=dict)           # logical tag -> node id / register
    allowed_clients: list[str] = Field(default_factory=lambda: ["/motion_planner", "cell_plc"])


class Human(BaseModel):
    id: str = "human-1"
    pose: list[float]
    behaviour: str = "operator_patrol"
    patrol: list[list[float]] = Field(default_factory=list)
    speed: float = 0.4


class Zone(BaseModel):
    id: str
    type: Literal["restricted", "restricted_while", "reduced_speed", "ssm"]
    min: list[float]
    max: list[float]
    condition: str | None = None          # e.g. "machine.state == RUNNING"
    max_tcp_speed: float | None = None    # reduced_speed
    min_separation: float | None = None   # ssm base separation (m)
    stop_time_s: float = 0.5              # ssm: separation grows with tcp speed * stop_time


class Sensor(BaseModel):
    id: str
    type: Literal["area_scanner", "light_curtain"]
    field_min: list[float]
    field_max: list[float]
    protective_stop: bool = True           # intrusion must trigger a protective stop
    stop_within_s: float = 0.5


class Step(BaseModel):
    id: str
    action: Literal["wait", "move", "machine", "gripper"]
    target: str | None = None              # move: named target (from targets/trays/machine)
    command: str | None = None             # machine: door_open | door_close | clamp | unclamp | cycle_start
    gripper: Literal["open", "closed"] | None = None
    precondition: str | None = None        # e.g. "machine.state == COMPLETE"
    expect: str | None = None              # machine: expected tag after the command
    duration_s: float = 3.0
    timeout_s: float | None = None
    height_offset: float = 0.0             # move: z offset applied to the target
    on_fail: str | None = None             # step id to continue from when this step fails


class Task(BaseModel):
    loop: bool = True
    steps: list[Step]


class Fault(BaseModel):
    step: str = "any"
    name: str
    expected_detection: str
    description: str
    params: dict[str, Any]
    duration_s: float = 10.0


class Scenario(BaseModel):
    id: str
    name: str
    description: str = ""
    robot: str | None = None
    targets: dict[str, list[float]] = Field(default_factory=dict)   # named EE targets (xyz)
    trays: list[Tray] = Field(default_factory=list)
    blocks: list[Block] = Field(default_factory=list)
    machine: Machine | None = None
    human: Human | None = None
    zones: list[Zone] = Field(default_factory=list)
    sensors: list[Sensor] = Field(default_factory=list)
    task: Task
    faults: dict[str, Fault] = Field(default_factory=dict)

    @model_validator(mode="after")
    def _check_targets(self):
        names = set(self.targets) | {"home"}
        for t in self.trays:
            for i in range(len(t.slots)):
                names.add(f"{t.id}.slot{i}")
            names.add(f"{t.id}.next")
        if self.machine:
            names |= {"machine.chuck", "machine.chuck_above", "machine.door_frame", "machine.front", "machine.obstacle"}
        for s in self.task.steps:
            if s.action == "move" and s.target not in names:
                raise ValueError(f"step '{s.id}': unknown move target '{s.target}' (known: {sorted(names)})")
        return self
