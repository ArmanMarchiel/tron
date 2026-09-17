"""Unified event model.

Every observation entering the platform, regardless of origin (Isaac Sim ground
truth, ROS 2 runtime, real hardware, OEM API), is normalised into an ``Event``.
The core platform only ever consumes ``Event`` objects; it never talks to a
simulator or a ROS graph directly.

The schema is intentionally flat and JSON-serialisable so it can be moved onto
a streaming backend (Kafka/NATS/etc.) later without change.
"""
from __future__ import annotations

import time
import uuid
from enum import StrEnum
from typing import Any

from pydantic import BaseModel, Field


class EventType(StrEnum):
    # --- physical / twin state ---
    RobotStateObserved = "RobotStateObserved"
    SensorObservation = "SensorObservation"
    VelocityObserved = "VelocityObserved"
    PositionObserved = "PositionObserved"
    EnvironmentObserved = "EnvironmentObserved"
    # --- commands / control ---
    CommandReceived = "CommandReceived"
    CommandExecuted = "CommandExecuted"
    ControllerStateChanged = "ControllerStateChanged"
    # --- ROS runtime ---
    ROSNodeStarted = "ROSNodeStarted"
    ROSNodeStopped = "ROSNodeStopped"
    ROSTopicObserved = "ROSTopicObserved"
    NetworkConnectionObserved = "NetworkConnectionObserved"
    # --- navigation ---
    NavigationGoalReceived = "NavigationGoalReceived"
    NavigationGoalReached = "NavigationGoalReached"
    TrajectoryGoalReceived = "TrajectoryGoalReceived"
    TrajectoryGoalReached = "TrajectoryGoalReached"
    # --- task / machine (Phase 2) ---
    TaskStepStarted = "TaskStepStarted"
    TaskStepCompleted = "TaskStepCompleted"
    TaskStepFailed = "TaskStepFailed"
    TaskCycleCompleted = "TaskCycleCompleted"    # one part finished end to end
    TaskRunCompleted = "TaskRunCompleted"        # every requested cycle is done
    MachineStateObserved = "MachineStateObserved"
    MachineCommandReceived = "MachineCommandReceived"
    InterlockViolated = "InterlockViolated"
    ReplayStarted = "ReplayStarted"
    ReplayFinished = "ReplayFinished"
    SessionStarted = "SessionStarted"
    # --- safety ---
    CollisionDetected = "CollisionDetected"
    SafetyThresholdExceeded = "SafetyThresholdExceeded"
    # --- platform-derived ---
    StateDivergenceObserved = "StateDivergenceObserved"
    AnomalyDetected = "AnomalyDetected"
    IncidentCreated = "IncidentCreated"
    IncidentUpdated = "IncidentUpdated"
    IncidentClosed = "IncidentClosed"
    FaultInjected = "FaultInjected"
    FaultCleared = "FaultCleared"


class Source(StrEnum):
    """Where an observation came from. Ground truth is *just another source*."""
    MUJOCO = "mujoco"              # physical ground truth from the simulator
    ROS2 = "ros2"                  # software / runtime state from the ROS 2 graph
    REAL_ROBOT = "real_robot"
    PLATFORM = "platform"          # derived by this system
    OPERATOR = "operator"          # human / CLI / UI actions
    MACHINE = "machine"            # PLC / OPC UA / Modbus adapter
    REPLAY = "replay"              # log replay adapter


def now_ts() -> float:
    return time.time()


def new_event_id() -> str:
    return uuid.uuid4().hex[:16]


class Event(BaseModel):
    event_id: str = Field(default_factory=new_event_id)
    timestamp: float = Field(default_factory=now_ts, description="Unix seconds (float)")
    event_type: EventType
    source: str
    entity_id: str
    payload: dict[str, Any] = Field(default_factory=dict)
    confidence: float = 1.0
    # optional: sequence assigned by the store (monotonic), useful for streaming replay
    seq: int | None = None

    def to_row(self) -> tuple:
        import json
        return (
            self.event_id,
            self.timestamp,
            str(self.event_type),
            self.source,
            self.entity_id,
            json.dumps(self.payload, separators=(",", ":")),
            self.confidence,
        )

    @classmethod
    def from_row(cls, row: tuple) -> "Event":
        import json
        seq, event_id, ts, et, src, ent, payload, conf = row
        return cls(
            seq=seq, event_id=event_id, timestamp=ts, event_type=EventType(et),
            source=src, entity_id=ent, payload=json.loads(payload), confidence=conf,
        )


def make_event(event_type: EventType, source: str, entity_id: str,
               payload: dict[str, Any] | None = None, confidence: float = 1.0,
               timestamp: float | None = None) -> Event:
    return Event(
        event_type=event_type, source=source, entity_id=entity_id,
        payload=payload or {}, confidence=confidence,
        timestamp=timestamp if timestamp is not None else now_ts(),
    )
