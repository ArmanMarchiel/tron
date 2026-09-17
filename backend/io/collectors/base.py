"""Adapter contract.

An adapter turns *some* source of robot state (Isaac Sim, a ROS 2 graph, a real robot,
an OEM API) into normalised ``Event`` objects and hands them to ``ingest``.  The platform
core never imports simulator or ROS libraries; it only sees events.

Fault hooks are optional: adapters that can apply a controlled, simulation-only fault
scenario implement ``apply_fault`` / ``clear_fault``.  Out-of-process adapters poll
``GET /api/faults/active`` instead.
"""
from __future__ import annotations

from collections.abc import Callable, Iterable
from typing import Protocol

from backend.pipeline.events.event_model import Event

Ingest = Callable[[Iterable[Event]], None]


class Adapter(Protocol):
    name: str

    async def run(self) -> None: ...
    def stop(self) -> None: ...
    def apply_fault(self, scenario: dict) -> None: ...
    def clear_fault(self) -> None: ...
