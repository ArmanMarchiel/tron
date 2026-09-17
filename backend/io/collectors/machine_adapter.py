"""Machine adapter: presents a cell machine (CNC) to the platform as normalised events.

In simulation the "server" is the in-process ``CNCMachine``; for a real machine the same class wraps an
OPC UA (``asyncua``) or Modbus (``pymodbus``) client using the tag map declared in the scenario.  Either
way the platform sees ``MachineStateObserved`` (on change and at 1 Hz), ``MachineCommandReceived`` for
every command with its source and outcome, and ``InterlockViolated`` when the machine rejects one.
"""
from __future__ import annotations

from backend.pipeline.events.event_model import EventType, Source, make_event
from backend.io.collectors.base import Ingest


class MachineAdapter:
    def __init__(self, ingest: Ingest, machine, robot_id: str, protocol: str = "simulated"):
        self.ingest = ingest
        self.machine = machine
        self.robot_id = robot_id
        self.protocol = protocol
        self._last_tags: dict | None = None
        self._last_emit = 0.0

    def tick(self, now: float) -> None:
        self.machine.tick(now)
        tags = self.machine.tags()
        changed = self._last_tags is None or any(tags[k] != self._last_tags.get(k) for k in ("state", "door", "chuck", "alarm", "part_loaded"))
        if changed or now - self._last_emit >= 1.0:
            self._last_emit = now
            self._last_tags = dict(tags)
            self.ingest([make_event(EventType.MachineStateObserved, Source.MACHINE, self.robot_id,
                                    {"machine_id": self.machine.id, "protocol": self.protocol, **tags})])

    def command(self, cmd: str, client: str, now: float) -> bool:
        ok, reason = self.machine.command(cmd, client, now)
        self.ingest([make_event(EventType.MachineCommandReceived, Source.MACHINE, self.robot_id,
                                {"machine_id": self.machine.id, "command": cmd, "client": client, "accepted": ok, "reason": reason,
                                 "state": self.machine.state})])
        if not ok:
            self.ingest([make_event(EventType.InterlockViolated, Source.MACHINE, self.robot_id,
                                    {"machine_id": self.machine.id, "command": cmd, "client": client, "reason": reason,
                                     "state": self.machine.state, "door": self.machine.door, "chuck": self.machine.chuck})])
        self.tick(now)
        return ok
