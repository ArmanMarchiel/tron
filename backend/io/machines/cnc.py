"""PLC-style state machine for a small vertical machining centre.

States: IDLE -> RUNNING -> COMPLETE -> (door opens, part swapped) -> IDLE ...; ALARM on interlock breach.
Interlocks: cycle_start needs door CLOSED, chuck CLAMPED and a part loaded; door_open is rejected while
RUNNING (a bypassed interlock raises ALARM instead).  The door is physical: the adapter moves the door
joint toward ``door_target`` and feeds the measured position back, so the reported door state is
MOVING until the slide actually arrives.
"""
from __future__ import annotations

DOOR_OPEN_POS = 0.78
JAW_CLAMPED_POS = 0.04


class CNCMachine:
    def __init__(self, machine_id: str, cycle_time_s: float, allowed_clients: list[str], part_loaded: bool = True):
        self.id = machine_id
        self.cycle_time_s = cycle_time_s
        self.allowed_clients = list(allowed_clients)
        self.state = "COMPLETE" if part_loaded else "IDLE"
        self.chuck = "CLAMPED" if part_loaded else "OPEN"
        self.part_loaded = part_loaded
        self.alarm: str | None = None
        self.cycle_count = 0
        self.cycle_started_at: float | None = None
        self.door_target = 0.0
        self.door_pos = 0.0
        self.interlock_bypass = False

    # ---------------------------------------------------------------- reported tags
    @property
    def door(self) -> str:
        if self.door_pos >= DOOR_OPEN_POS - 0.02:
            return "OPEN"
        if self.door_pos <= 0.02:
            return "CLOSED"
        return "MOVING"

    def tags(self) -> dict:
        return {"state": self.state, "door": self.door, "chuck": self.chuck, "alarm": self.alarm,
                "cycle_count": self.cycle_count, "part_loaded": self.part_loaded,
                "cycle_progress": (min(1.0, (self._now - self.cycle_started_at) / self.cycle_time_s) if self.state == "RUNNING" and self.cycle_started_at else 0.0)}

    _now = 0.0

    # ---------------------------------------------------------------- physical feedback from the sim
    def set_door_position(self, pos: float) -> None:
        self.door_pos = pos

    def set_part_loaded(self, loaded: bool) -> None:
        self.part_loaded = loaded

    @property
    def jaw_target(self) -> float:
        return JAW_CLAMPED_POS if self.chuck == "CLAMPED" else 0.0

    # ---------------------------------------------------------------- commands
    def command(self, cmd: str, client: str, now: float) -> tuple[bool, str]:
        self._now = now
        if client not in self.allowed_clients:
            # observed but not executed: the machine's own access control
            return False, f"client {client} not authorised"
        if cmd == "door_open":
            if self.state == "RUNNING" and not self.interlock_bypass:
                return False, "door interlock: cycle RUNNING"
            if self.state == "RUNNING" and self.interlock_bypass:
                self.alarm = "DOOR_OPENED_WHILE_RUNNING"
                self.state = "ALARM"
            self.door_target = DOOR_OPEN_POS
            return True, "ok"
        if cmd == "door_close":
            self.door_target = 0.0
            return True, "ok"
        if cmd == "clamp":
            self.chuck = "CLAMPED"
            return True, "ok"
        if cmd == "unclamp":
            if self.state == "RUNNING":
                return False, "chuck interlock: cycle RUNNING"
            self.chuck = "OPEN"
            return True, "ok"
        if cmd == "cycle_start":
            if self.state == "ALARM":
                return False, "machine in ALARM; reset required"
            if self.door != "CLOSED":
                return False, "cycle_start interlock: door not CLOSED"
            if self.chuck != "CLAMPED":
                return False, "cycle_start interlock: chuck not CLAMPED"
            if not self.part_loaded:
                return False, "cycle_start rejected: no part loaded"
            self.state = "RUNNING"
            self.cycle_started_at = now
            return True, "ok"
        if cmd == "reset":
            self.alarm = None
            self.state = "IDLE"
            return True, "ok"
        return False, f"unknown command {cmd}"

    def tick(self, now: float) -> None:
        self._now = now
        if self.state == "RUNNING" and self.cycle_started_at is not None and now - self.cycle_started_at >= self.cycle_time_s:
            self.state = "COMPLETE"
            self.cycle_count += 1
        if self.state == "COMPLETE" and not self.part_loaded and self.chuck == "OPEN":
            self.state = "IDLE"
