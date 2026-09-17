"""Append-only SQLite event store plus an in-process pub/sub bus.

The store is the single ingestion point.  Adapters call ``EventStore.append``;
the state engine, risk engine and incident engine subscribe to the bus.
Swapping SQLite for a streaming backend later only requires replacing this
module - the subscriber interface (``Callable[[Event], None]``) stays.
"""
from __future__ import annotations

import json
import sqlite3
import threading
from collections.abc import Callable, Iterable
from pathlib import Path

from backend.pipeline.events.event_model import Event, EventType

Subscriber = Callable[[Event], None]

SCHEMA = """
CREATE TABLE IF NOT EXISTS events (
    seq        INTEGER PRIMARY KEY AUTOINCREMENT,
    event_id   TEXT UNIQUE NOT NULL,
    timestamp  REAL NOT NULL,
    event_type TEXT NOT NULL,
    source     TEXT NOT NULL,
    entity_id  TEXT NOT NULL,
    payload    TEXT NOT NULL,
    confidence REAL NOT NULL DEFAULT 1.0
);
CREATE INDEX IF NOT EXISTS idx_events_ts ON events(timestamp);
CREATE INDEX IF NOT EXISTS idx_events_type_ts ON events(event_type, timestamp);
CREATE INDEX IF NOT EXISTS idx_events_entity_ts ON events(entity_id, timestamp);

CREATE TABLE IF NOT EXISTS snapshots (
    seq        INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp  REAL NOT NULL,
    entity_id  TEXT NOT NULL,
    state      TEXT NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_snap_entity_ts ON snapshots(entity_id, timestamp);

CREATE TABLE IF NOT EXISTS incidents (
    incident_id TEXT PRIMARY KEY,
    entity_id   TEXT NOT NULL,
    created_at  REAL NOT NULL,
    updated_at  REAL NOT NULL,
    closed_at   REAL,
    status      TEXT NOT NULL,
    severity    TEXT NOT NULL,
    title       TEXT NOT NULL,
    data        TEXT NOT NULL
);
"""


class EventStore:
    def __init__(self, db_path: str | Path = ":memory:"):
        self.db_path = str(db_path)
        self._conn = sqlite3.connect(self.db_path, check_same_thread=False)
        self._conn.execute("PRAGMA journal_mode=WAL")
        self._conn.execute("PRAGMA synchronous=NORMAL")
        self._conn.executescript(SCHEMA)
        self._lock = threading.RLock()
        self._subscribers: list[Subscriber] = []

    # ---------------- pub/sub ----------------
    def subscribe(self, fn: Subscriber) -> None:
        self._subscribers.append(fn)

    # ---------------- write ----------------
    def append(self, event: Event) -> Event:
        with self._lock:
            cur = self._conn.execute(
                "INSERT INTO events(event_id,timestamp,event_type,source,entity_id,payload,confidence)"
                " VALUES (?,?,?,?,?,?,?)", event.to_row())
            event.seq = cur.lastrowid
            self._conn.commit()
        for fn in list(self._subscribers):
            fn(event)
        return event

    def append_many(self, events: Iterable[Event]) -> list[Event]:
        return [self.append(e) for e in events]

    # ---------------- read ----------------
    def query(self, *, since: float | None = None, until: float | None = None,
              types: Iterable[EventType | str] | None = None, entity_id: str | None = None,
              sources: Iterable[str] | None = None, limit: int = 1000, after_seq: int | None = None,
              ascending: bool = True) -> list[Event]:
        clauses, params = [], []
        if since is not None:
            clauses.append("timestamp >= ?"); params.append(since)
        if until is not None:
            clauses.append("timestamp <= ?"); params.append(until)
        if entity_id is not None:
            clauses.append("entity_id = ?"); params.append(entity_id)
        if after_seq is not None:
            clauses.append("seq > ?"); params.append(after_seq)
        if types:
            tl = [str(t) for t in types]
            clauses.append(f"event_type IN ({','.join('?' * len(tl))})"); params.extend(tl)
        if sources:
            sl = list(sources)
            clauses.append(f"source IN ({','.join('?' * len(sl))})"); params.extend(sl)
        where = ("WHERE " + " AND ".join(clauses)) if clauses else ""
        order = "ASC" if ascending else "DESC"
        sql = f"SELECT seq,event_id,timestamp,event_type,source,entity_id,payload,confidence FROM events {where} ORDER BY timestamp {order}, seq {order} LIMIT ?"
        params.append(limit)
        with self._lock:
            rows = self._conn.execute(sql, params).fetchall()
        return [Event.from_row(r) for r in rows]

    def get(self, event_id: str) -> Event | None:
        with self._lock:
            row = self._conn.execute(
                "SELECT seq,event_id,timestamp,event_type,source,entity_id,payload,confidence FROM events WHERE event_id=?",
                (event_id,)).fetchone()
        return Event.from_row(row) if row else None

    def count(self) -> int:
        with self._lock:
            return self._conn.execute("SELECT COUNT(*) FROM events").fetchone()[0]

    # ---------------- snapshots (twin history) ----------------
    def save_snapshot(self, entity_id: str, timestamp: float, state: dict) -> None:
        with self._lock:
            self._conn.execute("INSERT INTO snapshots(timestamp,entity_id,state) VALUES (?,?,?)",
                               (timestamp, entity_id, json.dumps(state, separators=(",", ":"))))
            self._conn.commit()

    def snapshot_at(self, entity_id: str, timestamp: float) -> dict | None:
        """Latest snapshot at or before ``timestamp``."""
        with self._lock:
            row = self._conn.execute(
                "SELECT state FROM snapshots WHERE entity_id=? AND timestamp<=? ORDER BY timestamp DESC, seq DESC LIMIT 1",
                (entity_id, timestamp)).fetchone()
        return json.loads(row[0]) if row else None

    def snapshots(self, entity_id: str, since: float, until: float, limit: int = 5000) -> list[dict]:
        with self._lock:
            rows = self._conn.execute(
                "SELECT state FROM snapshots WHERE entity_id=? AND timestamp>=? AND timestamp<=? ORDER BY timestamp ASC LIMIT ?",
                (entity_id, since, until, limit)).fetchall()
        return [json.loads(r[0]) for r in rows]

    def snapshot_range(self, entity_id: str) -> tuple[float | None, float | None]:
        with self._lock:
            row = self._conn.execute("SELECT MIN(timestamp), MAX(timestamp) FROM snapshots WHERE entity_id=?",
                                     (entity_id,)).fetchone()
        return (row[0], row[1]) if row else (None, None)

    # ---------------- incidents ----------------
    def upsert_incident(self, inc: dict) -> None:
        with self._lock:
            self._conn.execute(
                "INSERT INTO incidents(incident_id,entity_id,created_at,updated_at,closed_at,status,severity,title,data)"
                " VALUES (?,?,?,?,?,?,?,?,?) ON CONFLICT(incident_id) DO UPDATE SET"
                " updated_at=excluded.updated_at, closed_at=excluded.closed_at, status=excluded.status,"
                " severity=excluded.severity, title=excluded.title, data=excluded.data",
                (inc["incident_id"], inc["entity_id"], inc["created_at"], inc["updated_at"], inc.get("closed_at"),
                 inc["status"], inc["severity"], inc["title"], json.dumps(inc, separators=(",", ":"))))
            self._conn.commit()

    def list_incidents(self, limit: int = 100) -> list[dict]:
        with self._lock:
            rows = self._conn.execute("SELECT data FROM incidents ORDER BY created_at DESC LIMIT ?", (limit,)).fetchall()
        return [json.loads(r[0]) for r in rows]

    def get_incident(self, incident_id: str) -> dict | None:
        with self._lock:
            row = self._conn.execute("SELECT data FROM incidents WHERE incident_id=?", (incident_id,)).fetchone()
        return json.loads(row[0]) if row else None

    def close(self) -> None:
        with self._lock:
            self._conn.close()
