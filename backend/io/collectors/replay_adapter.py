"""Log recording and replay (MCAP), plus incident bundles.

Recording writes an MCAP file with two kinds of channels:
  * ROS 2 messages (CDR, with schemas) for what a real collector would also see:
    ``/joint_states`` (sensor_msgs/JointState), the trajectory command topic (trajectory_msgs/JointTrajectory)
  * JSON channels carrying TRON's normalised events: ``/tron/events`` (software / machine / operator events)
    and ``/tron/ground_truth`` (physics observations).  A real field log has only the ROS channels; the
    replay then compares expected vs *reported* state and the reconstruction says so.

Replay reads any MCAP: ROS 2 channels are mapped with the same rules as the live ROS 2 collector; JSON
TRON channels are ingested as-is.  Original timestamps are preserved; playback is real-time, N× or
as fast as possible.
"""
from __future__ import annotations

import io
import json
import threading
import time
import zipfile
from pathlib import Path

from backend.pipeline.events.event_model import Event, EventType, Source, make_event

JOINT_STATE_SCHEMA = """std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""
TRAJ_SCHEMA = """std_msgs/Header header
string[] joint_names
trajectory_msgs/JointTrajectoryPoint[] points
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: trajectory_msgs/JointTrajectoryPoint
float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
builtin_interfaces/Duration time_from_start
================================================================================
MSG: builtin_interfaces/Duration
int32 sec
uint32 nanosec
"""
ROS_TYPES = {EventType.PositionObserved, EventType.CommandReceived}
GROUND_TRUTH_TYPES = {EventType.RobotStateObserved, EventType.CollisionDetected, EventType.EnvironmentObserved}
PLATFORM_DERIVED = {EventType.AnomalyDetected, EventType.SafetyThresholdExceeded, EventType.StateDivergenceObserved,
                    EventType.IncidentCreated, EventType.IncidentUpdated, EventType.IncidentClosed}


def _stamp(ts: float) -> dict:
    return {"sec": int(ts), "nanosec": int((ts - int(ts)) * 1e9)}


def record_mcap(store, out: Path, since: float, until: float, identity: dict) -> int:
    from mcap.writer import Writer as McapWriter
    from mcap_ros2.writer import Writer as Ros2Writer
    n = 0
    with open(out, "wb") as f:
        w = Ros2Writer(f)
        js = w.register_msgdef("sensor_msgs/msg/JointState", JOINT_STATE_SCHEMA)
        tj = w.register_msgdef("trajectory_msgs/msg/JointTrajectory", TRAJ_SCHEMA)
        base = w._writer  # underlying mcap writer for JSON channels
        json_schema = base.register_schema(name="tron.Event", encoding="jsonschema", data=json.dumps({"type": "object"}).encode())
        ch_events = base.register_channel(topic="/tron/events", message_encoding="json", schema_id=json_schema)
        ch_truth = base.register_channel(topic="/tron/ground_truth", message_encoding="json", schema_id=json_schema)
        base.add_metadata("tron", {"robot": identity.get("model", ""), "joints": json.dumps(identity.get("joints", [])),
                                   "since": str(since), "until": str(until)})
        cmd_topic = identity.get("ros2", {}).get("command_topic", "/joint_trajectory_controller/joint_trajectory")
        for e in store.query(since=since, until=until, limit=2_000_000):
            if e.event_type in PLATFORM_DERIVED:
                continue   # derived again on replay
            ns = int(e.timestamp * 1e9)
            pl = e.payload
            if e.event_type == EventType.PositionObserved:
                w.write_message(topic="/joint_states", schema=js, message={
                    "header": {"stamp": _stamp(e.timestamp), "frame_id": ""}, "name": pl.get("joint_names", identity.get("joints", [])),
                    "position": pl.get("position", []), "velocity": pl.get("velocity", []), "effort": pl.get("effort", [])},
                    log_time=ns, publish_time=ns)
            elif e.event_type == EventType.CommandReceived:
                dur = float(pl.get("duration_s") or 1.0)
                w.write_message(topic=pl.get("topic", cmd_topic), schema=tj, message={
                    "header": {"stamp": _stamp(e.timestamp), "frame_id": f"{pl.get('source_node') or ''}|{pl.get('goal_name') or ''}"},
                    "joint_names": pl.get("joint_names", identity.get("joints", [])),
                    "points": [{"positions": pl.get("positions", []), "velocities": [], "accelerations": [], "effort": [], "time_from_start": _stamp(dur)}]},
                    log_time=ns, publish_time=ns)
            ch = ch_truth if e.event_type in GROUND_TRUTH_TYPES else ch_events
            base.add_message(channel_id=ch, log_time=ns, publish_time=ns, data=json.dumps(e.model_dump()).encode(), sequence=e.seq or 0)
            n += 1
        w.finish()
    return n


class _JsonDecoderFactory:
    """Decoder for TRON's JSON channels (the ROS 2 factory covers CDR channels)."""

    def decoder_for(self, message_encoding: str, schema):
        if message_encoding == "json":
            return lambda data: json.loads(data)
        return None


class ReplayAdapter:
    name = "replay"

    def __init__(self, ingest, path: Path, speed: float = 1.0, ground_truth: bool = True, robot_id: str = "robot-001"):
        self.ingest = ingest
        self.path = Path(path)
        self.speed = speed
        self.ground_truth = ground_truth
        self.robot_id = robot_id
        self._stop = False
        self.state = "idle"
        self.count = 0
        self.first_ts: float | None = None
        self.last_ts: float | None = None
        self.thread = threading.Thread(target=self.run, name="tron-replay", daemon=True)

    def start(self) -> None:
        self.thread.start()

    def stop(self) -> None:
        self._stop = True

    def status(self) -> dict:
        return {"state": self.state, "path": str(self.path), "events": self.count, "speed": self.speed,
                "first_ts": self.first_ts, "last_ts": self.last_ts}

    def apply_fault(self, scenario: dict) -> None:   # faults are not applicable to a recording
        pass

    def clear_fault(self) -> None:
        pass

    def live_state(self):
        return None, None

    # ---------------------------------------------------------------- mapping
    def _map(self, topic: str, msg, log_time_ns: int, encoding: str, raw: bytes) -> list[Event]:
        ts = log_time_ns / 1e9
        if encoding == "json":
            d = json.loads(raw)
            if topic == "/tron/ground_truth" and not self.ground_truth:
                return []
            try:
                return [Event(**d)]
            except Exception:
                return []
        if topic == "/joint_states":
            return [make_event(EventType.PositionObserved, Source.REPLAY, self.robot_id, {
                "topic": "/joint_states", "joint_names": list(msg.name), "position": list(msg.position),
                "velocity": list(msg.velocity), "effort": list(msg.effort)}, timestamp=ts)]
        if topic.endswith("/joint_trajectory") and msg.points:
            last = msg.points[-1]
            src, _, goal = (msg.header.frame_id or "").partition("|")
            return [make_event(EventType.CommandReceived, Source.REPLAY, self.robot_id, {
                "topic": topic, "source_node": src or None, "goal_name": goal or "trajectory", "positions": list(last.positions),
                "duration_s": last.time_from_start.sec + last.time_from_start.nanosec * 1e-9, "joint_names": list(msg.joint_names)}, timestamp=ts)]
        return []

    def run(self) -> None:
        from mcap.reader import make_reader
        from mcap_ros2.decoder import DecoderFactory
        self.state = "running"
        self.ingest([make_event(EventType.ReplayStarted, Source.REPLAY, self.robot_id, {"path": str(self.path), "speed": self.speed})])
        with open(self.path, "rb") as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory(), _JsonDecoderFactory()])
            wall0, log0 = time.time(), None
            for schema, channel, message, decoded in reader.iter_decoded_messages():
                if self._stop:
                    break
                if log0 is None:
                    log0 = message.log_time
                    self.first_ts = log0 / 1e9
                if self.speed > 0:
                    due = wall0 + (message.log_time - log0) / 1e9 / self.speed
                    delay = due - time.time()
                    if delay > 0:
                        time.sleep(min(delay, 1.0))
                evs = self._map(channel.topic, decoded, message.log_time, channel.message_encoding, message.data)
                if evs:
                    self.ingest(evs)
                    self.count += len(evs)
                self.last_ts = message.log_time / 1e9
        self.state = "stopped" if self._stop else "finished"
        self.ingest([make_event(EventType.ReplayFinished, Source.REPLAY, self.robot_id, {"path": str(self.path), "events": self.count})])


def build_bundle(p, inc: dict) -> bytes:
    """Incident bundle: incident + reconstruction, events and snapshots in the window, render frames at key instants."""
    rec = inc["reconstruction"]
    since, until = rec["window"]["since"], rec["window"]["until"]
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("incident.json", json.dumps(inc, indent=1))
        z.writestr("events.jsonl", "\n".join(json.dumps(e.model_dump()) for e in p.store.query(since=since, until=until, limit=500000)))
        z.writestr("snapshots.jsonl", "\n".join(json.dumps(s) for s in p.store.snapshots(inc["entity_id"], since, until, limit=100000)))
        z.writestr("README.txt", f"TRON incident bundle {inc['incident_id']}\nwindow {since} .. {until}\n"
                                 f"files: incident.json (with reconstruction), events.jsonl, snapshots.jsonl, frames/*.jpg\n")
        if p.renderer is not None and p.adapter is not None and hasattr(p.adapter, "qadr"):
            keys = [("first_divergence", rec["summary"].get("first_detected_divergence")), ("created", inc["created_at"]), ("closed", inc.get("closed_at"))]
            for name, ts in keys:
                if ts is None:
                    continue
                snap = p.store.snapshot_at(inc["entity_id"], ts)
                if not snap:
                    continue
                qpos, mocap = p.adapter.live_state()
                for i, v in enumerate(snap["physical"]["position"][: p.adapter.n]):
                    qpos[p.adapter.qadr[i]] = v
                try:
                    jpeg = p.renderer.render_pose(qpos, mocap).result(timeout=5)
                    z.writestr(f"frames/{name}.jpg", jpeg)
                except Exception:
                    pass
    return buf.getvalue()
