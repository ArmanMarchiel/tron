"""Security rules: detect unexpected behaviour in the ROS 2 runtime.

Purely observational.  These compare the observed ROS graph / command stream against a
declared allow-list.  Nothing here interacts with the robot or the ROS graph.
"""
from __future__ import annotations

from backend.pipeline.risk.safety_rules import Hit


def unexpected_node(twin: dict, cfg: dict) -> Hit | None:
    running = [n for n, s in twin["software"]["nodes"].items() if s.get("state") == "running"]
    bad = [n for n in running if n not in cfg["expected_nodes"]]
    if bad:
        return Hit("UNEXPECTED_ROS_NODE", "MEDIUM", "security",
                   f"Unexpected ROS node(s) running: {', '.join(bad)}",
                   {"nodes": bad, "since": {n: twin["software"]["nodes"][n].get("since") for n in bad}})
    return None


def unexpected_publisher(twin: dict, cfg: dict) -> Hit | None:
    bad = {}
    for topic, info in twin["software"]["topics"].items():
        allowed = cfg["expected_publishers"].get(topic)
        if allowed is None:
            continue
        extra = [p for p in info.get("publishers", []) if p not in allowed]
        if extra:
            bad[topic] = extra
    if not bad:
        return None
    if cfg["command_topic"] in bad:
        pubs = bad[cfg["command_topic"]]
        return Hit("UNEXPECTED_COMMAND_SOURCE", "CRITICAL", "security",
                   f"Node(s) {', '.join(pubs)} are publishing on {cfg['command_topic']}; only "
                   f"{', '.join(cfg['expected_publishers'][cfg['command_topic']])} is authorised",
                   {"topic": cfg["command_topic"], "unexpected_publishers": pubs,
                    "authorised": cfg["expected_publishers"][cfg["command_topic"]],
                    "last_command": twin["software"]["last_command"]})
    topic, pubs = next(iter(bad.items()))
    return Hit("UNEXPECTED_TOPIC_PUBLISHER", "MEDIUM", "security",
               f"Unexpected publisher(s) {', '.join(pubs)} on {topic}", {"topic": topic, "publishers": pubs})


def unexpected_command_source(twin: dict, cfg: dict) -> Hit | None:
    cmd = twin["software"]["last_command"]
    if not cmd or not cmd.get("source_node"):
        return None
    if twin["ts"] is not None and twin["ts"] - cmd["ts"] > 3.0:
        return None  # stale
    allowed = cfg["expected_publishers"].get(cmd.get("topic", cfg["command_topic"]))
    if allowed is None:
        allowed = cfg["expected_publishers"][cfg["command_topic"]]
    if cmd["source_node"] not in allowed:
        return Hit("UNEXPECTED_COMMAND_SOURCE", "CRITICAL", "security",
                   f"Joint trajectory received from unauthorised node {cmd['source_node']} "
                   f"(goal '{cmd.get('goal_name')}', {cmd.get('duration_s', 0):.1f} s)",
                   {"command": cmd, "authorised": allowed})
    return None


def unexpected_subscriber(twin: dict, cfg: dict) -> Hit | None:
    for topic, allowed in cfg["expected_subscribers"].items():
        info = twin["software"]["topics"].get(topic)
        if not info:
            continue
        extra = [s for s in info.get("subscribers", []) if s not in allowed]
        if extra:
            return Hit("UNEXPECTED_TOPIC_SUBSCRIBER", "MEDIUM", "security",
                       f"Unexpected subscriber(s) {', '.join(extra)} on {topic}", {"topic": topic, "subscribers": extra})
    return None


def unexpected_rate(twin: dict, cfg: dict) -> Hit | None:
    for topic, (lo, hi) in cfg["expected_rates_hz"].items():
        info = twin["software"]["topics"].get(topic)
        if not info or info.get("rate_hz") is None:
            continue
        r = info["rate_hz"]
        if r > hi or (r < lo and r > 0):
            return Hit("UNEXPECTED_MESSAGE_RATE", "LOW", "security",
                       f"{topic} observed at {r:.1f} Hz (expected {lo:.0f}-{hi:.0f} Hz)",
                       {"topic": topic, "rate_hz": r, "expected": [lo, hi]})
    return None


def unexpected_connection(twin: dict, cfg: dict) -> Hit | None:
    bad = [n for n in twin["network"]["participants"] if n not in cfg["expected_participants"]]
    if bad:
        return Hit("UNEXPECTED_CONNECTION", "MEDIUM", "security",
                   f"Unexpected DDS participant(s): {', '.join(bad)}",
                   {"participants": {n: twin["network"]["participants"][n] for n in bad}})
    return None


def unexpected_controller_command(twin: dict, cfg_sec: dict, cfg_safety: dict | None = None) -> Hit | None:
    """A trajectory whose implied peak joint velocity exceeds the controller's limit."""
    cmd = twin["software"]["last_command"]
    if not cmd or cfg_safety is None or not cmd.get("positions") or not cmd.get("start_positions"):
        return None
    if twin["ts"] is not None and twin["ts"] - cmd["ts"] > 3.0:
        return None
    dur = max(0.05, float(cmd.get("duration_s") or 1.0))
    peak = max(abs(a - b) for a, b in zip(cmd["positions"], cmd["start_positions"])) / dur * 1.875  # min-jerk peak factor
    if peak > cfg_safety["cmd_max_joint_velocity"]:
        return Hit("UNEXPECTED_CONTROLLER_COMMAND", "HIGH", "security",
                   f"Trajectory '{cmd.get('goal_name')}' implies a peak joint velocity of {peak:.2f} rad/s "
                   f"(controller limit {cfg_safety['cmd_max_joint_velocity']:.2f})",
                   {"command": cmd, "implied_peak_velocity": peak, "limit": cfg_safety["cmd_max_joint_velocity"]})
    return None


SECURITY_RULES = [unexpected_node, unexpected_publisher, unexpected_command_source, unexpected_subscriber,
                  unexpected_rate, unexpected_connection]
