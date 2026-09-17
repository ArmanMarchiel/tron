"""Golden-run baseline capture: derive the ROS graph / rate / participant allow-lists from observation
instead of writing them by hand.  Run a known-good cycle, capture, review, commit the YAML."""
from __future__ import annotations

import time
from pathlib import Path

import yaml

from backend.app.config import DATA_DIR
from backend.pipeline.events.event_model import EventType


def current_baseline(p) -> dict:
    sec = p.security
    return {"expected_nodes": sec["expected_nodes"], "expected_publishers": sec["expected_publishers"],
            "expected_subscribers": sec["expected_subscribers"], "expected_rates_hz": sec["expected_rates_hz"],
            "expected_participants": sec["expected_participants"], "command_topic": sec["command_topic"]}


def capture_baseline(p, since: float | None = None) -> dict:
    since = since or (time.time() - 300)
    nodes, pubs, subs, rates, parts = set(), {}, {}, {}, set()
    for e in p.store.query(since=since, types=[EventType.ROSNodeStarted, EventType.ROSTopicObserved, EventType.NetworkConnectionObserved], limit=50000):
        pl = e.payload
        if e.event_type == EventType.ROSNodeStarted:
            nodes.add(pl["node"])
        elif e.event_type == EventType.ROSTopicObserved:
            pubs.setdefault(pl["topic"], set()).update(pl.get("publishers", []))
            subs.setdefault(pl["topic"], set()).update(pl.get("subscribers", []))
            if pl.get("rate_hz") is not None:
                rates.setdefault(pl["topic"], []).append(float(pl["rate_hz"]))
        else:
            parts.update(x["name"] for x in pl.get("participants", []))
    rate_bounds = {t: (round(max(0.0, min(v) * 0.5), 2), round(max(v) * 1.5 + 0.5, 2)) for t, v in rates.items() if v}
    baseline = {"captured_at": time.time(), "since": since,
                "expected_nodes": sorted(nodes), "expected_publishers": {t: sorted(v) for t, v in pubs.items()},
                "expected_subscribers": {t: sorted(v) for t, v in subs.items()}, "expected_rates_hz": rate_bounds,
                "expected_participants": sorted(parts), "command_topic": p.security["command_topic"]}
    out = Path(DATA_DIR) / "baselines"
    out.mkdir(parents=True, exist_ok=True)
    (out / f"baseline_{int(time.time())}.yaml").write_text(yaml.safe_dump(baseline, sort_keys=False))
    # apply to the running session (reviewable file written above)
    sec = dict(p.security)
    sec.update({k: baseline[k] for k in ("expected_nodes", "expected_publishers", "expected_subscribers", "expected_rates_hz", "expected_participants")})
    p.security = sec
    p.risk.security = sec
    p.state.security = sec
    return baseline
