#!/usr/bin/env python3
"""TRON command-line client.

  tron_cli.py status
  tron_cli.py twin                      # full twin JSON
  tron_cli.py summary                   # expected vs actual
  tron_cli.py robots                    # robot registry
  tron_cli.py scenarios                 # scenario files
  tron_cli.py session <robot> <scenario> [--adapter mujoco|external]
  tron_cli.py faults                    # fault scenarios of the active scenario
  tron_cli.py inject A [--duration 8]   # trigger fault A..I
  tron_cli.py clear
  tron_cli.py record [--since T --until T]   # write an MCAP of the last 5 min
  tron_cli.py replay <file.mcap> [--speed 1]
  tron_cli.py incidents
  tron_cli.py incident INC-0001 [--operator]
  tron_cli.py at <unix_ts>              # reconstruct the twin at a point in time
  tron_cli.py events [--types A,B] [--limit 50]
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import sys
import urllib.parse
import urllib.request

BASE = os.environ.get("TRON_URL", "http://127.0.0.1:8000")


def call(path: str, method: str = "GET", params: dict | None = None, body: dict | None = None):
    url = BASE + "/api" + path
    if params:
        url += "?" + urllib.parse.urlencode({k: v for k, v in params.items() if v is not None})
    data = json.dumps(body).encode() if body is not None else None
    headers = {"Content-Type": "application/json"}
    tok = os.environ.get("TRON_TOKEN")
    if tok:
        headers["Authorization"] = "Bearer " + tok
    req = urllib.request.Request(url, method=method, data=data, headers=headers)
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read() or b"null")


def ts(t: float | None) -> str:
    return dt.datetime.fromtimestamp(t).strftime("%H:%M:%S.%f")[:-3] if t else "—"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd")
    ap.add_argument("arg", nargs="?")
    ap.add_argument("arg2", nargs="?")
    ap.add_argument("--adapter", default="mujoco")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--since", type=float)
    ap.add_argument("--until", type=float)
    ap.add_argument("--duration", type=float)
    ap.add_argument("--operator", action="store_true", help="include fault-injection events in reconstructions")
    ap.add_argument("--types")
    ap.add_argument("--limit", type=int, default=50)
    a = ap.parse_args()

    if a.cmd == "status":
        print(json.dumps(call("/status"), indent=2))
    elif a.cmd == "twin":
        print(json.dumps(call("/robots/robot-001/twin"), indent=2))
    elif a.cmd == "summary":
        s = call("/robots")[0]
        e, x, d = s["expected"], s["actual"], s["divergence"]
        print(f"ROBOT {s['robot_id']}\nSTATUS {s['status']}  Risk: {s['risk']}  Active incidents: {s['active_incidents']}\n")
        print(f"{'':16}{'EXPECTED':>14}{'ACTUAL':>14}{'DIVERGENCE':>14}")
        print(f"{'Velocity':16}{e['velocity']:>11.2f} m/s{x['velocity']:>11.2f} m/s{d['velocity_pct']:>+12.0f}%")
        print(f"{'Heading':16}{e['heading_deg']:>13.0f}°{x['heading_deg']:>13.0f}°{d['heading_deg']:>+13.1f}°")
        print(f"{'Position':16}{'(%.1f, %.1f)' % (e['position']['x'], e['position']['y']):>14}{'(%.1f, %.1f)' % (x['position']['x'], x['position']['y']):>14}{d['position_m']:>12.2f} m")
        g = lambda p: ("(%.1f, %.1f)" % (p["x"], p["y"])) if p else "—"
        print(f"{'Goal':16}{g(e['goal']):>14}{g(x['goal']):>14}")
        print(f"{'Controller':16}{e['controller']:>24}{x['controller']:>24}")
    elif a.cmd == "robots":
        for r in call("/registry/robots"):
            print(f"{r['id']:14} {r['name']:36} {r['dof']} DoF  reach {r['reach_m']} m")
    elif a.cmd == "scenarios":
        for s in call("/scenarios"):
            print(f"{s['id']:14} {s.get('name',''):30} steps={s.get('steps','?')} faults={','.join(s.get('faults', []))} {'ACTIVE' if s.get('active') else ''}")
    elif a.cmd == "session":
        if not a.arg:
            print(json.dumps(call("/session"), indent=2)); return 0
        r = call("/session", "POST", body={"robot": a.arg, "scenario": a.arg2 or "cnc_tending", "adapter": a.adapter})
        print(f"session {r['id']} started: robot={r['robot']} scenario={r['scenario']} adapter={r['adapter']}")
    elif a.cmd == "faults":
        for s in call("/faults"):
            print(f"{s['id']}  {s['name']:28} -> {s['expected_detection']:28} step={s.get('step','any'):10} {'ACTIVE' if s['active'] else ''}\n    {s['description']}")
    elif a.cmd == "inject":
        if not a.arg:
            print("usage: inject <fault id>"); return 2
        r = call(f"/faults/{a.arg.upper()}/trigger", "POST", {"duration_s": a.duration})
        print(f"injected {r['id']} {r['name']} for {r['duration_s']} s (expect {r['expected_detection']})")
    elif a.cmd == "clear":
        print(json.dumps(call("/faults/clear", "POST")))
    elif a.cmd == "record":
        r = call("/record", "POST", body={"since": a.since, "until": a.until})
        print(f"recorded {r['events']} events to {r['path']}")
    elif a.cmd == "replay":
        r = call("/replay", "POST", body={"path": a.arg, "speed": a.speed})
        print(json.dumps(r, indent=2))
    elif a.cmd == "incidents":
        for i in call("/incidents"):
            print(f"{i['incident_id']}  {i['status']:6} {i['severity']:8} {ts(i['created_at'])}  {i['title']}  [{', '.join(i['rules'])}]")
    elif a.cmd == "incident":
        i = call(f"/incidents/{a.arg}", params={"include_operator": "true" if a.operator else "false"})
        r = i["reconstruction"]; s = r["summary"]
        print(f"INCIDENT {i['incident_id']}\n{i['title']}  ({i['severity']}, {i['status']})\n")
        for t in r["timeline"]:
            print(f"{ts(t['ts'])}  {t['kind']:11} {t['text']}")
        print(f"\nEXPECTED\n{s['expected']}\n\nACTUAL\n{s['actual']}\n\nFIRST DETECTED DIVERGENCE\n{ts(s['first_detected_divergence'])}")
        print(f"\nROBOT BELIEVED   {s['robot_believed']}\nPHYSICALLY       {s['physically']}")
        print("\nWHAT CHANGED"); [print("  -", w) for w in s["what_changed"]]
        print("\nROOT-CAUSE CANDIDATES")
        for c in r["root_cause_candidates"]:
            print(f"  {c['rank']}. [{c['label']}] {c['title']}\n     {c['detail']}\n     evidence: {', '.join(c['evidence'])}")
    elif a.cmd == "at":
        s = call("/robots/robot-001/timeline/at", params={"ts": a.arg})
        t = s["twin"]
        print(f"twin @ {ts(s['snapshot_ts'])}: pos ({t['physical']['position']['x']:.2f}, {t['physical']['position']['y']:.2f}) "
              f"v {t['physical']['velocity']['linear']:.2f}  expected v {t['expected']['velocity']['linear']:.2f}  risk {t['risk']['level']}  "
              f"rules {[r['rule'] for r in t['risk']['active_rules']]}  nodes {[n for n, v in t['software']['nodes'].items() if v['state']=='running']}")
        print(f"commands in preceding second: {[(c['payload']['source_node'], c['payload']['linear']) for c in s['commands']]}")
    elif a.cmd == "events":
        for e in call("/events", params={"types": a.types, "limit": a.limit}):
            print(f"{ts(e['timestamp'])} {e['event_type']:26} {e['source']:18} {json.dumps(e['payload'])[:100]}")
    else:
        print(__doc__); return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
