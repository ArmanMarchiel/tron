# TRON — Cyber-Physical Digital Twin (security + safety observability for robots)

> **Phase 2 status (2026-09-16):** implemented and verified on the Franka Panda CNC-tending cell: robot
> registry (Panda, FR3, UR5e, UR10e) with scene composition and per-robot IK; YAML scenarios with a task
> runner, a PLC-style CNC behind a machine adapter, an articulated operator, area scanner, zones and
> speed-and-separation rules; nine fault scenarios A–I all detected by their expected rule; MCAP record and
> replay with incident bundles; shadow-simulation torque residuals; Time Series tab; orbit/zoom camera;
> API tokens, audit log, golden-run baseline capture, CSV export, Docker/compose, CI. See `docs/PHASE2.md`.
> Not yet run live: full CNC cycles on FR3/UR5e/UR10e (they compose, compile and reach every target; the
> cycle itself was only exercised on the Panda), and the ROS 2 collector (no ROS 2 on this host; CI builds it).

> When a robot behaves unexpectedly, can we reconstruct exactly what happened by comparing the
> robot's expected physical/software state against its observed state?

TRON is a local-first prototype of an observability layer that sits on top of a simulated
manipulator. It is **not** a controller, simulator or generic dashboard. It ingests normalised
events from any source, keeps a persistent digital twin with strictly separated *observed* and
*expected* state, runs deterministic safety/security rules in joint space, and reconstructs
incidents as an evidence-backed causal timeline.

```
MuJoCo physics (ground truth)  ─┐                         ┌─ 3D render (MJPEG) / live twin state
simulated ROS 2 graph (software)┼─► unified events ─► twin ─► expected vs actual (per joint) ─► risk ─► incidents
real ROS 2 collector (rclpy)   ─┘        SQLite            history / timeline                    reconstruction
```

## Simulator and robot

The physics engine is **MuJoCo** (runs natively on macOS, no GPU required). The robot is the
**Franka Emika Panda** 7-DoF arm from MuJoCo Menagerie, placed on a pedestal in a workcell with a
table, a fixed obstacle, a human proxy and a restricted zone (`backend/io/sim/assets/scene.xml`).

The cell is built around a **Haas VF-2** vertical machining centre modelled at true scale from the
published Haas Machine Layout Drawing (3147 mm wide x 2249 mm deep, 1742 mm enclosure, 939 mm door
opening, 914 x 356 mm table, 762/406/508 mm travels). The robot stands on a riser in front of the
doors so the vise falls inside its envelope, as a real tending cell is laid out.

The cell sits on a sealed slate shop floor with a painted yellow boundary enclosing the machine and
the robot's envelope; the operator patrols the aisle outside it, so a walking route never crosses the
machine. The boundary is derived from the cell's contents, not authored, so it follows a machine swap.
Annotation layers -- zones, the robot ROM, the scanner field and the floor markings -- can each be
switched off from the viewer (`GET`/`POST /api/sim/overlays`); they are visual only and never collide.

Machines are swappable: `backend/conf/machines/machines.yaml` names each model's MJCF, chuck offset,
door travel and working volume, and a scenario selects one with `machine: {ref: haas_vf2}`. Adding a
machine is a registry entry plus an MJCF fragment that exposes the same contract (a `cnc_machine`
body, `cnc_door_joint`, `cnc_jaw_*_joint` and a `cnc_chuck_site`). `GET /api/machines` lists them.

The CNC cell runs a finite job: four raw blocks go in, four finished parts come out, and the run then
reaches a terminal state (`task.cycles` in the scenario; omit it to run continuously). Raw stock is
square and a finished part is a turned cylinder, so the machine's output is recognisable on sight.
The arm's range of motion is drawn as a translucent cylinder sized from the robot's own `reach_m`
plus a margin; an operator inside it is a protective stop in its own right, independent of the area
scanner, and the arm resumes the step it was on once they step clear.
An in-process motion planner runs a pick/place cycle through joint-space waypoints baked with IK
(`backend/io/sim/bake_waypoints.py` → `waypoints.json`), executed by a minimum-jerk joint-trajectory
controller. The offscreen renderer streams the scene to the UI at 15 fps and can render the twin's
joint configuration at any past instant.

## Quick start

```bash
backend/scripts/run.sh                       # creates .venv, installs deps, starts http://127.0.0.1:8000
open http://127.0.0.1:8000           # pick a robot and scenario in the header, "Load cell"
docker compose up --build            # same thing containerised (OSMesa software rendering)
.venv/bin/python backend/scripts/tron_cli.py summary
.venv/bin/python backend/scripts/tron_cli.py faults        # faults of the active scenario (A..I for cnc_tending)
.venv/bin/python backend/scripts/tron_cli.py inject A      # trigger one; each auto-clears
.venv/bin/python backend/scripts/tron_cli.py session ur5e cnc_tending   # switch robot / scenario
.venv/bin/python backend/scripts/tron_cli.py record        # last 5 min -> backend/data/recordings/*.mcap
.venv/bin/python backend/scripts/tron_cli.py session franka_panda cnc_tending --adapter external && \
.venv/bin/python backend/scripts/tron_cli.py replay backend/data/recordings/<file>.mcap
.venv/bin/python backend/scripts/tron_cli.py incidents
.venv/bin/python backend/scripts/tron_cli.py incident INC-0001
.venv/bin/python -m pytest tests                  # 9 platform tests (no simulator needed)
```

The database is recreated on every start (`TRON_FRESH_DB=0` to keep it); it lives at `backend/data/tron.sqlite`.

## What it does

**Unified event model** (`backend/pipeline/events/event_model.py`) — every observation is an `Event`
`{event_id, timestamp, event_type, source, entity_id, payload, confidence}`. Append-only SQLite
store with a subscriber bus (`event_store.py`), designed to be replaced by a streaming backend
without touching consumers.

**Digital twin** (`backend/pipeline/twin/`) — `robot_model.py` holds identity, *physical* (observed ground
truth: joint angles/velocities/efforts, end-effector, contacts, distances), *software_belief*
(what `/joint_states` reports), sensors, actuators, software (nodes, topics, controller,
planner), network (DDS participants), environment, *expected*, divergence, risk, incidents.
`state_engine.py` folds events into it and snapshots it at 10 Hz so any past instant can be
reconstructed (`/api/robots/{id}/timeline/at?ts=`). `kinematics.py` uses the robot description
purely for forward kinematics (expected and reported end-effector position).

**Expected state** (`backend/pipeline/twin/expected_state.py`) — deterministic, no ML. Only trajectories
from the allow-listed planner count. Expected q(t) follows the commanded minimum-jerk trajectory
from the expected position at acceptance; expected q̇(t) is its derivative; after completion the
robot is expected to hold. Divergence (per-joint error, velocity error, end-effector distance,
reported-vs-physical encoder error) is computed on every physical observation; crossing the
*notice* threshold emits `StateDivergenceObserved`, which is what "first detected divergence"
refers to.

**Risk engine** (`backend/pipeline/risk/`) — safety rules: `JOINT_VELOCITY_LIMIT_EXCEEDED`,
`JOINT_LIMIT_EXCEEDED`, `TORQUE_LIMIT_EXCEEDED` (saturated *and* stalled), `WORKSPACE_VIOLATION`,
`PROXIMITY_RISK`, `COLLISION_EVENT`, `SENSOR_STATE_DIVERGENCE` (encoder vs physical),
`TRAJECTORY_DIVERGENCE`. Security rules (observational only): `UNEXPECTED_ROS_NODE`,
`UNEXPECTED_COMMAND_SOURCE`, `UNEXPECTED_TOPIC_PUBLISHER/SUBSCRIBER`, `UNEXPECTED_MESSAGE_RATE`,
`UNEXPECTED_CONNECTION`, `UNEXPECTED_CONTROLLER_COMMAND` (trajectory implying an over-limit joint
velocity). Rule transitions become events with evidence attached.

**Incidents** (`backend/pipeline/incidents/`) — anomalies are grouped into incidents; reconstruction builds
the causal timeline (planner goals, trajectory messages, controller state, graph changes, motion
changes, divergence notices, anomalies, contacts), an EXPECTED / ACTUAL summary, the first
detected divergence, what the robot believed vs what physically happened, and **root-cause
candidates** labelled *Observed*, *Inferred* or *Possible cause*, each pointing at evidence event
ids. Candidates whose evidence begins after the first anomaly are demoted as consequences.
No candidate is ever asserted as "the" root cause.

**Fault injection** (`backend/pipeline/faults/`) — five explicit, simulation-only scenarios applied inside
the adapter exactly where a real fault would act; each auto-clears:

| id | scenario | mechanism | expected detection |
|---|---|---|---|
| A | excessive velocity | controller executes trajectories 2.5× faster than commanded | `JOINT_VELOCITY_LIMIT_EXCEEDED` |
| B | unexpected command source | `/teleop_override` joins and publishes a trajectory toward the human at 10 Hz | `UNEXPECTED_COMMAND_SOURCE` (+ node, participant, rate, proximity, workspace) |
| C | sensor disagreement | joint2 encoder reports +0.25 rad vs physical | `SENSOR_STATE_DIVERGENCE` |
| D | trajectory deviation | joint2/joint4 lose 95 % of position gain; arm sags under gravity | `TRAJECTORY_DIVERGENCE` |
| E | collision | planner goal inside obstacle-1, collision check disabled | `COLLISION_EVENT` (+ torque stall) |

Trigger from the UI, the CLI, or `POST /api/scenarios/{A..E}/trigger`. Out-of-process adapters
read the active fault from `GET /api/faults/active`.

**Web UI** (`frontend/`, no build step, light theme) — MuJoCo render (MJPEG live; a rendered frame
of the historical joint configuration when scrubbing), per-joint actual/expected bars, robot
state, risk, fault buttons, the **expected vs actual vs divergence** joint table, the twin
timeline strip (click/drag to enter history mode: every panel then shows the twin as it was at
that instant), incident list, full reconstruction with clickable timestamps and evidence,
ROS runtime/network view, live event feed.

## Running against a real ROS 2 arm

```bash
TRON_ADAPTER=external backend/scripts/run.sh                    # no in-process simulator
colcon build --packages-select tron_ros2_collector && source install/setup.bash
ros2 run tron_ros2_collector collector --ros-args -p platform_url:=http://127.0.0.1:8000
ros2 run tron_ros2_collector rogue_publisher            # scenario B only, publishes while B is active
```
The collector (rclpy, ROS 2 Jazzy/Humble) forwards `/joint_states`, the joint-trajectory
command topic, the controller state and the ROS graph. Physical ground truth must come from an
independent source (motion capture, a simulator bridge); without one the platform compares
expected vs *reported* state. The ROS 2 package is written and compile-checked but has not been
executed (no ROS 2 on the development host).

## Layout

```
backend/
  app/          main.py (ASGI) · config.py · session.py
    api/        platform.py (wiring) · routes.py (REST + WebSocket + MJPEG)
  pipeline/     the analysis chain: events -> twin -> risk -> incidents
    events/     event_model.py · event_store.py
    twin/       robot_model.py · state_engine.py · expected_state.py · kinematics.py
    risk/       safety_rules.py · security_rules.py · anomaly_engine.py
    incidents/  incident_engine.py · timeline.py · reconstruction.py
    faults/     scenarios.py · manager.py
  io/           everything that talks to the outside world
    collectors/ base.py · mujoco_adapter.py · replay_adapter.py · ros2_collector.py (pointer)
    sim/        renderer.py · compose.py · ik.py · assets/ (Menagerie robots, .msh)
    machines/   cnc.py (OPC UA / Modbus adapters)
    ros2/tron_ros2_collector/   ament_python package: collector node, rogue_publisher
  conf/         configuration as data
    scenarios/  loader.py · schema.py · defs/*.yaml
    cells/      <cell>.yaml: robot + scenario + thresholds + allow-lists
    registry/   robots.yaml: supported arms
    machines/   machines.yaml: supported machine tools (Haas VF-2, generic VMC)
    security/   auth.py · baseline.py
  scripts/      run.sh · tron_cli.py · obj_to_msh.py
  data/         runtime output, gitignored: tron.sqlite · scenes/ · sessions/ · waypoints/
frontend/       index.html · app.js · app.css · robot/ twin/ incidents/ timeline/ risk/
tests/          test_platform.py · test_phase2.py
```

## API

`GET /api/status` · `GET /api/robots` · `GET /api/robots/{id}/twin` ·
`GET /api/robots/{id}/timeline/at?ts=` · `GET /api/robots/{id}/timeline/track?since=&until=` ·
`GET /api/events?types=&sources=&since=&until=&limit=` · `GET /api/risk` ·
`GET /api/incidents` · `GET /api/incidents/{id}?include_operator=` · `GET /api/scenarios` ·
`POST /api/scenarios/{id}/trigger` · `POST /api/scenarios/clear` · `GET /api/faults/active` ·
`GET /api/sim/stream` (MJPEG) · `GET /api/sim/frame?ts=` · `GET /api/sim/waypoints` ·
`POST /api/ingest` (adapters) · `WS /api/ws` (twin at 5 Hz + live events)

## Known limitations

- The Menagerie Panda's wrist actuators (12 N·m) saturate briefly during ordinary motion under
  its high position gains; the torque rule therefore requires *sustained saturation while stalled*.
- The ROS 2 collector cannot attribute an individual trajectory message to its publisher when more
  than one publisher exists (rclpy does not expose per-message sender); it reports all candidates
  and the graph-level rule flags the unexpected one.
- After scenario E the gripper is wedged in the obstacle; the retreat can produce brief follow-up
  contacts, which the reconstruction labels as consequences.

## License

TRON is MIT licensed (see [LICENSE](LICENSE)).

The robot models under `backend/io/sim/assets/` (Franka Emika Panda, Franka FR3, Universal Robots
UR5e and UR10e) are from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)
and remain under their own licenses -- each model directory keeps its upstream `LICENSE`,
`README.md` and `CHANGELOG.md`.

Their visual meshes are stored as MuJoCo binary `.msh` rather than the upstream ASCII `.obj`
(`backend/scripts/obj_to_msh.py`): same vertices, normals and faces, about a third of the bytes,
and no ASCII parsing at model load. Collision geometry is untouched -- it comes from the `.stl`
meshes, so physics is bit-for-bit unaffected.
