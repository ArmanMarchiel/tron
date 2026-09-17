# TRON Phase 2 — From prototype to an OEM-usable cyber-physical twin

**Status:** implemented (see table) · **Audience:** robotics OEM integration and validation engineering · **Deployment target:** on-prem, next to the cell or in the validation lab

| milestone | status | notes |
|---|---|---|
| M1 tab styling, Time Series | done | per-joint small multiples, shared scrubber, value table below |
| M2 robot registry & selection | done | Panda, FR3, UR5e, UR10e; composer, per-robot IK; live cycle verified on the Panda only |
| M3 articulated operator, scanner | done | patrol / intrusion behaviours; scanner sensed vs ground truth |
| M4 scenarios, CNC, task rules | done | `backend/conf/scenarios/defs/cnc_tending.yaml`, machine adapter, Scenario tab with editor; faults A–I detected |
| M5 MCAP record / replay, bundles | done | round-trip test in CI; ROS 2 + JSON channels |
| M6 shadow simulation | done | inverse dynamics on the expected motion, `UNEXPECTED_LOAD` |
| M7 zones, SSM, protective stop | done | rules configured per scenario |
| M8 OEM hardening | partial | API tokens, audit, baseline capture, CSV export, health, Docker/compose, CI, cell config; multi-cell sessions, retention policy and incident PDF reports not done |

---

## 1. Why Phase 2

Phase 1 proved the core loop: a simulated Franka Panda runs a pick/place cycle in MuJoCo; every observation
(physics ground truth and the ROS 2 software graph) enters a unified event store; a digital twin keeps
*observed* and *expected* state strictly apart; deterministic rules turn divergence into anomalies;
incidents are reconstructed as an evidence-backed causal timeline with ranked, labelled root-cause
candidates. Five controlled fault scenarios are detected with zero false positives in normal operation.

What it is not yet: usable by an OEM validation team on a real cell. It has one hard-coded robot, one
hard-coded motion cycle, no machine in the loop, a placeholder human, no way to replay a field log, and
thresholds that live in a Python file.

Phase 2 turns the skeleton into a tool an OEM's integration and validation engineers use to:

1. Run a realistic cell scenario (robot + CNC machine + workpieces + human) end to end.
2. Replay recorded field logs (rosbag2 / MCAP) through the same twin, rules and reconstruction.
3. Explain an incident with evidence, including *what step of the task* the robot was in and what the
   machine said at the time.
4. Configure everything per robot and per cell as versioned files.
5. Deploy on-prem in one command.

### Success criteria

| # | Criterion | How we prove it |
|---|---|---|
| S1 | Select any robot in the registry; the cell reloads and all fault scenarios still detect | integration test per robot |
| S2 | CNC-tending scenario runs unattended; task step order and timing are part of expected state | `TASK_*` and machine rules fire only under injected faults |
| S3 | A recorded log replays into a fresh twin and yields the same incident and top candidate as the live run | replay fixtures in CI |
| S4 | Unexpected external loads are detected from torque residuals without a contact report | shadow-sim test |
| S5 | Zones and speed/separation rules are configured per cell in YAML, not code | config-driven tests |
| S6 | `docker compose up` brings the platform up with health checks | smoke test |

---

## 2. Architecture

The Phase 1 core (event store → state engine → risk → incidents, wired in `backend/app/api/platform.py`)
stays as is. Phase 2 adds three layers around it.

```
 CONFIG (versioned YAML)                ADAPTERS (normalised events only)
 ┌────────────────────────┐             ┌──────────────────────────────────────────────┐
 │ registry/robots.yaml   │             │ MuJoCo adapter        physics ground truth    │
 │ registry/machines.yaml │             │ ROS 2 collector       software / runtime      │
 │ backend/conf/scenarios/defs/ │             │ Machine adapter       OPC UA / Modbus (sim→real)│
 │ backend/conf/cells/*.yaml    │             │ Replay adapter        rosbag2 / MCAP           │
 └───────────┬────────────┘             └───────────────┬──────────────────────────────┘
             │                                          │  POST /api/ingest (or in-process)
             ▼                                          ▼
 ┌────────────────────────────────────────────────────────────────────────────────────┐
 │  SCENARIO & TASK ENGINE      →   EVENT STORE   →   TWIN   →   EXPECTED STATE       │
 │  assets, task steps, faults      (SQLite, bus)      robot        trajectory + task   │
 │                                                     machine      step + machine      │
 │                                                     human        shadow-sim torques  │
 │                                                          ↓                           │
 │                                     RISK ENGINE  (safety · security · task · zones)  │
 │                                                          ↓                           │
 │                                     INCIDENTS  (grouping, reconstruction, bundles)   │
 └────────────────────────────────────────────────────────────────────────────────────┘
             │
             ▼
 WEB UI  robot picker · scenario tab · Time Series · incidents · runtime · events
 SHADOW PHYSICS  second MuJoCo instance per robot/scenario, used for FK and torque prediction
```

Design rules carried over from Phase 1 and kept in Phase 2:

- **Ground truth is just another source.** The simulator's physics, a motion-capture system, or nothing.
  The platform must work, and say what it can and cannot conclude, when only reported state exists.
- **Adapters never reach the core.** They emit events with the schema in
  `backend/pipeline/events/event_model.py`; new event types are added there, never bypassed.
- **Expected state is intent, never observation.** Task steps and machine preconditions extend
  `backend/pipeline/twin/expected_state.py`; they do not read physics.
- **No claimed root causes.** Candidates stay labelled Observed / Inferred / Possible cause.

---

## 3. Item 1 — Robot selection

### Registry

`backend/conf/registry/robots.yaml`, one entry per supported arm. Initial entries: `franka_panda` (current),
`franka_fr3`, `ur5e`, `ur10e`. All are available in MuJoCo Menagerie under Apache 2.0 or BSD licences.

| field | purpose | today's equivalent |
|---|---|---|
| `mjcf`, `assets` | Menagerie path and mesh dir | `backend/io/sim/assets/franka_emika_panda/` |
| `joints[]`, `actuators[]` | ordered joint and actuator names | `JOINTS` in `backend/app/config.py` |
| `joint_limits`, `velocity_limits`, `effort_limits` | per-joint physical limits | `JOINT_LIMITS`, `EFFORT_LIMITS` |
| `ee_body`, `ee_offset` | where the tool-centre-point site is injected | `ee_site` added by sed to `panda_tron.xml` |
| `wrist_camera` | mount body, pose, fov | `wrist_cam` in `panda_tron.xml` |
| `home_keyframe`, `ik_posture` | start pose and IK nullspace bias | keyframe `home`, `bake_waypoints.py` |
| `gripper` | actuator, open/closed values | hard-coded `ctrl[7]` 0 / 255 |
| `ros2` | controller name, command topic, joint-state topic | `COMMAND_TOPIC`, `SECURITY` in config |
| `thresholds` | robot-specific overrides of `SAFETY` (velocity limit, tracking notice/alert, stall time) | global `SAFETY` |

### Scene composition

`backend/io/sim/assets/scene.xml` becomes a template. A composer (`backend/io/sim/compose.py`) writes a
per-session scene: robot include, pedestal height, TCP site, wrist camera, plus the scenario's assets
(§4). This replaces the one-off sed edits used to produce `panda_tron.xml`.

### IK baking per robot

`backend/io/sim/bake_waypoints.py` already solves damped-least-squares IK against the loaded model. It becomes
`bake(robot_id, scenario_id)` and caches results in `backend/data/waypoints/<robot>/<scenario>.json`. Targets come
from the scenario (§4), not from a constant table.

### Platform and UI

- `backend/app/config.py` keeps only platform-level settings; robot values come from the registry entry of the
  active session.
- New session endpoint: `POST /api/session {robot_id, scenario_id}` stops the adapter and renderer,
  recomposes the scene, rebakes if needed, and restarts with a fresh event namespace.
- UI: a robot dropdown in the header next to the title. `frontend/robot/simview.js` reads joint limits from
  the twin's identity block instead of its `LIMITS` constant.

Acceptance: pick UR5e; the cell reloads with the UR5e on the pedestal; the cycle completes; scenarios A–E
each produce their expected detection.

---

## 4. Item 2 — Scenarios with tasks (default: CNC tending)

### Scenario file

`backend/conf/scenarios/defs/cnc_tending.yaml` (YAML in repo, editable from the UI):

```yaml
id: cnc_tending
name: CNC machine tending
assets:
  robot: {ref: franka_panda, pose: [0, 0, 0]}
  cnc_machine: {ref: cnc_vmc_small, pose: [0.9, 0.0, 0], door: sliding_left}
  raw_blocks: {ref: metal_block_60mm, count: 3, tray: raw_tray, pose: [0.45, -0.45, 0]}
  finished_tray: {ref: tray, pose: [0.45, 0.45, 0]}
  human: {ref: humanoid_operator, behaviour: operator_patrol}
zones:
  - {id: cnc_interior, type: restricted_while, condition: cnc.state == RUNNING}
  - {id: collab_front, type: reduced_speed, max_tcp_speed: 0.25}
  - {id: operator_side, type: ssm, min_separation: 0.5}
task:
  loop: true
  steps:
    - {id: wait_cycle,    action: wait,  precondition: cnc.state == COMPLETE, timeout_s: 120}
    - {id: open_door,     action: machine, command: door_open, expect: cnc.door == OPEN, duration_s: 3}
    - {id: grasp_part,    action: move,  target: chuck_part, gripper: close, duration_s: 4}
    - {id: unload,        action: move,  target: finished_tray.next, gripper: open, duration_s: 5}
    - {id: pick_raw,      action: move,  target: raw_tray.next, gripper: close, duration_s: 5}
    - {id: load,          action: move,  target: chuck, duration_s: 5}
    - {id: clamp,         action: machine, command: clamp, expect: cnc.chuck == CLAMPED, duration_s: 1}
    - {id: retreat,       action: move,  target: home, gripper: open, duration_s: 4}
    - {id: close_door,    action: machine, command: door_close, expect: cnc.door == CLOSED, duration_s: 3}
    - {id: start_cycle,   action: machine, command: cycle_start, expect: cnc.state == RUNNING, duration_s: 1}
faults:
  A: {step: unload, params: {speed_factor: 2.5}}
  B: {step: any,    params: {rogue_node: /teleop_override, goal: operator_side}}
  C: {step: any,    params: {encoder_offset: {joint: 1, rad: 0.25}}}
  D: {step: load,   params: {actuator_gain_scale: 0.05, joints: [1, 3]}}
  E: {step: load,   params: {goal: door_frame, disable_collision_check: true}}
  F: {step: grasp_part, params: {machine_state_spoof: COMPLETE}}   # robot enters while spindle runs
```

### Scenario and task engine (`backend/conf/scenarios/`)

- `loader.py` validates YAML (pydantic models) and resolves asset references against the registries.
- `task_runner.py` replaces the fixed waypoint loop in `backend/io/collectors/mujoco_adapter.py`
  (`_planner_step`, `cycle` list). It plays the role of the **motion planner node**: for each step it
  emits `TaskStepStarted`, waits for preconditions from the machine twin, issues a trajectory
  (`TrajectoryGoalReceived` + `CommandReceived`, unchanged) or a machine command
  (`MachineCommandReceived`), and emits `TaskStepCompleted` / `TaskStepFailed`.
- Faults bind to steps, so a scenario can say "inject D during `load`" rather than "now".

### CNC machine

- **Geometry:** MJCF enclosure with a sliding door on an actuated slide joint, a chuck (two jaws on a slide
  joint), a spindle prop, and a work envelope volume used by the `cnc_interior` zone.
- **Behaviour:** `backend/io/machines/cnc.py`, a PLC-style state machine
  (`IDLE → RUNNING → COMPLETE → DOOR_OPEN → LOADING → IDLE`, plus `ALARM`) with interlocks:
  cycle cannot start unless the door is closed and the chuck is clamped; the door cannot open while
  `RUNNING`; opening the door during `RUNNING` raises `ALARM`.
- **Exposure:** `backend/io/collectors/machine_adapter.py` presents the machine as an OPC UA-shaped node set
  (`Machine/State`, `Machine/Door`, `Machine/Chuck`, `Machine/CycleCount`, `Machine/Alarm`) and emits
  `MachineStateObserved` on change (and at 1 Hz), `MachineCommandReceived` for every command with its
  source, and `InterlockViolated` when a command is rejected. In simulation the "server" is in-process;
  for a real machine the same adapter wraps an `asyncua` or `pymodbus` client with a node/register map
  declared in the scenario's `machines:` section. Nothing downstream changes.

### Workpieces

Metal blocks are free bodies. Grasping toggles a MuJoCo `equality` weld between the gripper and the block
so the block travels with the tool. Block mass therefore changes the arm's load, which the shadow
simulation (§8) needs in order to predict torques correctly.

### Task-level expected state

`backend/pipeline/twin/expected_state.py` gains a `TaskExpectation`: current step, its start time, the expected
completion window, expected next step, and the machine state the step assumes. The twin gets a
`machine` entity (state, door, chuck, alarm, last command, source) and a `task` block.

New rules in `backend/pipeline/risk/task_rules.py`:

| rule | fires when | category |
|---|---|---|
| `TASK_STEP_TIMEOUT` | a step exceeds its window (e.g. `wait_cycle` beyond 120 s) | safety |
| `TASK_ORDER_VIOLATION` | a step starts whose predecessor has not completed | safety |
| `INTERLOCK_VIOLATION` | machine rejected a command, or the robot entered `cnc_interior` while `RUNNING` | safety |
| `MACHINE_STATE_MISMATCH` | the robot's software acts on a machine state that differs from the machine's own report (fault F) | safety |
| `UNEXPECTED_MACHINE_COMMAND_SOURCE` | a machine command from a client not on the allow-list | security |

Reconstruction (`backend/pipeline/incidents/reconstruction.py`) gains: the task step at first divergence, machine
state transitions in the causal timeline, and candidates such as *"door opened while cycle RUNNING —
command came from the planner 1.2 s after a spoofed COMPLETE"*.

### UI

The **Faults** tab becomes **Scenarios**: picker, asset list, task step list with live progress and
per-step expected/actual duration, fault buttons bound to steps, and an editor for step durations and
zone parameters that writes back through `PUT /api/scenarios/{id}` (validated, then saved to the YAML).

---

## 5. Item 3 — Tab styling

`frontend/app.css` only.

- The peach highlight comes from the generic `button.active` rule (meant for fault buttons) leaking onto
  `.tabs` and `.viz-tabs`. Scope that rule to `.scen button.active`.
- Active tab indicator: a filled black square (`::before`, 8 × 8 px, `--fg`, vertically centred, 6 px gap)
  in place of the blue underline. Active label colour `--fg`, weight 600; inactive `--dim`, weight 500.
- Same treatment for the visualizer's Environment / Robot camera tabs.

---

## 6. Item 4 — Time Series tab (merges Divergence and Timeline)

One tab replaces two.

**Layout, top to bottom, all sharing one time axis and one scrubber:**

1. Seven small-multiple charts, one per joint: expected angle (green) and actual angle (blue) with the
   divergence band shaded when |Δq| exceeds the notice threshold and red beyond the alert threshold.
2. End-effector distance from expected, and reported-vs-physical encoder error.
3. Peak joint speed (actual vs expected) with the velocity limit line.
4. Risk band with anomaly, incident, fault-injection and task-step markers.
5. The current-value joint table (existing `frontend/twin/expected_actual.js`), driven by the scrubbed
   instant in history mode or the live twin otherwise.

**Backend:** `track()` in `backend/pipeline/incidents/timeline.py` returns per-joint arrays (`q[7]`, `q_exp[7]`,
`qd[7]`, `err[7]`) alongside today's scalars, and `markers()` includes `TaskStepStarted`. Snapshots already
hold everything needed; no new storage.

**Frontend:** `frontend/timeline/timeline.js` keeps its scrub, window and marker logic; the small multiples
are one draw function called in a loop with a per-joint y-range. Canvas height grows to fit the sidebar's
scroll area; the sidebar width stays 520 px.

---

## 7. Item 5 — Realistic human

- Replace the capsule-and-sphere mocap body with MuJoCo's articulated humanoid: pelvis, torso, head, two
  arms (shoulder, elbow), two legs (hip, knee, ankle). It is kinematic: a mocap root drives position and
  heading; joint angles are set from scripted poses (standing, walking gait cycle, reaching). No external
  assets, runs on the current install. A mesh-based body is a Phase 3 option.
- Behaviours declared per scenario (`operator_patrol`: stand at the finished tray, walk to the CNC door,
  reach toward the chuck, walk back). Behaviour timing is also a **task expectation** for the human, so the
  scenario can express "operator should not be at the door while `RUNNING`".
- Proximity: `_nearest_human()` in `backend/io/collectors/mujoco_adapter.py` becomes the minimum
  `mj_geomDistance` between every arm geom and every humanoid geom, so a reaching forearm counts.
- The human becomes a **sensed** quantity as well as ground truth: a simulated safety area scanner and a
  light curtain at the door emit `SensorObservation` events (`sensor: area_scanner` /
  `light_curtain`). Ground truth vs sensor gives a new sensor-divergence case: the scanner failed to
  report a person who was physically inside the field.

---

## 8. Additional capabilities

### 8.1 Log replay (rosbag2 / MCAP)

The main way an OEM investigates a field incident.

- `backend/io/collectors/replay_adapter.py` reads a bag or MCAP file with `rosbag2_py` or the `mcap` package
  (no ROS install needed for MCAP), maps topics to events with the same mapping the live collector uses
  (`/joint_states` → `PositionObserved`, trajectory topic → `CommandReceived`, controller state →
  `CommandExecuted`, TF, machine tags → `MachineStateObserved`), and preserves original timestamps.
- Playback modes: real-time, N×, or as fast as possible. The UI shows a REPLAY banner and the timeline
  spans the log.
- Ground truth is optional: with a motion-capture or independent-measurement topic mapped, physical
  state is filled; without it, the twin compares expected vs *reported* state and the reconstruction says
  so explicitly.
- Incident bundles: `GET /api/incidents/{id}/bundle.zip` packs events, snapshots, the reconstruction and
  render frames at key timestamps for hand-off to another team.
- CI keeps replay fixtures in `tests/fixtures/*.mcap`; a test asserts the same incident title and top
  candidate as the live scenario run that produced the fixture.

### 8.2 Physics-based expected state (shadow simulation)

Today expected state is kinematic. A second MuJoCo instance (`backend/pipeline/twin/shadow_sim.py`, loaded like
`backend/pipeline/twin/kinematics.py`) runs the *accepted* trajectory with the scenario's payload (block mass when
grasped) and predicts joint torques and contacts.

- New divergence: `torque_residual[7]` = observed effort − predicted effort.
- New rule `UNEXPECTED_LOAD`: residual above a per-joint threshold, sustained for 0.3 s, with no commanded
  contact. This is model-based collision detection as real collaborative arms implement it, and it works
  from reported effort alone, so it applies to replayed field logs without any contact sensor.
- Reconstruction candidate: *"unexpected external load on joint N"* with the residual trace as evidence,
  ranked against actuator-degradation and command-source candidates by timing as today.

### 8.3 Machine and PLC integration

Covered by §4 for the simulated path. For real machines:

- `backend/conf/scenarios/defs/*.yaml` `machines:` section declares the protocol (`opcua` or `modbus`), endpoint, and a
  node-id / register map onto the same logical tags (`state`, `door`, `chuck`, `alarm`, `cycle_count`).
- Security rules extend to machine traffic: `UNEXPECTED_MACHINE_COMMAND_SOURCE` and
  `UNEXPECTED_OPCUA_CLIENT` (a client session not on the allow-list), mirroring the ROS graph rules in
  `backend/pipeline/risk/security_rules.py`.

### 8.4 Safety zones and speed-and-separation monitoring

Zones move from `ENVIRONMENT["forbidden_zones"]` in code to the scenario file, with three types:

| type | rule | note |
|---|---|---|
| `restricted` / `restricted_while` | `WORKSPACE_VIOLATION` (existing) with a machine-state condition | e.g. CNC interior while RUNNING |
| `reduced_speed` | `REDUCED_SPEED_ZONE_VIOLATION`: TCP speed above the zone limit while inside | collaborative front area |
| `ssm` | `SSM_VIOLATION`: separation to the human below `f(tcp_speed)` (protective separation distance in the style of ISO/TS 15066) | thresholds per robot profile |

A protective-stop expectation ties the sensed human to the controller: when the scanner or light curtain
reports intrusion, the expected state becomes "stopping within *t* ms". `PROTECTIVE_STOP_NOT_ISSUED`
fires when motion continues. Standards are referenced as design guidance for rule shape, not as a
compliance claim.

---

## 9. Cross-cutting requirements for OEM use

| area | what changes |
|---|---|
| Config as code | `backend/conf/cells/<cell>.yaml` binds robot, scenario, thresholds and allow-lists; everything the UI edits is written back to files, so a validation setup is reviewable in git |
| Baseline capture | `backend/conf/security/baseline.py` records a "golden run" and generates the ROS graph, publisher, rate and OPC UA client allow-lists that are hand-written today |
| Sessions | one platform instance hosts several sessions (cells); events are already keyed by entity, the twin becomes a map of entities |
| Retention and export | configurable snapshot retention (today 1 h in memory of SQLite), incident bundles, CSV/Parquet export of events |
| Deployment | Docker image and `docker-compose.yml` (platform; optional ROS 2 collector image); `/healthz`; all settings via environment or mounted config |
| Access | local users, API tokens for adapters, audit log of operator actions (fault injection is already recorded as `operator` events) |
| ROS 2 collector | per-message publisher attribution using DDS participant GIDs where the middleware exposes them; SROS2 identities when enabled; exercised in CI with a ROS 2 Jazzy container |
| Testing | scenario-driven integration tests (run each fault, assert detection and top candidate), replay-fixture tests, registry tests per robot |

---

## 10. Milestones

Each milestone is demoable on its own and leaves the current demo working.

| milestone | scope | items |
|---|---|---|
| M1 | Tab styling; Time Series tab | 3, 4 |
| M2 | Robot registry, scene composer, per-robot IK, robot picker | 1 |
| M3 | Articulated humanoid, behaviours, scanner and light-curtain sensing | 5 |
| M4 | Scenario loader, task runner, CNC model, machine adapter, task rules, Scenarios tab | 2, 8.3 (sim) |
| M5 | Replay adapter, incident bundles, replay fixtures | 8.1 |
| M6 | Shadow simulation, torque residual, `UNEXPECTED_LOAD` | 8.2 |
| M7 | Zones in YAML, reduced-speed and SSM rules, protective-stop expectation | 8.4 |
| M8 | Config as code, baseline capture, sessions, deployment, access, retention/export | 9 |

Suggested order: M1 → M2 → M4 → M3 → M5 → M6 → M7 → M8. M4 before M3 because the humanoid's
behaviours reference task steps and the CNC door.

---

## 11. Open questions

1. **First real-robot interface.** ROS 2 with `ros2_control` is assumed. If the OEM's arms expose a native
   API (e.g. an RTDE-style or vendor SDK stream) before ROS 2, a native adapter should be scheduled in M5.
2. **CNC vendor.** The OPC UA node set is generic; a target controller family (e.g. Fanuc FOCAS, Siemens
   828D/840D, Haas) would fix the tag map and the alarm semantics.
3. **Ground truth on real cells.** Which independent measurement, if any, will be available (motion capture,
   external cameras, none)? This decides how much of the safety rule set can run on reported state alone.
4. **Model licensing.** Menagerie models are Apache 2.0 / BSD; confirm redistribution terms for a shipped
   product and whether OEM CAD/URDF should replace them.
5. **Retention and data ownership.** How long validation runs are kept and whether logs leave the site.

---

## Appendix A — New event types

`TaskStepStarted`, `TaskStepCompleted`, `TaskStepFailed`, `MachineStateObserved`,
`MachineCommandReceived`, `InterlockViolated`, `ReplayStarted`, `ReplayFinished`. All use the existing
`Event` schema in `backend/pipeline/events/event_model.py`.

## Appendix B — New rules

Safety: `TASK_STEP_TIMEOUT`, `TASK_ORDER_VIOLATION`, `INTERLOCK_VIOLATION`, `MACHINE_STATE_MISMATCH`,
`UNEXPECTED_LOAD`, `REDUCED_SPEED_ZONE_VIOLATION`, `SSM_VIOLATION`, `PROTECTIVE_STOP_NOT_ISSUED`,
scanner/light-curtain `SENSOR_STATE_DIVERGENCE` cases.
Security: `UNEXPECTED_MACHINE_COMMAND_SOURCE`, `UNEXPECTED_OPCUA_CLIENT`.

## Appendix C — Phase 1 code the plan builds on

| capability | file |
|---|---|
| unified events, store, bus | `backend/pipeline/events/event_model.py`, `backend/pipeline/events/event_store.py` |
| twin, state engine, expected state, FK | `backend/pipeline/twin/robot_model.py`, `backend/pipeline/twin/state_engine.py`, `backend/pipeline/twin/expected_state.py`, `backend/pipeline/twin/kinematics.py` |
| rules | `backend/pipeline/risk/safety_rules.py`, `backend/pipeline/risk/security_rules.py`, `backend/pipeline/risk/anomaly_engine.py` |
| incidents, timeline, reconstruction | `backend/pipeline/incidents/incident_engine.py`, `backend/pipeline/incidents/timeline.py`, `backend/pipeline/incidents/reconstruction.py` |
| simulator, renderer, IK | `backend/io/collectors/mujoco_adapter.py`, `backend/io/sim/renderer.py`, `backend/io/sim/bake_waypoints.py` |
| faults | `backend/pipeline/faults/scenarios.py`, `backend/pipeline/faults/manager.py` |
| API | `backend/app/api/platform.py`, `backend/app/api/routes.py` |
| UI | `frontend/app.js`, `frontend/app.css`, `frontend/timeline/timeline.js`, `frontend/twin/expected_actual.js`, `frontend/robot/simview.js` |
| ROS 2 | `backend/io/ros2/tron_ros2_collector/` |
