Build a local-first prototype of a **Cyber-Physical Digital Twin for a robot using NVIDIA Isaac Sim + ROS 2**.

This is the real prototype of the concept we discussed.

The core question is:

> **When a robot behaves unexpectedly, can we reconstruct exactly what happened by comparing the robot's expected physical/software state against its observed state?**

The product is NOT a robotics controller, simulator, or generic robot dashboard.

The product is a **security + safety observability layer** sitting on top of a simulated robot.

---

# 1. Core Architecture

Build this architecture:

```text
                         NVIDIA ISAAC SIM
                    ┌─────────────────────────┐
                    │                         │
                    │  Physics                │
                    │  Robot                  │
                    │  Sensors                │
                    │  Environment             │
                    │                         │
                    └────────────┬────────────┘
                                 │
                         ROS 2 BRIDGE
                                 │
                    ┌────────────▼────────────┐
                    │                         │
                    │   ROS 2 EVENT LAYER     │
                    │                         │
                    │ topics                  │
                    │ commands                │
                    │ sensor observations     │
                    │ transforms              │
                    │ controller state        │
                    │ node state              │
                    │ network metadata        │
                    │                         │
                    └────────────┬────────────┘
                                 │
                                 ▼
                    ┌─────────────────────────┐
                    │    YOUR PLATFORM        │
                    │                         │
                    │ Event Collector         │
                    │ State Engine             │
                    │ Digital Twin            │
                    │ Expected State          │
                    │ Risk Engine              │
                    │ Incident Reconstruction │
                    └────────────┬────────────┘
                                 │
                                 ▼
                    ┌─────────────────────────┐
                    │        WEB UI            │
                    │                         │
                    │ Robot Twin              │
                    │ Live State              │
                    │ Expected vs Actual      │
                    │ Risk                    │
                    │ Incidents               │
                    │ Timeline                │
                    └─────────────────────────┘
```

Isaac Sim should be the **source of physical ground truth**.

ROS 2 should be the **source of software/runtime state**.

Your platform should correlate the two.

---

# 2. Use NVIDIA Isaac Sim

Use the current NVIDIA Isaac Sim release available in the development environment.

Use the official Isaac Sim ROS 2 bridge.

Prefer:

- ROS 2 Jazzy
- Ubuntu 24.04
- Isaac Sim
- Python
- ROS 2 topics/services/actions

Isaac Sim supports ROS 2 Jazzy and Humble and provides the ROS 2 bridge for publishing/subscribing to ROS topics.

Do NOT attempt to recreate physics yourself.

Do NOT build a custom simulator.

Isaac Sim is the simulation engine.

---

# 3. Robot

Start with ONE simple non-humanoid robot.

Prefer a differential-drive mobile robot such as the Clearpath Dingo example available in Isaac Sim.

The exact robot is less important than having:

- position
- orientation
- velocity
- wheels
- lidar
- camera
- collision geometry
- ROS 2 control
- navigation

Isaac Sim's ROS 2 examples already demonstrate this kind of URDF → physics → sensor → ROS 2 workflow.

Keep the initial environment extremely simple:

```text
┌───────────────────────────────┐
│                               │
│          WALL                 │
│                               │
│    ┌───────┐                  │
│    │       │                  │
│    │ ROBOT │       OBSTACLE   │
│    │       │                  │
│    └───────┘                  │
│                               │
│                 HUMAN         │
│                               │
└───────────────────────────────┘
```

The goal is not realism.

The goal is demonstrating **state divergence**.

---

# 4. Digital Twin

Create a persistent digital representation of the robot.

The twin should maintain:

```text
Robot
├── Identity
├── Physical State
│   ├── position
│   ├── orientation
│   ├── velocity
│   ├── acceleration
│   └── battery/state
│
├── Sensors
│   ├── lidar
│   ├── camera
│   └── IMU
│
├── Actuators
│   ├── left wheel
│   └── right wheel
│
├── Software
│   ├── ROS nodes
│   ├── topics
│   ├── controllers
│   └── navigation
│
├── Network
│   ├── ROS participants
│   ├── connections
│   └── message activity
│
└── Expected State
```

The twin should have two distinct concepts:

### Observed State

What the system actually reported / what Isaac Sim physically did.

### Expected State

What the robot was supposed to do.

Never blur these together.

---

# 5. Event Model

Make events first-class objects.

At minimum:

```text
RobotStateObserved
SensorObservation
VelocityObserved
PositionObserved
CommandReceived
CommandExecuted
ControllerStateChanged
ROSNodeStarted
ROSNodeStopped
ROSTopicObserved
CollisionDetected
SafetyThresholdExceeded
NavigationGoalReceived
NavigationGoalReached
NetworkConnectionObserved
AnomalyDetected
IncidentCreated
IncidentUpdated
```

Every event should contain:

```json
{
  "event_id": "...",
  "timestamp": "...",
  "event_type": "...",
  "source": "...",
  "entity_id": "...",
  "payload": {},
  "confidence": 1.0
}
```

Keep the event schema generic enough that later we can ingest real robots.

---

# 6. Expected State Engine

Create a simple deterministic expected-state model.

For example:

```text
Robot receives:

MOVE_FORWARD
velocity = 0.5 m/s
duration = 5 seconds
```

Expected:

```text
velocity ≈ 0.5 m/s
heading ≈ constant
trajectory ≈ straight
position ≈ predicted trajectory
```

Actual Isaac Sim state:

```text
velocity = 0.5 m/s
heading = changed
trajectory = deviated
```

The system should detect:

```text
EXPECTED ≠ ACTUAL
```

Do NOT begin with ML.

Use deterministic physics/state comparisons first.

---

# 7. Safety Monitoring

Create a basic safety/risk engine.

Examples:

### Velocity

```text
Expected max velocity: 0.5 m/s
Observed: 0.85 m/s

→ VELOCITY_LIMIT_EXCEEDED
```

### Workspace

```text
Robot enters forbidden zone

→ WORKSPACE_VIOLATION
```

### Human proximity

```text
Human distance < safety threshold

→ PROXIMITY_RISK
```

### Collision

```text
Collision detected

→ COLLISION_EVENT
```

### Sensor disagreement

For example:

```text
Lidar:
obstacle = 1.2m

Simulation ground truth:
obstacle = 0.5m

→ SENSOR_STATE_DIVERGENCE
```

This distinction is important:

**Isaac Sim knows ground truth.**

Your system should pretend it does NOT know ground truth and treat the simulator's physical state as an external reality against which the software's beliefs can be compared.

---

# 8. Security Monitoring

Now add the security side.

Monitor the ROS 2 runtime for unexpected behavior.

Examples:

```text
Unexpected ROS node
Unexpected topic publisher
Unexpected topic subscriber
Unexpected command source
Unexpected message rate
Unexpected connection
Unexpected controller command
```

Example:

```text
Expected:

navigation_node
      ↓
cmd_vel
      ↓
controller

Observed:

navigation_node
      ↓
cmd_vel
      ↓
controller

unknown_node
      ↓
cmd_vel
```

Generate:

```text
UNEXPECTED COMMAND SOURCE
```

Do not implement exploitation or offensive functionality.

We are detecting unexpected behavior, not attacking the system.

---

# 9. The Key Feature: Expected vs Actual

This should become the centerpiece of the UI.

Create a view like:

```text
ROBOT #001

STATUS
──────────────────────────
Operational
Risk: HIGH
Active incidents: 1


EXPECTED STATE
──────────────────────────
Velocity       0.50 m/s
Heading        90°
Position       (4.2, 2.1)
Goal           (8.0, 2.1)
Controller     Nav2


ACTUAL STATE
──────────────────────────
Velocity       0.83 m/s
Heading        104°
Position       (4.9, 1.7)
Goal           (8.0, 2.1)
Controller     Nav2


DIVERGENCES
──────────────────────────
Velocity       +66%
Heading        +14°
Position       0.73m
```

This is the fundamental product experience.

---

# 10. Incident Reconstruction

This is the most important feature.

When something goes wrong, reconstruct the entire causal timeline.

Example:

```text
INCIDENT #0007
Unexpected Robot Motion

14:03:21.104
Navigation goal received

14:03:21.112
cmd_vel published
Expected velocity: 0.5 m/s

14:03:21.115
Controller receives command

14:03:21.119
Wheel velocity increases

14:03:21.145
Actual velocity: 0.62 m/s

14:03:21.200
Actual velocity: 0.83 m/s

14:03:21.205
Safety threshold exceeded

14:03:21.210
Robot enters restricted zone

14:03:21.220
Collision detected
```

Then summarize:

```text
EXPECTED
Robot should travel forward
at 0.5 m/s.

ACTUAL
Robot accelerated to 0.83 m/s,
deviated from its planned trajectory,
entered a restricted zone,
and collided with an obstacle.

FIRST DETECTED DIVERGENCE
14:03:21.145

ROOT-CAUSE CANDIDATES
1. Controller output divergence
2. Unexpected velocity command
3. Sensor/controller disagreement
```

Important:

Do NOT claim "root cause" unless the evidence actually establishes it.

Use:

**Observed**
**Inferred**
**Possible cause**

---

# 11. Deliberately Inject Failures

The demo needs controlled failure scenarios.

Create a safe simulation-only fault injection system.

Do NOT randomly corrupt the system.

Create explicit scenarios:

### Scenario A — Excessive velocity

Robot receives:

```text
expected = 0.5 m/s
actual = 0.9 m/s
```

Expected result:

```text
VELOCITY_LIMIT_EXCEEDED
```

### Scenario B — Unexpected command source

Introduce a simulated ROS node that publishes an unexpected command.

Expected:

```text
UNEXPECTED_COMMAND_SOURCE
```

### Scenario C — Sensor disagreement

Create a discrepancy between simulated sensor observation and physical ground truth.

Expected:

```text
SENSOR_STATE_DIVERGENCE
```

### Scenario D — Navigation deviation

Robot receives a valid navigation goal but physically deviates from expected trajectory.

Expected:

```text
TRAJECTORY_DIVERGENCE
```

### Scenario E — Collision

Robot collides with an obstacle.

Expected:

```text
COLLISION_EVENT
```

The user should be able to trigger each scenario from the UI or CLI.

---

# 12. Digital Twin Timeline

Maintain historical state.

The system should allow:

```text
CURRENT
│
├── 14:03:20
├── 14:03:21
├── 14:03:22
├── 14:03:23
└── 14:03:24
```

Selecting a timestamp should reconstruct:

```text
Robot state
ROS state
Sensor state
Commands
Risk state
Environment state
```

This is critical.

The twin should not simply represent:

> "What does the robot look like now?"

It should represent:

> **"What was the state of the robot and its software at any point in time?"**

---

# 13. Visualization

Build a web UI.

### Main screen

Isaac Sim visualization on one side.

Digital twin state on the other.

```text
┌──────────────────────┬──────────────────────┐
│                      │ ROBOT STATE          │
│                      │                      │
│    ISAAC SIM         │ Position             │
│                      │ Velocity             │
│       🤖             │ Sensors              │
│                      │ Controller           │
│                      │                      │
│                      │ Risk: MEDIUM         │
├──────────────────────┼──────────────────────┤
│ EXPECTED             │ ACTUAL               │
│                      │                      │
│ Velocity: 0.5        │ Velocity: 0.7        │
│ Heading: 90°         │ Heading: 97°         │
│ Position: ...        │ Position: ...        │
└──────────────────────┴──────────────────────┘
```

Do not spend excessive time making the UI beautiful.

Prioritize:

1. state
2. divergence
3. timeline
4. incident reconstruction

---

# 14. Architecture

Use a modular architecture.

```text
/backend
  /collectors
    ros2_collector
    isaac_state_collector

  /events
    event_model
    event_store

  /twin
    robot_model
    state_engine
    expected_state

  /risk
    safety_rules
    security_rules
    anomaly_engine

  /incidents
    incident_engine
    timeline
    reconstruction

  /api

/frontend
  /robot
  /twin
  /incidents
  /timeline
  /risk
```

Use SQLite initially.

Do NOT introduce Kafka/Kubernetes/cloud infrastructure.

But make the event model compatible with eventually moving to a streaming backend.

---

# 15. Important Architectural Principle

The product should NOT be tightly coupled to Isaac Sim.

Isaac Sim is the **simulation adapter**.

Eventually:

```text
                  YOUR PLATFORM
                       │
          ┌────────────┼────────────┐
          │            │            │
      Isaac Sim      ROS 2       Real Robot
       Adapter      Adapter        Adapter
          │            │            │
          └────────────┼────────────┘
                       │
                 Unified Twin
```

The core platform should not care whether an event came from:

```text
Isaac Sim
ROS 2
real hardware
robot OEM API
```

It should consume normalized events.

---

# 16. Realism About Ground Truth

This is extremely important.

Isaac Sim gives us information unavailable on a real robot:

```text
true position
true velocity
true collision state
true object positions
true physical state
```

Use that advantage during development.

But architect the platform as though Isaac Sim's ground truth is simply another observation source.

For example:

```text
GROUND_TRUTH
    ↓
Isaac Adapter

ROS OBSERVATION
    ↓
ROS Adapter

COMMAND
    ↓
Command Adapter

        ↓

UNIFIED EVENT MODEL

        ↓

DIGITAL TWIN

        ↓

EXPECTED vs ACTUAL
```

This will make the eventual transition to real robots much easier.

---

# 17. MVP Acceptance Criteria

The prototype is successful when I can:

1. Launch Isaac Sim.
2. Launch ROS 2.
3. Spawn a robot.
4. See the robot physically move.
5. See its ROS 2 state being collected.
6. See the robot represented in the digital twin.
7. Establish an expected state.
8. Inject a controlled failure.
9. Detect the divergence.
10. Generate an incident.
11. Reconstruct the timeline.
12. Show expected vs actual state.
13. Identify the first observed divergence.
14. Show the evidence behind the anomaly.

The killer demo should look like:

```text
NORMAL

Robot follows expected trajectory.

        ↓

FAULT INJECTION

Something changes.

        ↓

OBSERVATION

ROS 2 + Isaac Sim report state.

        ↓

DIVERGENCE

Expected ≠ Actual.

        ↓

RISK EVENT

Safety/security rule triggered.

        ↓

INCIDENT

System automatically reconstructs:

"What happened?"

"When did it start?"

"What changed?"

"What did the robot believe?"

"What actually happened?"

"What evidence do we have?"
```

---

# 18. Development Instructions

Before writing code:

1. Inspect the existing repository.
2. Understand its architecture and existing primitives.
3. Do not rewrite existing infrastructure unnecessarily.
4. Identify the minimum new components required.
5. Verify the installed environment and Isaac Sim/ROS 2 compatibility.
6. Create a minimal Isaac Sim + ROS 2 proof of life first.
7. Then build the event collector.
8. Then the digital twin.
9. Then expected-vs-actual comparison.
10. Then risk detection.
11. Then incident reconstruction.
12. Then the UI.

Keep the implementation incremental.

After each major step, make sure the system actually runs before proceeding.

Do not build mock Isaac Sim data if the real Isaac Sim environment is available.

Use actual Isaac Sim state and ROS 2 messages wherever possible.

The ultimate objective is not "a robot simulation."

The objective is:

> **A system that can explain what happened when a physical robot's observed behavior diverges from its expected behavior.**