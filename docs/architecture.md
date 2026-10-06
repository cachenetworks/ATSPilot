# Architecture

```
ATSPilot/
├── external/scs_sdk_1_15/     SCS SDK headers (vendored, SCS licence)
├── src/core/                  static library, no SDK or Windows dependencies
│   ├── math/                  Vec2/Vec3/Quat, angles, curvature, Hermite, coordinate transforms
│   ├── path/                  arc-length polyline: projection, sampling, trimming
│   ├── control/               PID, Pure Pursuit/Stanley, steering shaper, speed planner, pedal controller
│   ├── pilot/                 VehicleState, Autopilot state machine, safety (override, watchdog)
│   ├── map/                   HashFS, SII, sector/prefab parsers, lane graph, localizer, path builder, MapService
│   ├── config/                TOML subset parser, validated Config
│   ├── sim/                   kinematic vehicle simulator and scenario runner
│   └── util/                  async rotating logger, telemetry recorder, key bindings
├── src/plugin/                atspilot.dll: SDK glue only (Telemetry.cpp, Input.cpp, Runtime.cpp)
├── src/tools/                 atspilot_sim, atspilot_mapdump, atspilot_plugin_host
└── tests/                     doctest unit, controller and integration tests
```

Version-dependent code is confined to the plugin's SDK files, which check API
versions, and to the map parsers, which check `.base`/`.ppd` versions. The
control algorithms know nothing about game versions.

## Data flow

```
                  game main thread                                   background threads
 ┌──────────────────────────────────────────────────────┐   ┌──────────────────────────────────┐
 │ telemetry channels ─▶ pending VehicleState            │   │ MapService                        │
 │ frame_end:                                            │   │  load cache / build map (once)    │
 │   VehicleState ──────────────── submitVehicle() ─────────▶│  every ~100 ms:                   │
 │   hotkeys ─▶ Autopilot::request()                      │   │   Localizer (lane match)          │
 │                                                       │   │   job destination ─▶ async A*     │
 │                                                       │   │     route (recalc when off-route) │
 │   Autopilot::update(state, latest PathSnapshot) ◀────────│   buildPlannedPath (route / road)  │
 │     safety ▶ lateral ▶ speed plan ▶ pedals ▶ command   │   │   publish shared_ptr<PathSnapshot>│
 │   recorder / status (queued to workers)               │   └──────────────────────────────────┘
 │ input device callback (next frame):                   │   Logger thread: file I/O, rotation
 │   currentOutput() ─▶ semantical steering/aforward/... │   Recorder thread: CSV
 └──────────────────────────────────────────────────────┘   Worker: status.json, audio cues
```

Separation follows the pipeline: **perception** (telemetry → VehicleState,
map → localization), **planning** (path builder, speed planner), **control**
(lateral controller, steering shaper, pedal controller), **safety** (override
detector, watchdog, deviation limits, emergency stop) and **output**
(semantical input device with command expiry).

## Threading rules

- SDK callbacks never block. The control step is a few hundred microseconds
  (projection on a ≤ 700 m path plus scalar maths).
- The main thread exchanges data with the planner through two short mutex
  sections: copying a VehicleState in, and copying a `shared_ptr` out.
  Snapshots are immutable once published.
- Logging, CSV recording, status writing and audio run on their own threads
  with bounded queues. When a queue is full, messages are dropped rather than
  blocking the game.
- Shutdown joins every thread. A plugin `sdk reload` therefore leaves nothing
  running.

## Update rates

| Component | Rate |
|---|---|
| Telemetry, control loop | every physics frame (`frame_end`) |
| Input output | every render frame (input callback) |
| Localization and path building | ~10 Hz on the planner thread |
| Map parsing | once, then cached on disk |
| Status file | 4 Hz |

## Lifecycle

`PLUGIN_LOAD` (first `*_init`) creates the Runtime: data directories,
config, logger, Autopilot and MapService (background load). `started`/`paused`
events gate driving. Relocations, ferries, trains and job transitions disengage
and invalidate the route. `PLUGIN_SHUTDOWN` (last `*_shutdown`) disengages,
stops all threads and closes files. Every callback body catches exceptions; an
error disengages with neutral output and is logged.

## Presentation model

`PilotStatus` (mode, availability, speeds, cross-track error, road,
next manoeuvre, message) is the only thing status consumers see. The plugin
serialises it to `status.json`, which an optional external overlay can read.
ATSPilot itself never depends on a desktop application.
