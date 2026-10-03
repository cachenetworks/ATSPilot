# ATSPilot

ATSPilot is an autopilot plugin for **American Truck Simulator**. It runs inside
the game as a native SCS SDK plugin. It reads live telemetry, finds the truck on
a lane-level road graph parsed from the game's own map files, and steers,
accelerates and brakes through an SDK input device. It uses no memory hacking
and no external app.

> **Status: pre-alpha (v0.1.0).** The complete chain is implemented and tested
> outside the game: telemetry → localization → path → controllers → SDK input
> device. That includes the real plugin DLL driving a simulated truck along real
> ATS road geometry. It has **not yet been validated with ATS's own vehicle
> physics**, so the [Alpha checklist](#alpha-checklist) is not yet met. Test on
> quiet roads, and keep your hands near the controls.

## Current capabilities

| Feature | State |
|---|---|
| Plugin loads with ATS (telemetry API 1.01, input API 1.00, SDK 1.15) | implemented, exercised by the plugin host |
| Live telemetry → vehicle state (position, heading, speed, inputs, gear, nav limit, truck/trailer/job config) | implemented |
| Steering / throttle / brake output through an SDK **semantical input device** | implemented |
| Configurable hotkeys: autopilot, lane assist, cruise, set speed ±, resume, cancel, emergency disable | implemented |
| Cruise control: PID with brake hysteresis, rate-limited pedals, follows the navigation speed limit | implemented, simulator-tested |
| Map: HashFS v1/v2 reader, sector (`.base` v907) and prefab (`.ppd` v25) parsers, lane graph for the whole USA map, cached | implemented; 925/925 sectors of ATS 1.61 parse with 0 errors, 97.2% of lane ends connect, 2,106 depots resolved |
| Localization with lane hysteresis | implemented |
| Rolling path along the lane graph | implemented |
| Navigation: A* route over the lane graph to the job's destination depot (lane changes, merges, recalculation when off-route, "Destination Reached" stop at the entrance) | implemented; plugin-host delivery of 7.3 km passes |
| Pure Pursuit (default) and Stanley steering, speed-adaptive lookahead, rate limiting | implemented, simulator-tested |
| Curve speed planning (lateral-acceleration limit, trailer/cargo/rain derating) | implemented, simulator-tested |
| Safety: driver override, watchdog, command expiry, controlled emergency stop, pause/teleport/ferry handling, steering-sign self-check | implemented, unit-tested |
| Logging (rotating), status file, audio cues, CSV telemetry recorder | implemented |
| Traffic awareness, traffic lights/stop signs, overtaking, parking | **not implemented** (see [roadmap](#roadmap)) |

## Current limitations

- **No traffic awareness.** ATSPilot does not see other vehicles, traffic
  lights or stop signs. The SDK does not expose them. Stay alert and brake;
  braking always disengages ATSPilot.
- **Routes are ATSPilot's own, not the in-game GPS route.** The SDK does not
  expose the GPS route polyline. ATSPilot plans its own route to the job's
  destination depot from the map data. It may pick a different, equally valid
  road than the in-game GPS. With no job (or with special transport, which has
  no destination company id) it follows the road and takes the straightest
  continuation at forks.
- **Lane changes are planned, not traffic-checked.** A lane change needed by the
  route is executed as a smooth blend without checking for other vehicles.
- **Not yet tuned against ATS physics.** Controller gains, the steering-angle
  estimate and the input-mixing assumptions come from the SDK documentation, the
  game's `controls.sii` and simulation. In-game tuning is the next step.
- **No in-game HUD.** The SDK has no supported UI extension. ATSPilot reports
  status through audio cues, warnings in the game console/log, and a
  `status.json` file that an optional external overlay can read.
- Map mods packed as `.zip` are not read, and only the base game plus installed
  DLC archives are loaded by default.

## Supported ATS version

Built against **SCS SDK 1.15** headers (telemetry game version 1.07, ATS 1.61+).
The map parser targets sector format **907** and prefab format **25**, as
shipped with ATS 1.61. Other versions are rejected cleanly with a log message
rather than misread.

## Installation

1. Download a release, or build from source (see [Development](#development)).
2. Run `install.ps1` from the release folder. It finds ATS through Steam and
   copies `plugins\atspilot.dll` to
   `...\American Truck Simulator\bin\win_x64\plugins\`.
   To install manually, copy the DLL there yourself.
3. Start ATS and accept the "advanced SDK features" prompt.
4. On the first start ATSPilot parses the map in the background, which takes
   about 15–30 s. It then caches the result in
   `Documents\American Truck Simulator\atspilot\cache\`. The cache rebuilds
   automatically when a game update or DLC changes the archives.

No ATS mod and no controls changes are needed. The input device feeds the
game's existing `steering`, `aforward` and `abackward` mixes.

Uninstall with `install.ps1 -Uninstall`, or delete the DLL.

## Usage

1. Accept a job (optional: with a job, ATSPilot routes to the destination depot
   and stops at its entrance, "Destination Reached"; parking stays manual).
2. Drive onto a road, settle in a lane, and press **F9** to engage the autopilot. ATSPilot refuses with a reason (in the
   log and console) if telemetry, the map or the lane match is not valid.
3. Adjust the set speed with **=** / **-**. Engaging while moving holds your
   current speed.
4. Take over at any time: braking, steering or pressing the throttle disengages
   immediately.

### Controls

| Action | Default key |
|---|---|
| Toggle autopilot (steer + speed) | `F9` |
| Toggle lane assist (steer only) | `F8` |
| Toggle cruise (speed only) | `Insert` |
| Increase / decrease set speed | `=` / `-` |
| Resume previous mode | `Shift+F9` |
| Cancel | `Delete` |
| Emergency disable (also blocks resume) | `Shift+Delete` |

These defaults were checked against a stock ATS 1.61 `controls.sii`. **F10 and
F11 are not used** because ATS binds them to screenshot and radio. Change any
binding in the `[controls]` section of the config. Keys are only read while ATS
has focus.

## Configuration

`Documents\American Truck Simulator\atspilot\atspilot.toml` is created with
documented defaults on first start. Main sections:

- `[speed]`: units, maximum set speed, speed-limit following
- `[steering]`: controller (`pure_pursuit` or `stanley`), lookahead, rate limits, output sign
- `[cruise]`: PID gains, brake thresholds
- `[planner]`: curve lateral-acceleration limit, deceleration, aggressiveness
- `[safety]`: override thresholds, timeouts, deviation limits
- `[route]`: navigation on/off, lane-change cost, off-route recalculation delay
- `[controls]`, `[map]`, `[audio]`, `[debug]`

Invalid values are clamped or replaced by defaults and listed in the log. A bad
config never stops ATS from loading. Run `sdk reinit` in the ATS console to
reload the config without restarting.

## Safety and disengagement behaviour

| Situation | Response |
|---|---|
| Driver brakes, steers or presses the throttle | Immediate disengage, neutral output |
| Telemetry stale, game paused, input device inactive, plugin error | Immediate disengage, neutral output |
| Truck relocated (teleport, service, ferry, train, job delivered/cancelled) | Disengage and route invalidation |
| Path stale or lost, cross-track > 3 m, heading error > 35° | Controlled emergency stop: no throttle, brakes ramp to 85%, keeps tracking the lane if geometry is still trusted, otherwise unwinds the steering slowly |
| Steering response opposes the command | Disengage with "Steering direction mismatch" |
| Control loop stalls | Commands expire after 0.25 s and the device outputs zero |

ATSPilot only ever adds to your own input. With the plugin disengaged, its
device outputs exactly zero.

## Troubleshooting

- **"ATSPilot unavailable: No valid road path"**: the map is still loading, or
  the truck is off-road or in an area without lane data. Check `logs\atspilot.log`.
- **Truck steers the wrong way**: set `steering.output_sign = 1` and report it
  in an issue. The default is derived from `controls.sii`.
- **Map errors after a game update**: the log names the format version. The
  parser refuses unknown versions rather than guessing.
- Logs: `Documents\American Truck Simulator\atspilot\logs\atspilot.log`.
  Rotated at 5 MB, 3 files kept.

## Development

Requirements: Windows 10/11 x64, Visual Studio 2022 or 2026 (or Build Tools)
with "Desktop development with C++". CMake comes with Visual Studio.
Dependencies (zlib, doctest) are fetched by CMake.

```powershell
./build.ps1            # configure, build Release, run unit tests + simulator, assemble build/release/ATSPilot
./build.ps1 -Zip       # also produce build/ATSPilot-<version>.zip
./build.ps1 -Configuration Debug -SkipTests
```

Development tools (built into `build/cmake/src/tools/<Config>/`):

- `atspilot_sim`: closed-loop controller scenarios (straight, curves, S-curve, cloverleaf ramp, loaded trailer)
- `atspilot_mapdump`: `ls`/`cat` archive contents, `build` the map with statistics, `locate` a world
  position, `companies`, `route`/`routefrom` (A* between depots), `reach` (connectivity), `item` (raw map items)
- `atspilot_plugin_host`: loads the real `atspilot.dll`, plays the game's side of the SDK, and drives a simulated truck on real map geometry

See [docs/testing.md](docs/testing.md).

Documentation: [architecture](docs/architecture.md) · [control system](docs/control-system.md) ·
[map parsing](docs/map-parsing.md) · [SDK findings](docs/sdk.md) · [testing](docs/testing.md)

## Alpha checklist

| Requirement | Status |
|---|---|
| ATS loads normally with ATSPilot installed | needs in-game confirmation |
| Live telemetry | implemented; plugin host verified |
| Autopilot toggle | implemented |
| Disabled mode sends no commands | implemented and tested (zero output) |
| Player override | implemented and unit-tested; in-game thresholds to confirm |
| Analogue steering, throttle, braking | implemented through the semantical device; in-game confirmation pending |
| Cruise holds speed smoothly | simulator-tested |
| Position matched to road geometry | implemented on the real map |
| Steering follows highway geometry, curves without oscillation | simulator- and plugin-host-tested on real ATS geometry |
| Loss of valid state disengages | implemented and tested |
| Logs explain behaviour | implemented |
| Unit/controller tests pass | 82 test cases pass |
| Installation documented, build reproducible | done |

## Roadmap

1. **In-game validation and tuning** (next): confirm the semantical input
   mixing and steering sign, measure the steering ratio, and tune gains with the
   telemetry recorder.
2. Highway pilot hardening: trailer-aware steering gains, lane changes timed
   with more look-ahead, correlating the route with the game's navigation distance.
3. Traffic awareness: an optional, asynchronous computer-vision module for lead
   vehicles (time-gap adaptive cruise), traffic lights and lane markings.
4. Intersections (traffic lights, stop signs), then parking assist.

## Privacy and security

ATSPilot works fully offline. It sends no telemetry anywhere, opens no network
ports, and runs no downloaded code.

## License

MIT; see [LICENSE](LICENSE). Third-party components and references are listed
in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md). ATSPilot is not affiliated
with or endorsed by SCS Software.
