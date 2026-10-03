# SCS SDK integration and findings

All statements below were checked against the **SCS SDK 1.15** headers
(`external/scs_sdk_1_15/include`) and, where the headers say nothing, against
the game data and profile files of ATS 1.61. When this document and the headers
disagree, the headers win.

## Plugin loading

- The game loads every DLL in `<ATS>\bin\win_x64\plugins\`
  (SDK `readme.txt`). It may run several init/shutdown cycles per process
  (`sdk reinit`, `sdk reload`). ATSPilot keeps one `Runtime` with a reference
  count shared by the telemetry and input APIs, and supports repeated cycles.
- Exports: `scs_telemetry_init`, `scs_telemetry_shutdown`, `scs_input_init`,
  `scs_input_shutdown` (plus two `atspilot_dev_*` functions used only by the
  test harness, which ATS ignores).
- All SDK callbacks arrive on the game's main thread, and calls back into the
  SDK are only allowed from there (`readme.txt`, "Calling conventions and
  threading"). Game-console logging is therefore queued and flushed from
  `frame_end`.

## Telemetry used

| Channel / event | Use |
|---|---|
| `truck.world.placement` (dplacement) | position and heading |
| `truck.speed` | speedometer speed, m/s |
| `truck.local.velocity.linear/angular` | recorder and simulator parity |
| `truck.input.steering/throttle/brake` | driver-override detection |
| `truck.effective.steering` + `truck.wheel.steering[i]` | online steering-ratio estimate and steering-sign self-check |
| `truck.engine.gear`, `truck.brake.parking`, `truck.engine.enabled` | engagement preconditions |
| `truck.navigation.speed.limit`, `.distance`, `.time` | speed-limit following, status |
| `truck.wipers` | rain proxy for curve-speed derating |
| events `frame_start/frame_end/paused/started` | timing, pause handling |
| `configuration` `truck` | wheel positions, steerable and powered wheels, giving the wheelbase |
| `configuration` `trailer.N`, `job` | trailer count, cargo mass |
| `gameplay` `player.use.ferry/train`, `job.delivered/cancelled` | route invalidation |

### Finding: the GPS route is not exposed
**Finding:** telemetry exposes only the navigation distance, time and speed
limit. It does not expose the route polyline or the destination position.

**Impact:** ATSPilot cannot read the waypoints of the in-game GPS.

**Solution (planned):** route over the parsed lane graph to the job's
destination company, and correlate it with the navigation distance. Until then,
junctions use "follow the road".

### Finding: no traffic information
The SDK exposes nothing about other vehicles, traffic lights or signs. Traffic
awareness needs an optional perception module (see the roadmap).

## Input: the semantical device

`scssdk_input_device.h` offers `SCS_INPUT_DEVICE_TYPE_semantical`, whose inputs
"map directly to mixes with the same name … if mix expression in a fresh
controls.sii references something like `semantical.<mixname>?0`, then semantical
input is likely supported for that mix."

The relevant mixes from an ATS 1.61 profile's `controls.sii`:

```
mix steering  `dsteering - memory(j_steer_c?1, …) - semantical.steering?0`
mix aforward  `memory(j_throttle_c?1, …) + semantical.aforward?0`
mix abackward `memory(j_brake_c?1, …) + semantical.abackward?0`
```

Consequences, all handled in code:

1. The device **adds to** the driver's input. Outputting 0 is neutral, so the
   disabled state cannot fight the driver.
2. `semantical.steering` is **subtracted**, like a joystick axis.
   `truck.input.steering` is counterclockwise-positive (left = +, header note).
   So ATSPilot sends `-steering` (`steering.output_sign = -1`). Because this is
   inferred, the autopilot also checks it at runtime: if the effective steering
   keeps opposing the command, it disengages ("Steering direction mismatch").
3. Whether `truck.input.*` already includes the semantical contribution is not
   documented. The override detector learns this from data (`InputMixing`) and
   meanwhile uses the conservative interpretation.

No binding UI and no mod are needed. The trade-off, as the header notes, is
that a future change to these mixes would need a plugin update.

### Finding: default key conflicts
In a stock 1.61 profile, `F10` is bound to screenshot, `F11` to radio and the
bug reporter, and `Ctrl+F9` to teleport. ATSPilot's defaults avoid these:
`F9`, `F8`, `Insert`, `=`, `-`, `Delete`, `Shift+F9` and `Shift+Delete` are
unbound (or modifier-distinct) in that profile. Hotkeys are read with
`GetAsyncKeyState` only while the ATS window has focus, with exact modifier
matching.

## Coordinate systems

From `scssdk_value.h` and the SDK `telemetry_position` example:

- World: +X east, +Y up, +Z south, metres.
- Heading: unit range [0, 1), counterclockwise from above; 0 = north (−Z),
  0.25 = west, 0.5 = south, 0.75 = east.
- Vehicle space: −Z forward, +X right, +Y up. The SDK example rotates a local
  vector by the heading as `x' = x·cos h + z·sin h`, `z' = −x·sin h + z·cos h`.
- Wheel steering: rotations in [−0.25, 0.25], counterclockwise (left) positive.

ATSPilot plans in a right-handed top-down **plan** frame: `plan.x = world.x`,
`plan.y = −world.z`, `yaw = 2π·heading + π/2`. The only conversions live in
`src/core/math/Coordinates.h` and are unit-tested, including against the SDK
example's rotation.

## Version handling

`scs_telemetry_init` accepts API 1.00/1.01 and logs the game's telemetry
version, warning on an unexpected major version. Map formats are
version-checked separately (see map-parsing.md).
