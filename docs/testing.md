# Testing

## Unit and controller tests (`tests/`, doctest)

Run them with `./build.ps1`, or `ctest -C Release` in the build directory.
There are 75 test cases. They cover:

- **Maths:** angle normalization and differences, curvature sign and radius,
  quaternion rotation, Hermite splines, SDK heading and world/plan conversions
  (checked against the SDK example's rotation).
- **Path:** arc length, signed cross-track error, windowed projection on a
  self-approaching loop, interpolation, curvature estimate, resampling, trimming.
- **Controllers:**
  - PID (P, I, derivative on measurement, anti-windup, preload)
  - Pure Pursuit, including the steady-state angle on an arc; Stanley signs
  - lookahead limits, steering-shaper rate and authority limits
  - curve speed, planner slow-down before (not after) curves, path end as a stop
  - pedal controller with brake hysteresis and the emergency ramp
- **Config:** TOML parsing and error lines, defaults round-trip, invalid values
  fall back with warnings, garbage input never throws.
- **Safety:**
  - override detection (multi-frame steering, immediate brake, own braking
    ignored, learned input mixing)
  - watchdog
- **State machine:**
  - engage/refuse with reasons
  - set speed stepping
  - lane assist without pedals
  - stale telemetry → neutral
  - stale path → controlled stop to standstill
  - driver brake override
  - pause and resume
  - emergency disable blocks resume
  - inverted steering detection
- **Map:**
  - CityHash, tokens, SII parser, bounds-checked reader
  - HashFS v1 archive round-trip with zlib and directory listings
  - synthetic sector decoding and rejection of unknown items or versions
  - lane-graph query and cache round-trip with fingerprint check
  - localizer heading selection and hysteresis
  - path-builder branch choice and plan stability
- **Scenarios** (closed loop through `Autopilot`): straight, recovery from an
  offset, R = 800, R = 60, S curve, cloverleaf ramp, loaded trailer with a slow
  steering rack, Stanley variant, cruise hold.

## Controller simulator

```powershell
build\cmake\src\tools\Release\atspilot_sim.exe                 # metrics table
build\cmake\src\tools\Release\atspilot_sim.exe --config x.toml --csv out
```

## Map validation

```powershell
atspilot_mapdump build "<ATS dir>" map.cache     # statistics, gap histograms, per-look diagnostics
atspilot_mapdump locate map.cache <x> <z>        # lanes near a world position, e.g. from a telemetry CSV
atspilot_mapdump ls "<ATS dir>\base_map.scs" map/usa
```

## Plugin host (end-to-end without the game)

```powershell
atspilot_plugin_host <atspilot.dll> "<ATS dir>" map.cache [scenarios] [seconds] [speedup]
```

The host loads the real DLL and calls its exports exactly as ATS would:

- version negotiation
- event and channel registration
- the `truck` configuration event
- frame events, and the input device's per-frame callback loop

Its simulated truck drives on lanes picked at random from the real map. The
host engages the autopilot through the development export and measures lane
deviation with an independent localizer.

Latest run (ATS 1.61 map, 8 scenarios × 90 s at 4× speed): **8/8 stayed
engaged, about 15 km driven, RMS lane deviation 0.02–0.06 m on roads.** One
scenario ran into a city intersection (R ≈ 20 m), where Pure Pursuit cut the
corner by 1.2–2.4 m. That is expected with highway-tuned lookahead and is
documented in control-system.md.

## What still needs the real game

The simulator and host cannot validate:

- ATS vehicle dynamics: tyre slip, trailer articulation, real steering ratio and
  lag, engine and gearbox response
- the semantical mixing assumptions in sdk.md
- whether `truck.input.*` includes the device contribution
- frame timing under load

In-game checklist:

1. ATS loads, the log shows "Input device registered" and "Map loaded".
2. Cruise mode on a straight road: speed holds within ±2 mph and brakes on downhill.
3. Lane assist on a highway: no "Steering direction mismatch". The learned
   `max_wheel_angle` appears in recordings.
4. Brake, steer and throttle each disengage. Pausing disengages.
5. Autopilot through highway curves and interchanges with a trailer.
