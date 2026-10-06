# Testing

## Unit and controller tests (`tests/`, doctest)

Run them with `./build.ps1`, or `ctest -C Release` in the build directory.
There are 112 test cases. They cover:

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
- **Game cruise control:**
  - switching on only at a usable speed
  - spaced +/- presses
  - its own cancellations told apart from the game's
  - adopting the player's set speed
  - learning the minimum speed
- **Autopilot:**
  - one-key on/off
  - cruise hand-over
  - player maximum
  - curve slow-down through the cruise set speed
  - hand back when the game cancels its cruise control
  - stop at a light and go on a throttle tap
  - holding the throttle while waiting is a takeover
  - give-way slow-down
  - blinkers on and off
  - arrival with quick-park
  - heavy-haul auto profile
- **Traffic and lights:**
  - a stopped vehicle ahead becomes a stop short of it
  - a moving one is followed at the time gap
  - traffic in the next lane, oncoming or behind is ignored
  - crossing traffic only counts when the truck would meet it
  - signal stops are matched to live lights by semaphore id, otherwise by
    position and learned facing (ambiguous candidates are not trusted)
  - amber: stop with room, otherwise go
  - closed loop: queue behind a stopped car and move off with it, stop on red
    and go on green without a tap, a light that cannot be read is an all-way
    stop,
    and stop signs are left once crossing traffic has passed
- **Routing:**
  - lane neighbours
  - A* with a required lane change, where every step must be a real edge
  - unreachable destinations
  - staying in lane when possible
  - route-following path with a smooth lane-change blend and manoeuvre announcement
  - arrival stop
  - GPS-distance route matching picks the branch whose length matches
  - the in-game GPS route steers A* onto its branch
  - city turn (R = 20 m) with and without curvature-limited lookahead
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
atspilot_mapdump build "<ATS or ETS2 dir>" map.cache     # statistics, gap histograms, per-look diagnostics
atspilot_mapdump locate map.cache <x> <z>        # lanes near a world position, e.g. from a telemetry CSV
atspilot_mapdump ls "<ATS dir>\base_map.scs" map/usa
atspilot_mapdump ls "<ETS2 dir>\base_map.scs" map/europe
```

## Plugin host (end-to-end without the game)

```powershell
atspilot_plugin_host <atspilot.dll> "<game dir>" map.cache [scenarios] [seconds] [speedup] [nav_seconds] [ats|ets2]
```

The host loads the real DLL and calls its exports with the same SCS SDK contracts used by ATS and ETS2:

- version negotiation
- event and channel registration
- the `truck` configuration event
- frame events, and the input device's per-frame callback loop

Its simulated truck drives on lanes picked at random from the real map. The
host engages the autopilot through the development export and measures lane
deviation with an independent localizer.

Latest extended run (ATS 1.61 map, 8 scenarios × 90 s at 4× speed): **8/8 stayed
engaged, about 15 km driven, RMS lane deviation 0.02–0.06 m.**

Dual-game smoke validation on 2026-10-05 also built fresh ATS 1.61.3.1 and
ETS2 1.61.1.1 map caches with zero sector parse errors, then ran the real DLL
through one simulated road scenario in each game identity. Both stayed engaged.

With `nav_seconds`, the host also sends a job configuration event. The
destination is a real depot 2.5–8 km away. The host then checks that the
plugin plans a route, follows it with navigation active and stops at the
entrance. The host emulates the game's cruise control (toggle, ±, cancel on
brake), the blinker switches, quick-park and the GPS navigation distance. It
taps the throttle 1 s after ATSPilot starts waiting at a stop line.

Latest run, Winner `gld_frm_grg`:
- 7.3 km planned, 7,342 m driven
- game cruise control on for 271 s
- blinkers used for 146 s
- 15 stop lines released with a throttle tap
- quick-park pressed once
- ended in "Destination Reached"

The HUD can be checked with `ATSPILOT_HUD_STANDALONE=1`, which shows it on the
primary monitor without a game window.

In tight city intersections (R ≈ 20 m), Pure Pursuit with the highway-tuned
lookahead cuts corners by 1–2 m (see control-system.md).

## What still needs the real game

The simulator and host cannot validate:

- Real game vehicle dynamics: tyre slip, trailer articulation, real steering ratio and
  lag, engine and gearbox response
- the semantical mixing assumptions in sdk.md
- whether `truck.input.*` includes the device contribution
- frame timing under load

In-game checklist:

1. ATS or ETS2 loads, the log shows "Input device registered" and "Map loaded".
2. Cruise mode on a straight road: speed holds within ±2 mph and brakes on downhill.
3. Lane assist on a highway: no "Steering direction mismatch". The learned
   `max_wheel_angle` appears in recordings.
4. Brake, steer and throttle each disengage. Pausing disengages.
5. Autopilot through highway curves and interchanges with a trailer.
