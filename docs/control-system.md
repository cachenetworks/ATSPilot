# Control system

## Modes

`OFF`, `AUTOPILOT` and `EMERGENCY STOP`, switched with a single key. ATSPilot
has no separate cruise or lane-assist modes: the game's own cruise control and
lane assist are used directly (see "Speed via the game's cruise control"). Engagement validates the following, and on failure reports
the reason instead of engaging:

- telemetry valid and not paused
- not in reverse
- parking brake released and engine running (pedal modes)
- for steering modes: a fresh path, cross-track error within
  `safety.max_cross_track_m`, and heading error within `safety.max_heading_error_deg`

## Lateral control

Positions are taken at the **rear axle** (Pure Pursuit) or the **steer axle**
(Stanley). Both are computed from the truck's wheel positions in the `truck`
configuration event.

- **Lookahead:** `Ld = clamp(base + v·factor, min, max)`, with defaults
  15 m + 0.5 s·v within [4, 60] m. It is then limited to `0.35 × R_min`,
  where `R_min` is the tightest radius within the lookahead. On a 20 m
  city turn this cuts the corner error from 1.53 m to 0.38 m in the simulator.
  Highway curves (R ≥ 60 m) are unaffected.
- **Pure Pursuit:** `δ = atan(2·L·sin α / Ld)`, where α is the angle to the path
  point `Ld` ahead of the projection.
- **Stanley:** `δ = ψ_e + atan2(−k·e, v + v_s)`.

`SteeringShaper` converts the road-wheel angle δ into the normalized command
`δ / δ_max`:

- `δ_max` is learned from telemetry (|wheel steering| / |effective steering|),
  starting at 35°.
- Authority bound: `|δ| ≤ atan(L · a_lat · margin / v²)`, so a bad target cannot
  command a violent turn at speed.
- First-order smoothing (80 ms).
- Rate limit of 1.5 units/s at low speed, scaled down to 35% at 25 m/s and by
  0.75 per trailer.

## Speed planning

For every 5 m of path within the 400 m horizon:

```
v_curve(s) = sqrt(a_lat / |κ(s)|)              κ from a three-point circumcircle over ±12 m
v_allowed  = min over s of sqrt(v_curve(s)² + 2·a_comfort·(s − s₀))
```

The end of known path counts as a stop line, so the truck never runs past known
geometry at speed. `a_lat` starts at 1.9 m/s² and is multiplied by
`aggressiveness` and derated:

- ×0.85 per trailer
- down to ×0.75 for cargo above 15 t
- ×0.85 when the wipers run

The target is the posted navigation speed limit plus offset, capped by
`speed.max`. Where no limit is reported the last one still applies. Before any
limit has been seen, `speed.unknown_limit` (45 mph) is used.

## Longitudinal control

A PID on the speed error produces a signed demand in [−1, 1]. It uses
derivative on measurement, conditional-integration anti-windup and a bumpless
preload.

- Braking starts below −0.10 and ends above −0.03. The band in between is
  coasting, which removes pedal flicker.
- Brake level classes: minor (≤ 0.05), normal (≤ 0.45), strong (≤ 0.75).
- Pedals are rate-limited when applied (throttle 1.2/s, brake 0.8/s) and
  released faster.
- Emergency stop: throttle 0, brake ramps at 0.6/s to 0.85. It never steps
  straight to full brake.

## Safety layer

| Check | Threshold (default) | Action |
|---|---|---|
| Driver brake | > 0.10 beyond ATSPilot's own | disengage |
| Driver steering | > 0.20 beyond what was held at engagement, for 3 frames | disengage |
| Driver throttle | > 0.30 for 5 frames | disengage |
| Telemetry age | > 0.5 s | disengage |
| Path age | > 1.5 s | emergency stop |
| Cross-track error | > 3.0 m (warn at 1.2 m) | emergency stop |
| Heading error | > 35° | emergency stop |
| Steering response sign | opposite for 45 frames | disengage |
| Command age at output | > 0.25 s | output zero, disengage |

## Tuning workflow

1. Set `debug.record_telemetry = true` and drive a test stretch. The CSV in
   `atspilot\recordings` has target and actual heading, speed, steering, pedals,
   cross-track error, lookahead and curve radius.
2. Reproduce changes offline with `atspilot_sim --config my.toml --csv out`.
3. Gains worth adjusting first: `steering.lookahead_*`, `steering.max_steering_rate`,
   `cruise.kp/ki`, `planner.max_lateral_accel`.

Simulator results with the defaults (kinematic model, 150 ms steering lag):

| Scenario | max XTE | max lateral accel |
|---|---|---|
| straight 3 km | 0.00 m | 0.00 m/s² |
| R = 800 m curve at 29 m/s | 0.06 m | 1.03 |
| R = 60 m curve from 25 m/s | 0.58 m | 1.52 |
| S curve R = 150 m | 0.47 m | 1.76 |
| highway exit with R = 70 m loop | 0.51 m | 1.60 |
| loaded trailer, 350 ms steering lag | 0.23 m | 1.69 |

A shorter lookahead (6 m + 0.8·v) halves corner cutting in tight urban turns
but adds steering dither at low speed. The conservative default favours highway
smoothness.

## Speed via the game's cruise control

`GameCruiseManager` drives the game's cruise control through the input
device's `cruiectrl`, `cruiectrlinc` and `cruiectrldec` controls. It reads the
result back from `truck.cruise_control`:

- Below the game's cruise minimum (starting guess 8.5 m/s, learned upwards
  when a toggle press is refused), ATSPilot drives its own pedals.
- Above it, it presses the toggle once and then nudges the set speed towards
  the planned target. Presses are at least 0.25 s apart. While cruise control
  is on, ATSPilot sends no throttle or brake.
- A set-speed change that ATSPilot did not cause is the player's. With
  `speed.cruise_sets_max` it becomes the maximum speed; by default it is
  ignored and the set speed returns to the limit.
- When the plan needs more than 1.5 m/s of extra braking (sharp curves, stop
  lines), ATSPilot brakes itself, which cancels the cruise control as it would
  for a driver. Cruise control is switched back on afterwards.
- If the cruise control switches off without ATSPilot causing it, the
  autopilot disengages and asks the driver to take over. Causes include driver
  input, emergency brake assist, or adaptive cruise dropping out at low speed.

This puts speed in traffic under the game's adaptive cruise control and
emergency brake assist, on trucks that have them.

## Junctions

The planner publishes the controlled junction lanes on the path (`PathStop`):
traffic light (semaphore id), stop sign, give way, railway crossing.

- **Traffic lights and stop signs:** a speed constraint of 0 at the stop line,
  which is the lane start minus the truck's front overhang and a 1.5 m margin.
  Once the truck is stopped there, ATSPilot holds the brake and waits. A
  throttle tap shorter than 1.5 s releases that stop. Holding the throttle
  longer is a takeover.
- **Give way and railway crossings:** a 4 m/s constraint.
- **Right on red:** at a red light whose lane turns 60-150 degrees right, the
  truck stops fully at the line for 2 s. It then turns once the junction has
  been clear for 1 s. Clear means nothing is moving in the junction, nothing is
  coming along the road being joined within 8 s, and no oncoming vehicle near
  the junction could be turning left into it. Once committed it carries on,
  and the traffic checks still apply on the path. `intersections.right_on_red`
  switches it off.

Released stops are remembered by lane id so they do not trigger again.

## Services: weigh stations and fuel

The map builder reads prefab spawn points: type 3 is a fuel pump stand and type
6 is a weigh-station scale. Each is matched to the prefab lane passing nearest
it, and stored in the map cache (format 6). `planRouteWithServices`
(`map/ServicePlanner.cpp`) then adds stops to the route.

- **Weigh stations:** disabled by default because telemetry does not expose a
  reliable "this truck must weigh now" state. When explicitly enabled, the
  first scale beside the route ahead counts if it is
  within 150 m, faces the same way, and is at least 250 m on. The route then
  goes truck → scale lane → destination, unless that adds more than 3 km.
  Stations already passed are skipped for 15 minutes. At the scale the truck
  stops with the rig on it, then leaves after 5 s if no light ahead is red
  (45 s at most).
- **Fuel:** below `services.refuel_below` (25%), or with the dashboard
  warning, the route goes via the pump reachable along mapped lanes that is
  nearest by driving (within 25 km straight-line / 40 km driving), then on to
  the destination. At the pump
  the truck stops with its front 3.5 m past it and holds the game's
  `activate` control. If no fuel flows within 6 s it moves 4 m forward and
  tries again, up to 3 times. It finishes when the tank is 98% full or the
  level stops rising for 4 s. Leaving a pump still low counts as a failed
  attempt; after two, fuel routing pauses for 30 minutes.

The route's stops appear on the path as `PathStop`s of kind `Fuel` and `Weigh`
at the stand's position along its lane. A route that only leads to a pump
(free roam, no destination) is not navigation, so its end is no arrival.

## Blinkers and arrival

The path builder derives turn-signal intervals (`Indication`) from the geometry
of the lanes it chains together:

| Manoeuvre | Detected when | Signal on | Off |
|---|---|---|---|
| Lane change | a route lane change (blend to the parallel lane) | 40 m before the blend | blend complete |
| Turn | a branch whose junction lanes turn more than 35° | 60 m before the junction lane | 60% through it |
| Exit / fork | a branch off the straight-on lane that is at least 6 m away from it 120 m on, and still diverging | 150 m before | 40 m after |
| Merge | lanes joining, both nearly parallel (within 35°) 80 m back, ours clearly to the side | 150 m before the join | 10 m after |
| Lane drop | the path shifts at least 2.5 m sideways into the continuing lane | 150 m before | 10 m after |

The truck's front bumper position on the path selects the active interval.
ATSPilot reads `truck.lblinker` and `truck.rblinker` and toggles to match. It
only cancels blinkers it switched on. `atspilot_mapdump route <cache> <city>
<company> <city> <company> --signals` lists every signal along a route, with
the heading change after it as a check.

At a route's end (< 40 m remaining, stopped), it presses the game's
`quickpark` and disengages with "Destination Reached".

## Profiles

`applyProfile` multiplies curve lateral acceleration, comfort deceleration and
steering rate:

| Profile | Multiplier | Speed cap |
|---|---|---|
| comfort | 0.8 | none |
| normal | 1.0 | none |
| assertive | 1.2–1.25 | none |
| heavy_haul | 0.7 | 55 mph |

`auto` chooses heavy_haul above 25 t of cargo.

## Traffic and traffic lights (game memory)

`GameMemory` reads the AI traffic around the truck and the state of nearby
traffic lights every frame. `assessTraffic` turns them into speed constraints:

- **On the path:** each body (car, tractor, trailer) is projected onto the path.
  If its footprint reaches into the corridor (±1.5 m around the lane centre),
  the truck aims to be at its speed `standstill_gap + time_gap · v` behind it.
- **Crossing:** vehicles more than 30° off the path direction are extrapolated
  for up to 4 s. If one would occupy the corridor ahead about when the truck
  gets there (within 2.5 s), it is treated as stopped at that point. A moving
  cross-traffic vehicle already over a distant intersection is still timed
  instead of being mistaken for a stopped lead vehicle.
- **Other levels:** a vehicle whose height is more than 3.5 m from the road
  surface at its spot on the path is on a bridge above or a road below, and is
  ignored. Road heights come from the map, so hills do not trigger this.
- **Hard braking:** if stopping behind the nearest obstacle needs more than
  2.5 m/s², ATSPilot brakes directly in proportion instead of waiting for the
  speed loop. For a crossing that is only predicted, the prediction must hold
  for 0.4 s first, unless it is within 15 m.

A game hitch (loading, alt-tab) is not a fault. Output goes neutral until frames
resume, and the path from before the hitch stays usable for `path_timeout_s`
afterwards. Autopilot carries on rather than switching off.

Signal lanes carry their prefab's semaphore id (`LaneSegment::semaphoreId`).
The live light with that id nearest the stop line decides:

| Light | Action |
|---|---|
| green | go |
| red, red-amber | stop at the line |
| amber | stop if that needs at most `amber_max_decel` (3 m/s²), otherwise go |
| flashing, off | give way |
| not found | all-way stop: stop, then go once the junction is clear |

Matching is by semaphore id first. Without a match, a light just before or up
to 50 m beyond the stop line, within 25 m of the lane and facing along the road
is used. Which way lights face relative to their traffic is learned from
id-matched lights. Until it is known, position matching is only trusted when
all candidate lights agree. Further signal lanes within 60 m of one just passed
on green (inside the same junction) follow it.

At stop signs, and at lights whose state cannot be read, the truck stops fully
for 1.5 s and goes once nothing is on or crossing the path within 40 m. Only
without game memory does ATSPilot ask for a throttle tap.

## Direct steering

With game memory, the command is written to the truck's steering value instead
of the semantical input, so the driver's own input and ATSPilot's are not added
together. Before the first engagement, the stored value is compared with the
SDK's effective steering while the driver steers. That gives its sign and scale,
which are logged. The steering-direction check of the safety layer still
applies.
