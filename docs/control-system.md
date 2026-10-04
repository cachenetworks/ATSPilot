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
geometry at speed. `a_lat` starts at 1.6 m/s² and is multiplied by
`aggressiveness` and derated:

- ×0.85 per trailer
- down to ×0.75 for cargo above 15 t
- ×0.85 when the wipers run

The target is then capped by the set speed and, if enabled, the navigation
speed limit plus offset.

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
- A set-speed change that ATSPilot did not cause is the player's: it becomes
  the maximum speed.
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

Released stops are remembered by lane id so they do not trigger again.

## Blinkers and arrival

Within 150 m of a "Keep", "Turn" or "Change Lane" manoeuvre, ATSPilot sets the
matching blinker. It reads `truck.lblinker` and `truck.rblinker` and toggles to
match. It only cancels blinkers it switched on.

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
