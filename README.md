# ATSPilot

ATSPilot is an autopilot plugin for **American Truck Simulator**. It runs inside
the game as a native SCS SDK plugin. It reads live telemetry, finds the truck on
a lane-level road graph parsed from the game's own map files, steers along it,
and controls speed through the **game's own cruise control**. That means the
game's adaptive cruise control and emergency brake assist handle traffic.
Everything is controlled with **one key**. It uses no memory hacking and needs
no external app.

> **Status: pre-alpha (v0.2.0).** The full chain has been tested outside the
> game, including the real plugin DLL completing a 7.3 km delivery on real ATS
> map data (game cruise control, 15 stop lines, blinkers, quick-park, arrival).
> It has **not yet been validated against ATS's own vehicle physics and input
> handling**. Test on quiet roads and keep your hands near the controls.

## How it drives

| Area | What ATSPilot does |
|---|---|
| **One key** | `F9` switches ATSPilot on and off. Braking, steering or the throttle take over immediately. |
| **Speed** | Switches on the game's cruise control and nudges its set speed with the game's +/- controls. It slows for curves, speed limits and stops. Your own cruise +/- presses set the maximum speed. It uses its own pedals only below cruise-control speed, for hard braking and at stop lines. |
| **Traffic** | Handled by the game's **adaptive cruise control** and **emergency brake assist** on trucks equipped with them. If the game switches its cruise control off on its own, ATSPilot hands back control ("Cruise control cancelled - take over"). |
| **Steering** | Pure Pursuit along the lane centre from the parsed map. The lookahead shrinks in tight curves, so city turns are tracked to about 0.4 m instead of cutting the corner. The game's own lane assist is left off so the two don't fight. |
| **Route** | Plans a route to the job's destination depot over the lane graph, including lane changes. It keeps that route consistent with the **in-game GPS** by matching the GPS's remaining navigation distance, and re-plans along the matching branch when they disagree. With no job it follows the road. |
| **Junctions** | From map data it knows which junction lanes have **traffic lights, stop signs, give-way rules and railway crossings**. It stops at lights and stop signs and waits; **tap the throttle** to go. It slows to 4 m/s through give-way lanes and rail crossings, and never routes through truck-prohibited lanes. |
| **Blinkers** | Indicates 150 m before lane changes, exits and turns, and cancels only blinkers it switched on itself. |
| **Arrival** | Stops at the depot entrance, presses the game's **quick-park**, and switches off ("Destination Reached"). |
| **Profiles** | `auto` (heavy haul above 25 t, otherwise normal), `comfort`, `normal`, `assertive`, `heavy_haul`. Each scales curve speed, braking and steering rate. Heavy haul caps speed at 55 mph. |
| **HUD** | An in-game status panel showing mode, set speed, cruise control, navigation, next manoeuvre, remaining distance, profile and messages. |

## Limitations

- **Traffic awareness comes from the game's systems.** The SDK exposes no
  vehicles, lights or signs. Following traffic depends on the truck having
  adaptive cruise control. Below cruise-control speed, such as pulling away
  from a stop, ATSPilot drives its own pedals without seeing traffic. Watch the
  road.
- **Traffic-light state is unknown.** ATSPilot knows *where* lights are, not
  their colour. It therefore stops at every signal and waits for your throttle
  tap.
- **GPS matching is indirect.** The SDK gives only the GPS's remaining
  distance. ATSPilot picks the route whose length matches it, considering
  alternatives at the next six forks. Very unusual GPS routes may still differ.
- **Lane changes are not traffic-checked.** A route lane change is a smooth
  blend with the blinker on, but nothing checks the target lane is clear.
- **Parking** uses the game's quick-park feature, which depends on the
  game's parking settings. There is no reverse-parking automation.
- **The HUD needs borderless or windowed mode.** It is a click-through overlay
  window, because the SDK has no UI extension, so exclusive fullscreen may hide
  it. `status.json` carries the same data.
- `.zip` map mods are not read. Base game and installed DLC are.

## Supported ATS version

Built against **SCS SDK 1.15** (telemetry 1.07, input 1.00, ATS 1.61+). The map
parser targets sector format **907** and prefab format **25**. Other versions
are rejected cleanly with a log message rather than misread.

## Installation

1. Download a release, or build from source (see [Development](#development)).
2. Run `install.ps1` from the release folder. It finds ATS through Steam and
   copies `plugins\atspilot.dll` to `...\American Truck Simulator\bin\win_x64\plugins\`.
3. Start ATS and accept the "advanced SDK features" prompt.
4. On the first start the map is parsed in the background (about 20 s) and
   cached in `Documents\American Truck Simulator\atspilot\cache\`.

No mod and no controls changes are needed. The plugin's input device feeds the
game's existing steering, pedal, cruise-control, blinker and quick-park
controls.

## Usage

1. Optionally take a job. ATSPilot will route to its depot and stop at the
   entrance.
2. Drive onto a road, settle in a lane, and press **F9**.
3. Set your maximum speed with the **game's cruise control +/- keys**.
4. At a traffic light or stop sign ATSPilot stops and waits. **Tap the throttle**
   to go; holding it means you are taking over.
5. Press **F9** again, brake, or steer to take over.

The key is configurable in `[controls] toggle`. F10 and F11 are avoided because
ATS uses them.

## Configuration

`Documents\American Truck Simulator\atspilot\atspilot.toml` is created with
documented defaults on first start:

- `[controls]`: the single key
- `[profile]`
- `[speed]`: units, default maximum, speed-limit following
- `[ingame]`: use game cruise control, blinkers, quick-park
- `[intersections]`: stop at signals or stop signs, give-way speed, tap duration
- `[steering]` / `[cruise]` / `[planner]`: tuning
- `[safety]`
- `[route]`: navigation, GPS matching, lane-change cost
- `[hud]`, `[map]`, `[audio]`, `[debug]`

Invalid values are clamped or replaced by defaults and listed in the log. Run
`sdk reinit` in the ATS console to reload.

## Safety and disengagement

| Situation | Response |
|---|---|
| Brake, steer, hold the throttle, or press F9 | Immediate disengage, neutral output |
| Game cancels its cruise control by itself | Disengage, "take over" |
| Telemetry stale, game paused, plugin error | Immediate disengage |
| Teleport, service, ferry, train, job delivered or cancelled | Disengage and route reset |
| Path stale or lost, deviation > 3 m, heading error > 35° | Controlled emergency stop |
| Steering response opposes the command | Disengage ("Steering direction mismatch") |
| Control loop stalls | Commands expire after 0.25 s |

ATSPilot only adds to your input. When it is off its device outputs zero and
presses nothing.

## Troubleshooting

- **"ATSPilot unavailable: …"**: the reason is in the message and in
  `logs\atspilot.log`.
- **Cruise control never switches on**: the truck may need to be above the
  game's cruise-control minimum. ATSPilot learns that minimum.
- **Truck steers the wrong way**: set `steering.output_sign = 1` and report it
  in an issue.
- **HUD not visible**: use the game's borderless or windowed display mode, or
  set `hud.enabled = false`.

## Development

Requirements: Windows x64, Visual Studio 2022/2026 (or Build Tools) with C++.
CMake comes with Visual Studio; zlib and doctest are fetched automatically.

```powershell
./build.ps1          # build, 97 unit/controller tests, simulator, package build/release/ATSPilot
./build.ps1 -Zip
```

Tools:
- `atspilot_sim`: closed-loop controller scenarios
- `atspilot_mapdump`: archive browsing, map build statistics, routes, connectivity
- `atspilot_plugin_host`: drives the real DLL through the SDK with a simulated
  truck on real map data, emulating the game's cruise control, blinkers and
  GPS distance

Docs: [architecture](docs/architecture.md) · [control system](docs/control-system.md) ·
[map parsing](docs/map-parsing.md) · [SDK findings](docs/sdk.md) · [testing](docs/testing.md)

## Roadmap

1. In-game validation: semantical-input mixing, steering sign, cruise-control
   step size and minimum, quick-park behaviour.
2. Traffic-light colour from an optional, asynchronous screen-capture module
   (low-latency, newest-frame-only).
3. Traffic-checked lane changes, and better trailer-aware steering.

## Privacy and security

Fully offline. No telemetry leaves your computer, no network ports are opened,
and no downloaded code runs.

## License

MIT; see [LICENSE](LICENSE) and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
Not affiliated with or endorsed by SCS Software.
