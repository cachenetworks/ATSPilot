# ATSPilot

ATSPilot is a self-driving plugin for **American Truck Simulator**. It runs
inside the game as a native SCS SDK plugin. It finds the truck on a lane-level
road graph parsed from the game's own map files and drives the in-game GPS route
to the job's depot. It follows traffic, obeys traffic lights and gives way at
junctions. Speed is held with the **game's own cruise control**. Everything is
controlled with **one key**.

> **Status: pre-alpha (v0.3.0).** The driving logic is tested in a simulator and
> with the real plugin DLL on real ATS map data. The game-memory features
> (direct steering, traffic, light states, GPS route) are new and **not yet
> validated in game**. Test on quiet roads and keep your hands near the controls.

## How it drives

| Area | What ATSPilot does |
|---|---|
| **One key** | `F9` switches ATSPilot on and off. Braking, steering or the throttle take over immediately. |
| **Steering** | Pure Pursuit along the lane centre from the parsed map, written directly to the truck's steering. The lookahead shrinks in tight curves, so city turns are tracked to about 0.4 m. |
| **Speed** | Switches on the game's cruise control and nudges its set speed with the game's +/- controls. It slows for curves, speed limits, traffic and stops. Your own cruise +/- presses set the maximum speed. Below cruise-control speed, and to brake, it uses its own pedals. |
| **Traffic** | Sees the AI traffic around the truck. It follows the vehicle ahead at a 2 s gap and queues behind stopped vehicles. It waits for crossing traffic it would meet at a junction, and brakes hard if something cuts in. |
| **Traffic lights** | Reads each light's state. It stops on red, stops for amber when there is room, and goes on green, with no input from you. |
| **Junctions** | Stops fully at stop signs, then pulls away once crossing traffic has cleared. Slows through give-way lanes and railway crossings. It never routes through truck-prohibited lanes. |
| **Route** | Follows the **in-game GPS route** exactly, to a job's depot or to any destination you set on the GPS. It plans lane changes along the way. With no route it follows the road. |
| **Blinkers** | Indicates 150 m before lane changes, exits and turns, and cancels only blinkers it switched on itself. |
| **Arrival** | Stops at the depot entrance, presses the game's **quick-park**, and switches off ("Destination Reached"). |
| **Profiles** | `auto` (heavy haul above 25 t, otherwise normal), `comfort`, `normal`, `assertive`, `heavy_haul`. Each scales curve speed, braking and steering rate. Heavy haul caps speed at 55 mph. |
| **HUD** | An in-game panel showing mode, speeds, cruise control, navigation, next manoeuvre, traffic ahead, the next light's state and messages. |

## Game memory

The SCS SDK exposes no traffic, light states or GPS route, and its input mix
proved unreliable for steering. ATSPilot therefore reads these from the game's
memory and writes the truck's steering directly. It uses the game structure
layouts of [ETS2LA's game plugin](https://github.com/ETS2LA/plugin) (MIT). This
is single-player, local and read-mostly: the only value written is the truck's
own steering.

- Only used on the game version those layouts were written for (**1.61**).
  Addresses are found by pattern scan when the game starts.
- Every access is guarded: a failed read switches that feature off for the
  session instead of crashing the game.
- Switch it off with `[memory] enabled = false`. ATSPilot then drives with the
  SDK alone: it stops at every light and waits for a throttle tap, and leaves
  traffic to the game's adaptive cruise control.

## Limitations

- **Pre-alpha in game.** Steering calibration, light matching and traffic
  reading are verified offline only. The log reports what they find on the
  first drive.
- **Lane changes only look ahead.** Vehicles in the target lane ahead of the
  truck are on its path, so it follows them or stops for them. Faster traffic
  approaching from behind is not considered.
- **Not for multiplayer.** TruckersMP vehicles are not read.
- **Parking** uses the game's quick-park feature, which depends on the game's
  parking settings. There is no reverse-parking automation.
- **The HUD needs borderless or windowed mode.** It is a click-through overlay
  window, because the SDK has no UI extension, so exclusive fullscreen may hide
  it. `status.json` carries the same data.
- `.zip` map mods are not read. Base game and installed DLC are.

## Supported ATS version

Built against **SCS SDK 1.15** (telemetry 1.07, input 1.00, ATS 1.61+). The map
parser targets sector format **907** and prefab format **25**; the game-memory
layer targets **1.61**. Other versions are rejected cleanly with a log message
rather than misread.

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
4. ATSPilot obeys traffic lights and stop signs by itself. If a light's state
   cannot be read, it stops and waits: **tap the throttle** to go. Holding the
   throttle means you are taking over.
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
- `[memory]`: game-memory features (steering, traffic, lights, GPS route)
- `[traffic]`: following gap, standstill gap, amber-light braking limit
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
- **"Game memory features need game version ..."**: your game version is not
  supported by the memory layer. ATSPilot drives on map data alone.

## Development

Requirements: Windows x64, Visual Studio 2022/2026 (or Build Tools) with C++.
CMake comes with Visual Studio; zlib and doctest are fetched automatically.

```powershell
./build.ps1          # build, 110 unit/controller tests, simulator, package build/release/ATSPilot
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

1. In-game validation of direct steering, light matching and traffic reading.
2. Lane changes that check traffic approaching from behind.
3. Better trailer-aware steering, and reverse parking.

## Privacy and security

Fully offline. No telemetry leaves your computer, no network ports are opened,
and no downloaded code runs. Game memory is read inside the game process only;
nothing is shared with other programs.

## License

MIT; see [LICENSE](LICENSE) and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
Not affiliated with or endorsed by SCS Software.
