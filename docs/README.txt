ATSPilot - autopilot plugin for ATS and ETS2 (pre-alpha)
=========================================================

Contents
  plugins\atspilot.dll   the ATS/ETS2 plugin (SCS SDK telemetry + input)
  config\atspilot.toml   default configuration (copied on install if none exists)
  tools\                 development tools (map validation, controller simulator)
  install.ps1            installs or removes the plugin

Install
  1. Right-click install.ps1 > Run with PowerShell
     (or: powershell -ExecutionPolicy Bypass -File install.ps1 [-Game ATS|ETS2] [-GameDir "<game folder>"])
     With no -Game argument, every detected supported game is installed.
  2. Start the game and accept the "SDK features" prompt.
  3. The first start builds the map cache in the background (about 15-30 s).

Files
  Documents\American Truck Simulator\atspilot\
  Documents\Euro Truck Simulator 2\atspilot\
    atspilot.toml  configuration
    logs\          atspilot.log (rotated)
    cache\         parsed map cache (rebuilt automatically after updates)
    recordings\    telemetry CSVs when debug.record_telemetry = true
    status.json    live status for optional overlays

Controls (configurable)
  F9 toggles ATSPilot on/off. Set speed with the game's own cruise-control keys.
  Braking, steering or throttle input takes over immediately.

ATSPilot has no traffic awareness. Stay attentive.
Full documentation: https://github.com/cachenetworks/ATSPilot
