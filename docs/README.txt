ATSPilot - autopilot plugin for American Truck Simulator (pre-alpha)
===================================================================

Contents
  plugins\atspilot.dll   the ATS plugin (SCS SDK telemetry + input)
  config\atspilot.toml   default configuration (copied on install if none exists)
  tools\                 development tools (map validation, controller simulator)
  install.ps1            installs or removes the plugin

Install
  1. Right-click install.ps1 > Run with PowerShell
     (or: powershell -ExecutionPolicy Bypass -File install.ps1 [-GameDir "<ATS folder>"])
     The plugin is copied to <ATS>\bin\win_x64\plugins\atspilot.dll
  2. Start ATS and accept the "SDK features" prompt.
  3. The first start builds the map cache in the background (about 15-30 s).

Files
  Documents\American Truck Simulator\atspilot\
    atspilot.toml  configuration
    logs\          atspilot.log (rotated)
    cache\         parsed map cache (rebuilt automatically after updates)
    recordings\    telemetry CSVs when debug.record_telemetry = true
    status.json    live status for optional overlays

Controls (configurable)
  F9 autopilot | F8 lane assist | Insert cruise | = / - set speed
  Shift+F9 resume | Delete cancel | Shift+Delete emergency disable
  Braking, steering or throttle input takes over immediately.

ATSPilot has no traffic awareness. Stay attentive.
Full documentation: https://github.com/cachenetworks/ATSPilot
