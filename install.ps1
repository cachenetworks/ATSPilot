<#
.SYNOPSIS
    Installs (or removes) the ATSPilot plugin into American Truck Simulator.

.DESCRIPTION
    Copies plugins\atspilot.dll to <ATS>\bin\win_x64\plugins\. The ATS folder is
    found through Steam's library list unless -GameDir is given. Configuration,
    logs and the map cache live in Documents\American Truck Simulator\atspilot
    and are created by the plugin on first start.

.EXAMPLE
    ./install.ps1
    ./install.ps1 -GameDir "D:\SteamLibrary\steamapps\common\American Truck Simulator"
    ./install.ps1 -Uninstall
#>
param(
    [string]$GameDir,
    [switch]$Uninstall
)

$ErrorActionPreference = 'Stop'

function Find-Ats {
    $steam = (Get-ItemProperty 'HKCU:\Software\Valve\Steam' -ErrorAction SilentlyContinue).SteamPath
    if (-not $steam) { return $null }
    $libraries = @($steam)
    $vdf = Join-Path $steam 'steamapps\libraryfolders.vdf'
    if (Test-Path $vdf) {
        foreach ($m in (Select-String -Path $vdf -Pattern '"path"\s+"([^"]+)"').Matches) {
            $libraries += $m.Groups[1].Value -replace '\\\\', '\'
        }
    }
    foreach ($lib in $libraries | Select-Object -Unique) {
        # Libraries on drives that no longer exist (unplugged, removed) are skipped.
        if (-not (Test-Path -LiteralPath $lib)) { continue }
        $candidate = [System.IO.Path]::Combine($lib, 'steamapps\common\American Truck Simulator')
        if (Test-Path (Join-Path $candidate 'bin\win_x64\amtrucks.exe')) { return $candidate }
    }
    return $null
}

if (-not $GameDir) { $GameDir = Find-Ats }
if (-not $GameDir -or -not (Test-Path (Join-Path $GameDir 'bin\win_x64\amtrucks.exe'))) {
    throw 'American Truck Simulator was not found. Pass -GameDir "<path to American Truck Simulator>".'
}

$pluginDir = Join-Path $GameDir 'bin\win_x64\plugins'
$target = Join-Path $pluginDir 'atspilot.dll'

if ($Uninstall) {
    if (Test-Path $target) { Remove-Item $target; Write-Host "Removed $target" } else { Write-Host 'ATSPilot is not installed.' }
    return
}

$source = Join-Path $PSScriptRoot 'plugins\atspilot.dll'
if (-not (Test-Path $source)) { throw "Plugin not found at $source" }
New-Item -ItemType Directory -Force $pluginDir | Out-Null
Copy-Item $source $target -Force
Write-Host "Installed $target"

$data = Join-Path ([Environment]::GetFolderPath('MyDocuments')) 'American Truck Simulator\atspilot'
$cfg = Join-Path $data 'atspilot.toml'
if (-not (Test-Path $cfg) -and (Test-Path (Join-Path $PSScriptRoot 'config\atspilot.toml'))) {
    New-Item -ItemType Directory -Force $data | Out-Null
    Copy-Item (Join-Path $PSScriptRoot 'config\atspilot.toml') $cfg
    Write-Host "Default configuration written to $cfg"
}
Write-Host 'Start ATS and accept the "SDK features" dialog. The first start builds the map cache (~15-30 s, in the background).'
