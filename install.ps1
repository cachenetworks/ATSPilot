<#
.SYNOPSIS
    Installs or removes ATSPilot for American Truck Simulator and Euro Truck Simulator 2.

.DESCRIPTION
    Copies plugins\atspilot.dll into each selected game's bin\win_x64\plugins folder.
    Steam libraries are detected automatically unless -GameDir is supplied. With no
    -Game argument, all detected supported games are installed; -Game selects one.

.EXAMPLE
    ./install.ps1
    ./install.ps1 -Game ATS
    ./install.ps1 -Game ETS2
    ./install.ps1 -Game ETS2 -GameDir "D:\SteamLibrary\steamapps\common\Euro Truck Simulator 2"
    ./install.ps1 -Uninstall
#>
param(
    [ValidateSet('ATS', 'ETS2')]
    [string]$Game,
    [string]$GameDir,
    [switch]$Uninstall
)

$ErrorActionPreference = 'Stop'

$gameDefinitions = @{
    ATS = @{
        Name = 'American Truck Simulator'
        Exe = 'amtrucks.exe'
        SteamDir = 'American Truck Simulator'
        DocumentsDir = 'American Truck Simulator'
    }
    ETS2 = @{
        Name = 'Euro Truck Simulator 2'
        Exe = 'eurotrucks2.exe'
        SteamDir = 'Euro Truck Simulator 2'
        DocumentsDir = 'Euro Truck Simulator 2'
    }
}

function Get-SteamLibraries {
    $steam = (Get-ItemProperty 'HKCU:\Software\Valve\Steam' -ErrorAction SilentlyContinue).SteamPath
    if (-not $steam) { return @() }
    $libraries = @($steam)
    $vdf = Join-Path $steam 'steamapps\libraryfolders.vdf'
    if (Test-Path $vdf) {
        foreach ($m in (Select-String -Path $vdf -Pattern '"path"\s+"([^"]+)"').Matches) {
            $libraries += $m.Groups[1].Value -replace '\\\\', '\'
        }
    }
    return @($libraries | Select-Object -Unique | Where-Object { Test-Path -LiteralPath $_ -ErrorAction SilentlyContinue })
}

function Find-GameDir([string]$Key) {
    $definition = $gameDefinitions[$Key]
    foreach ($library in Get-SteamLibraries) {
        $candidate = [System.IO.Path]::Combine($library, 'steamapps\common', $definition.SteamDir)
        if (Test-Path (Join-Path $candidate "bin\win_x64\$($definition.Exe)")) { return $candidate }
    }
    return $null
}

function Resolve-GameFromDir([string]$Path) {
    foreach ($key in @('ATS', 'ETS2')) {
        $definition = $gameDefinitions[$key]
        if (Test-Path (Join-Path $Path "bin\win_x64\$($definition.Exe)")) { return $key }
    }
    return $null
}

function Install-ForGame([string]$Key, [string]$Path, [string]$Source) {
    $definition = $gameDefinitions[$Key]
    $exe = Join-Path $Path "bin\win_x64\$($definition.Exe)"
    if (-not (Test-Path $exe)) { throw "$($definition.Name) executable was not found under $Path" }

    $pluginDir = Join-Path $Path 'bin\win_x64\plugins'
    $target = Join-Path $pluginDir 'atspilot.dll'
    if ($Uninstall) {
        if (Test-Path $target) {
            Remove-Item $target
            Write-Host "Removed $target"
        } else {
            Write-Host "ATSPilot is not installed for $($definition.Name)."
        }
        return
    }

    New-Item -ItemType Directory -Force $pluginDir | Out-Null
    Copy-Item $Source $target -Force
    Write-Host "Installed $target"

    $data = Join-Path ([Environment]::GetFolderPath('MyDocuments')) "$($definition.DocumentsDir)\atspilot"
    $cfg = Join-Path $data 'atspilot.toml'
    $defaultCfg = Join-Path $PSScriptRoot 'config\atspilot.toml'
    if (-not (Test-Path $cfg) -and (Test-Path $defaultCfg)) {
        New-Item -ItemType Directory -Force $data | Out-Null
        Copy-Item $defaultCfg $cfg
        Write-Host "Default configuration written to $cfg"
    }
}

$source = Join-Path $PSScriptRoot 'plugins\atspilot.dll'
if (-not $Uninstall -and -not (Test-Path $source)) {
    $releaseSource = Join-Path $PSScriptRoot 'build\release\ATSPilot\plugins\atspilot.dll'
    if (Test-Path $releaseSource) {
        $source = $releaseSource
    } else {
        throw "Plugin not found at $source or $releaseSource"
    }
}

$targets = @()
if ($GameDir) {
    $resolved = Resolve-GameFromDir $GameDir
    if (-not $resolved) { throw "No supported ATS/ETS2 executable was found under $GameDir" }
    if ($Game -and $resolved -ne $Game) { throw "-Game $Game does not match the executable found under $GameDir" }
    $targets += [pscustomobject]@{ Key = $resolved; Path = $GameDir }
} elseif ($Game) {
    $path = Find-GameDir $Game
    if (-not $path) { throw "$($gameDefinitions[$Game].Name) was not found in the configured Steam libraries." }
    $targets += [pscustomobject]@{ Key = $Game; Path = $path }
} else {
    foreach ($key in @('ATS', 'ETS2')) {
        $path = Find-GameDir $key
        if ($path) { $targets += [pscustomobject]@{ Key = $key; Path = $path } }
    }
    if ($targets.Count -eq 0) {
        throw 'Neither American Truck Simulator nor Euro Truck Simulator 2 was found. Pass -Game and/or -GameDir.'
    }
}

foreach ($target in $targets) { Install-ForGame $target.Key $target.Path $source }

if (-not $Uninstall) {
    Write-Host 'Start the game and accept the "SDK features" dialog. The first start builds that game''s map cache in the background.'
}
