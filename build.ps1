<#
.SYNOPSIS
    Builds ATSPilot, runs the tests and assembles build/release/ATSPilot.

.PARAMETER Configuration
    Release (default) or Debug.

.PARAMETER SkipTests
    Build only.

.PARAMETER Clean
    Delete the build directory first.

.PARAMETER Zip
    Also produce build/ATSPilot-<version>.zip.

.EXAMPLE
    ./build.ps1
    ./build.ps1 -Configuration Debug -SkipTests
#>
param(
    [ValidateSet('Release', 'Debug')][string]$Configuration = 'Release',
    [switch]$SkipTests,
    [switch]$Clean,
    [switch]$Zip
)

$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot
$buildDir = Join-Path $root 'build\cmake'
$releaseDir = Join-Path $root 'build\release\ATSPilot'

function Find-CMake {
    $onPath = Get-Command cmake -ErrorAction SilentlyContinue
    if ($onPath) { return $onPath.Source }
    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
    if (Test-Path $vswhere) {
        $installs = & $vswhere -all -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
        foreach ($i in $installs) {
            $candidate = Join-Path $i 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
            if (Test-Path $candidate) { return $candidate }
        }
    }
    throw 'CMake not found. Install Visual Studio 2022/2026 (or Build Tools) with "Desktop development with C++", or put cmake on PATH.'
}

function Assert-Toolchain {
    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
    if (-not (Test-Path $vswhere)) { throw 'Visual Studio installer not found; a MSVC x64 toolchain is required.' }
    $vc = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    if (-not $vc) { throw 'No Visual Studio installation with the MSVC x64 toolset (VC.Tools.x86.x64) was found.' }
    Write-Host "Toolchain: $vc"
}

$cmake = Find-CMake
Write-Host "CMake: $cmake"
Assert-Toolchain

if ($Clean -and (Test-Path $buildDir)) { Remove-Item -Recurse -Force $buildDir }

# Configure: CMake picks the newest installed Visual Studio generator.
& $cmake -S $root -B $buildDir -A x64
if ($LASTEXITCODE -ne 0) { throw 'CMake configure failed' }

& $cmake --build $buildDir --config $Configuration --parallel
if ($LASTEXITCODE -ne 0) { throw 'Build failed' }

if (-not $SkipTests) {
    $ctest = Join-Path (Split-Path $cmake) 'ctest.exe'
    & $ctest --test-dir $buildDir -C $Configuration --output-on-failure
    if ($LASTEXITCODE -ne 0) { throw 'Tests failed' }
    & (Join-Path $buildDir "src\tools\$Configuration\atspilot_sim.exe")
    if ($LASTEXITCODE -ne 0) { throw 'Controller simulation scenarios failed' }
}

# Assemble the release layout.
if (Test-Path $releaseDir) { Remove-Item -Recurse -Force $releaseDir }
New-Item -ItemType Directory -Force "$releaseDir\plugins", "$releaseDir\config", "$releaseDir\tools" | Out-Null
Copy-Item (Join-Path $buildDir "src\plugin\$Configuration\atspilot.dll") "$releaseDir\plugins\"
foreach ($tool in 'atspilot_mapdump.exe', 'atspilot_sim.exe') {
    Copy-Item (Join-Path $buildDir "src\tools\$Configuration\$tool") "$releaseDir\tools\"
}
& (Join-Path $buildDir "src\tools\$Configuration\atspilot_sim.exe") --write-default-config "$releaseDir\config\atspilot.toml"
Copy-Item (Join-Path $root 'install.ps1') $releaseDir
Copy-Item (Join-Path $root 'LICENSE') $releaseDir
Copy-Item (Join-Path $root 'THIRD_PARTY_NOTICES.md') $releaseDir
Copy-Item (Join-Path $root 'docs\README.txt') "$releaseDir\README.txt"

Write-Host ""
Write-Host "Release assembled in $releaseDir"

if ($Zip) {
    $version = (Select-String -Path (Join-Path $root 'CMakeLists.txt') -Pattern 'project\(ATSPilot VERSION ([0-9.]+)').Matches[0].Groups[1].Value
    $zipFile = Join-Path $root "build\ATSPilot-$version.zip"
    if (Test-Path $zipFile) { Remove-Item $zipFile }
    Compress-Archive -Path "$releaseDir\*" -DestinationPath $zipFile
    Write-Host "Archive: $zipFile"
}
