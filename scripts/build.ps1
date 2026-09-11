# Configure + build tello_race with the portable MinGW toolchain (see scripts/setup_toolchain.ps1).
# The build directory lives outside OneDrive (C:\dev\build\tello-race) to avoid file locks and sync noise.
param([switch]$Clean)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$bin = "C:\dev\build\tello-race"
$env:Path = "C:\dev\mingw64\bin;C:\dev\cmake\bin;C:\dev\ninja;" + $env:Path
if ($Clean -and (Test-Path $bin)) { Remove-Item -Recurse -Force $bin }
cmake --preset mingw-release -S $root
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
cmake --build $bin
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
Write-Host "`nBuilt: $bin\tello_race.exe" -ForegroundColor Green
