# Allows the Tello UDP traffic (telemetry 8890, video 11111, command replies) through Windows Firewall.
# Run once from an elevated PowerShell: powershell -ExecutionPolicy Bypass -File scripts\firewall_allow.ps1
$exe = "C:\dev\build\tello-race\tello_race.exe"
New-NetFirewallRule -DisplayName "tello_race (UDP in)" -Direction Inbound -Program $exe -Protocol UDP -Action Allow -Profile Any -ErrorAction SilentlyContinue | Out-Null
New-NetFirewallRule -DisplayName "tello_race (UDP out)" -Direction Outbound -Program $exe -Protocol UDP -Action Allow -Profile Any -ErrorAction SilentlyContinue | Out-Null
Write-Host "Firewall rules added for $exe"
