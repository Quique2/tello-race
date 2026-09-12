# Switch this laptop's Wi-Fi to the drone (default TELLO-FE18DB) or back to a home/school network.
#   scripts\wifi_tello.ps1            -> join the drone (open network profile is created if missing)
#   scripts\wifi_tello.ps1 -Back      -> reconnect to the last known internet network (-Home overrides the SSID)
# The drone's own IP is always 192.168.10.1. With a single Wi-Fi radio the internet is unavailable while joined;
# use Ethernet or USB tethering for internet if the chat/other tools must stay online during a session.
param([switch]$Back, [string]$Drone = "TELLO-FE18DB", [string]$Home = "INFINITUM2C06")
$ErrorActionPreference = "Stop"
if ($Back) {
    netsh wlan connect name="$Home" | Out-Null
    Write-Host "Reconnecting to $Home ..."; Start-Sleep 4; netsh wlan show interfaces | Select-String "SSID|State"
    exit 0
}
$profiles = netsh wlan show profiles | Select-String "All User Profile" | ForEach-Object { ($_ -split ":")[1].Trim() }
if ($profiles -notcontains $Drone) {
    $xml = @"
<?xml version="1.0"?>
<WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">
  <name>$Drone</name>
  <SSIDConfig><SSID><name>$Drone</name></SSID><nonBroadcast>false</nonBroadcast></SSIDConfig>
  <connectionType>ESS</connectionType>
  <connectionMode>manual</connectionMode>
  <MSM><security><authEncryption><authentication>open</authentication><encryption>none</encryption><useOneX>false</useOneX></authEncryption></security></MSM>
</WLANProfile>
"@
    $tmp = Join-Path $env:TEMP "tello_profile.xml"; Set-Content -Path $tmp -Value $xml -Encoding ASCII
    netsh wlan add profile filename="$tmp" user=current | Out-Null
}
netsh wlan connect name="$Drone" | Out-Null
Write-Host "Joining $Drone ..."; Start-Sleep 4
netsh wlan show interfaces | Select-String "SSID|State"
ping -n 1 -w 1000 192.168.10.1 | Select-String "Reply|Request timed out"
Write-Host "Next: C:\dev\build\tello-race\tello_race.exe telemetry --duration 10"
