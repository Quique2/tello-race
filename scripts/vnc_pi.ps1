# Open the Raspberry Pi ground-station desktop on this laptop (TigerVNC viewer, portable, no install).
#   scripts\vnc_pi.ps1 100.x.y.z          (Pi's Tailscale or LAN address; port 5900 by default)
# The Pi side must have VNC enabled (raspi-config -> Interface Options -> VNC); log in with the Pi user's password.
param([Parameter(Mandatory=$true)][string]$PiAddress, [int]$Port = 5900)
$viewer = "C:\dev\tigervnc\vncviewer64.exe"
if (-not (Test-Path $viewer)) { Write-Error "TigerVNC viewer not found at $viewer (download vncviewer64-<version>.exe from sourceforge.net/projects/tigervnc)"; exit 1 }
& $viewer "${PiAddress}:${Port}"
