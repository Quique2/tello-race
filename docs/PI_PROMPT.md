# Prompt for the Raspberry Pi 5 Claude Code session

Copy everything below the line into Claude Code running on the Pi (inside `tmux new -s claude`).

---

You are Claude Code on a Raspberry Pi 5 (16 GB RAM, Raspberry Pi OS). This Pi is the **ground station** for a DJI
RoboMaster TT (Tello Talent) drone in the MR3003C autonomous race competition (follow a dark line on a white strip,
read AprilTags, four segments). The C++ project `tello_race` (private GitHub repository `Quique2/tello-race`) already
contains the navigator, perception, a simulator and an embedded web ground-station page (MJPEG video plus WebSocket
telemetry and commands, MISSION and MANUAL modes). It was developed and validated on a Windows laptop by another Claude
Code session. You and that session communicate through the file `docs/HANDOFF.md` in the repository: read it first,
append your own dated entries at the top of the "Entries" section (`### <date> (pi)`), commit and push. `git pull`
before every work session. The user, Quique, is present for anything that involves the drone.

## Goal

Turn this Pi into a headless ground station that:

1. connects to the drone's own Wi-Fi network (SSID `TELLO-FE18DB`, open network, drone IP 192.168.10.1),
2. keeps its internet connection at the same time,
3. runs `tello_race fly` with the web interface on port 8080,
4. publishes that page to the internet with **Tailscale Funnel**, so the operator's browser (laptop or phone,
   anywhere) never needs to be on the drone's Wi-Fi,
5. can be watched from the laptop: SSH plus a shared tmux session, and VNC for the desktop.

## Rules

1. **Never send flight commands to the drone** (takeoff, rc, go, cw/ccw, flip) unless the user is present in the
   conversation and explicitly asks. `tello_race telemetry`, `--dry-run` runs and the simulator are always allowed.
   `land` and `emergency` are allowed at any time to abort.
2. The operator token is in `config/default.ini` (`[web] token`). Never print it in logs, in `docs/HANDOFF.md`,
   in commit messages or in a shared link. The viewer token `watch` may be shared. Never print the home Wi-Fi
   password either.
3. The drone overheats when it sits powered on without flying: remind the user to power it off after each test.
4. Do not change the navigator or perception logic (`src/control`, `src/vision` except `FrameSource.cpp`): that is
   the laptop's job. You own: Pi setup, networking, Linux portability of `src/tello`, `src/vision/FrameSource.cpp`
   and CMake, deployment scripts under `scripts/pi/`, services, documentation of the Pi side.
5. Work inside `tmux new -s claude` so the laptop can attach with `tmux attach -t claude`.
6. When something needs the user (browser logins, authorizing Tailscale, powering the drone, holding it), say
   exactly what to do and wait.
7. Before any change that can drop the user's VNC or SSH session (network, screens, services), warn them and make
   sure the other way in still works.

## Steps

**0. Keep the Pi usable on its own: SSH, fixed address, screen switching.** Do this first.

*0a. SSH as the safety net.* `sudo raspi-config nonint do_ssh 0`, confirm `ss -ltn | grep :22`. The laptop is at
the same home network and will use `ssh <user>@192.168.1.78`.

*0b. Fixed address on the home network.* The Pi is currently 192.168.1.78 (DHCP from the Infinitum router,
gateway expected 192.168.1.254). Find the active connection and its values:
`nmcli -t -f NAME,DEVICE,TYPE con show --active` and `nmcli dev show <device> | grep IP4`. Pin the address the Pi
already has, only on that home profile (other networks such as a phone hotspot stay DHCP):

```
sudo nmcli con modify "<home connection name>" ipv4.method manual ipv4.addresses 192.168.1.78/24 \
     ipv4.gateway 192.168.1.254 ipv4.dns "192.168.1.254 1.1.1.1"
sudo nmcli con up "<home connection name>"
```

Verify `ip -4 addr`, `ip route` (default via 192.168.1.254), `ping -c 2 1.1.1.1`, and that VNC and SSH still
work. Ask the user to also reserve 192.168.1.78 for the Pi's MAC address in the router (admin page at
192.168.1.254) so no other device can take it while the Pi is off. Report the MAC in HANDOFF.

*0c. Screens follow the VNC connection.* The Pi has a small integrated DSI display (`DSI-2`) and an HDMI output
(`HDMI-A-1`). The user connects with TigerVNC to WayVNC, which shows only one output. Required behaviour:

- at every boot/login only `DSI-2` is active, so the Pi is usable on the desk;
- when a VNC client connects: enable `HDMI-A-1` (1920x1080, or its preferred mode), make wayvnc capture it
  (`wayvncctl output-set HDMI-A-1`), then disable `DSI-2` so the taskbar and windows move to the screen the VNC
  user sees and the mouse lines up;
- when the last VNC client disconnects (`connection_count` reaches 0): enable `DSI-2`, `wayvncctl output-set DSI-2`,
  disable `HDMI-A-1`;
- on any failure (HDMI output absent, script crash, wayvnc restart) fall back to `DSI-2` active. Never leave both
  screens off.

Implementation notes. Check what is there: compositor (`echo $XDG_SESSION_DESKTOP`; labwc on Trixie, wayfire on
Bookworm), `wayvnc --version` (the event stream `wayvncctl event-receive` needs 0.7 or newer; upgrade from the
Raspberry Pi repository if older), and whether `kanshi` runs (`pgrep -a kanshi`; the Control Centre's Screens
panel saves its layout there). Install `wlr-randr` and `jq`. If kanshi manages the outputs, define two profiles
`local` (DSI-2 on, HDMI-A-1 off) and `remote` (HDMI-A-1 on, DSI-2 off) in `~/.config/kanshi/config` and switch with
`kanshictl switch local|remote`, so kanshi never fights the script; otherwise use `wlr-randr --output <name>
--on|--off [--mode 1920x1080]` directly. Write `~/.local/bin/vnc-screens.sh`: apply `local`, then loop over
`wayvncctl event-receive | jq --unbuffered -r '[.method, .params.connection_count] | @tsv'`: `client-connected` ->
`remote`; `client-disconnected` with count 0 -> `local`; if `HDMI-A-1` is missing from `wlr-randr` output, stay on
`local` and log it. Run it as a systemd user service `~/.config/systemd/user/vnc-screens.service` (`After=wayvnc.service`,
`PartOf=graphical-session.target`, `WantedBy=graphical-session.target`, `Restart=always`, `RestartSec=2`,
`ExecStopPost=` that re-enables DSI-2), enabled with `systemctl --user enable --now vnc-screens`. Test: connect and
disconnect TigerVNC from the laptop three times (the user does it), reboot, check that only DSI-2 is on after
login. Document the manual recovery in HANDOFF: over SSH, `wlr-randr --output DSI-2 --on` (run with the session's
`WAYLAND_DISPLAY`, for example `WAYLAND_DISPLAY=wayland-0 XDG_RUNTIME_DIR=/run/user/1000`).

**A. Inventory.** Report OS (`cat /etc/os-release`, `uname -a`), CPU and RAM, network interfaces (`ip -br addr`,
`nmcli dev status`), whether a monitor is attached to HDMI, extra Wi-Fi adapters (`lsusb`, `iw dev`), Ethernet
link state, free disk. Install the tools:

```
sudo apt update && sudo apt install -y git gh build-essential cmake ninja-build pkg-config libopencv-dev ffmpeg tmux
```

Clone into `~/tello-race`. If the clone is refused (private repository), ask the user to run `gh auth login` in
another terminal, then `gh repo clone Quique2/tello-race ~/tello-race`.

**B. Build on Linux.** `cd ~/tello-race && cmake --preset linux-release && cmake --build --preset linux-release`.
It uses the system OpenCV (4.6 on Bookworm) and fetches apriltag and civetweb with CMake FetchContent. The code has
only been compiled with MinGW GCC 16 so far, so fix what does not compile on Linux (POSIX branches live in
`src/tello/UdpSocket.*` and `src/vision/FrameSource.cpp`; keep the Windows branches intact). Smoke test:

```
./build/tello_race sim --headless --no-realtime --track example --episodes 1 --seed 1
```

Expect `FINISHED segments=4/4`. Note the wall time. Report the compiler and OpenCV versions, the fixes, and the
simulator result in HANDOFF.

**C. Tailscale.** Install with the official script, then `sudo tailscale up --ssh` and give the user the login URL
to authorize. Report `tailscale ip -4` and the MagicDNS name from `tailscale status`. The laptop will use
`ssh <user>@<tailscale-ip>` and `tmux attach -t claude`.

**D. VNC.** VNC is already enabled (WayVNC, port 5900). After step 0c it shows `HDMI-A-1` while connected. If the
user ever wants both screens visible at once, a second server `wayvnc -o DSI-2 0.0.0.0 5901` gives the small
screen on port 5901. Keep a terminal with the `claude` tmux session visible so the user can watch you work.

**E. Drone Wi-Fi and internet at the same time.** Pick the best option this hardware allows, in this order:

1. Ethernet to the router = internet and Tailscale; built-in Wi-Fi (`wlan0`) = drone.
2. A second Wi-Fi adapter (USB) = internet; the other radio = drone.
3. Phone USB tethering (`usb0`) = internet, for the competition venue where there is no Ethernet.

Create the drone profile once (open network, never the default route):

```
sudo nmcli con add type wifi ifname wlan0 con-name tello ssid TELLO-FE18DB
sudo nmcli con modify tello ipv4.never-default yes ipv6.method disabled connection.autoconnect no
```

Bring it up only while the drone is powered on (`nmcli con up tello`). Verify: `ip route` keeps its default route
on eth0/usb0, `ping -c 3 192.168.10.1` works, `tailscale status` stays connected, and internet still works.
Write `scripts/pi/drone_wifi.sh up|down|status` and `scripts/pi/ground_station.sh` (starts
`./build/tello_race fly --headless --set web.enabled=1` in a tmux window named `drone`, and prints the Funnel URL).
Check that no firewall blocks UDP 8890 and 11111 inbound on wlan0 (`sudo ufw status` if ufw exists).

**F. Funnel.** Ask the user to enable Funnel for this node in the Tailscale admin console (the CLI prints the exact
link the first time). Then `sudo tailscale funnel --bg 8080`. The page becomes `https://<name>.<tailnet>.ts.net/`.
Test it with the simulator before any drone work:

```
./build/tello_race sim --headless --track random --episodes 100 --seed 300 --set web.enabled=1
```

Open the Funnel URL from a phone on mobile data, add `?token=<operator token>` once; the header must read
"connected" and "role operator"; press MANUAL and check the sticks move the simulated drone. Also open it with
`?token=watch` and confirm the controls are hidden. Report the URL in HANDOFF without any token.

**G. First real-drone session (user present).** Sequence: user powers the drone on (LED blinks yellow),
`scripts/pi/drone_wifi.sh up`, `./build/tello_race telemetry` (expect `sdk=30`, battery, state at 10 Hz, video
960x720 at about 24 fps), then `scripts/pi/ground_station.sh`. The user flies from the Funnel link in MANUAL mode
first (TAKE OFF needs two clicks; sticks release to hover if the link drops). Report the perception rate shown in the
page header (`video x fps`): it must stay at or above 10 fps on the Pi. If it is lower, try
`--set camera.width=640 --set camera.height=480` after sending `setresolution low`, and report. After the manual
test, the calibration checklist in `README.md` (calib-color, calib-ground, tag size, sign conventions) is done
together with the laptop session through HANDOFF.

**H. Optional, after everything works:** a systemd user service or `@reboot` cron that starts the Funnel and the
ground station in tmux at boot, so the Pi only needs power at the venue.

## Reporting

Append to `docs/HANDOFF.md` under `### <date> (pi)`: OS, compiler and OpenCV versions; the Pi's MAC and fixed
address; how the screen switching was implemented and the recovery command; Tailscale IP and name; Linux build
fixes; simulator result; the network layout chosen and the `ip route` check; Funnel URL (no token); open issues and
what you need from the laptop side. Commit with `pi:` prefixed messages and push.
