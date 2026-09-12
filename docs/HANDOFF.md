# Handoff notes between the two development machines

Two Claude Code sessions work on this project: one on the Windows laptop (owner of the navigator, perception
and simulator; develops the web interface) and one on the Raspberry Pi 5 ground station (owner of the Pi
environment, networking, hardware and deployment). They communicate through this repository:

* code and this file are the single source of truth: `git pull` before working, commit with a clear message, `git push`;
* append dated entries below (newest first) with what was done, what is needed from the other side, and open issues;
* the laptop session reaches the Pi over Tailscale SSH for hands-on work; the Pi session must never send flight
  commands to the drone (dry-run and telemetry tests only) unless the user is present and asks for it.

Drone facts: RoboMaster TT (Tello Talent), direct-mode SSID `TELLO-FE18DB` (open network), drone IP 192.168.10.1,
SDK 3.0 (`sdk?` -> 30). Battery 1100 mAh, ~13 min. Board temperature rises fast when powered on without flying:
keep it off between tests. Windows laptop: single Wi-Fi radio, home SSID INFINITUM2C06 (2.4 GHz).

## Seeing the Pi from the laptop

* **Desktop**: enable VNC on the Pi (`sudo raspi-config nonint do_vnc 0`; on Bookworm this is wayvnc, set a
  headless resolution with `sudo raspi-config nonint do_vnc_resolution 1920x1080` if no monitor is attached).
  Laptop: `scriptsnc_pi.ps1 <pi-address>` opens it with the portable TigerVNC viewer in `C:\dev	igervnc`.
* **Terminal of the Pi's Claude session**: start Claude Code on the Pi inside `tmux new -s claude`. From the
  laptop, `scripts\pi_shell.ps1 <pi-address>` attaches to that same session (both sides see the same screen;
  Ctrl+B then D detaches). The laptop session uses plain `ssh` for its own work in other tmux windows.
* **Drone video and controls**: the web interface served by tello_race on the Pi (in development), opened in a
  browser at `http://<pi-address>:8080`.

## Entries

### 2026-09-12 (laptop)
* Repository pushed with the simulator-validated navigator (58/60 random courses), first contact with the real
  drone verified (telemetry 10 Hz, video 960x720 at ~24 fps), belly camera switch (`downvision`) added.
* Next on the laptop: embedded web interface (civetweb: MJPEG stream + WebSocket telemetry/commands) so the
  ground station can be operated from any browser.
* Needed from the Pi: Tailscale IP + SSH access, OS/OpenCV versions, build + `sim` smoke test result, the
  NetworkManager profile for `TELLO-FE18DB` with `ipv4.never-default yes`, and a dry-run `telemetry` result.
