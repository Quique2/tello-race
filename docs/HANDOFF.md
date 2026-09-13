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
  Laptop: `scripts/vnc_pi.ps1 <pi-address>` opens it with the portable TigerVNC viewer in `C:/dev/tigervnc`.
* **Terminal of the Pi's Claude session**: start Claude Code on the Pi inside `tmux new -s claude`. From the
  laptop, `scripts/pi_shell.ps1 <pi-address>` attaches to that same session (both sides see the same screen;
  Ctrl+B then D detaches). The laptop session uses plain `ssh` for its own work in other tmux windows.
* **Drone video and controls**: the web interface served by tello_race on the Pi (in development), opened in a
  browser at `http://<pi-address>:8080`.

## Entries

### 2026-09-12 19:30 (pi) VNC cycle test: both switches work, wayvnc crashes afterwards with DSI-2 captured; Funnel on
* Sequence (output state polled every 0.5 s + journal):
  - 19:24:10.03 client disconnected (count 0) -> 19:24:10 DSI-2 on -> 19:24:11 capture DSI-2 -> 19:24:12 HDMI-A-1
    off -> 19:24:13.44 `local: enabled DSI-2, wayvnc captures DSI-2`. wayvnc (PID 1684) **survived all three steps**.
  - 19:24:14.64 wayvnc 1684 segfault (status=139), 2 s after the last change, with 0 clients and no output change
    in progress. Then more segfaults of each restarted wayvnc: 19:24:15.17, 19:24:31.00, 19:24:40.48, 19:24:51.05,
    19:24:59.47 (NRestarts=7). None has a pam/auth line, so the crashes come before authentication. vnc-screens
    followed each restart (`wayvnc shut down` -> go_local; DSI-2 was already on, so there was no output commit).
  - 19:25:45.13 client authenticated -> `client connected (count 1)` -> 19:25:46 HDMI-A-1 on -> capture HDMI ->
    19:25:47 DSI-2 off -> 19:25:48.37 `remote`. wayvnc (PID 14385) **survived** and is still running.
* So the on/off switching doesn't crash wayvnc anymore; the crashes happen while DSI-2 is captured and no client is
  connected. Two candidate triggers, not yet separated: (a) TigerVNC reconnect attempts hitting a wayvnc that captures
  DSI-2 (the `ext-image-copy-capture: No supported buffer formats` error shows up on every connect), or (b) my test
  poller, which called `wayvncctl output-list` and `client-list` every 0.5 s. The crashes stopped when I killed that
  poller (~19:25:05–10), but also 45 s before the successful connect. It was replaced by a poller without wayvncctl
  (outputs, wayvnc PID, established TCP connections on 5900). `VNC_SCREENS_TOGGLE` stays 1 so the next test changes
  only one variable. If crashes still follow the disconnect, the next step is `VNC_SCREENS_TOGGLE=0` plus keeping the
  capture on HDMI-A-1.
* Funnel: approved; `tailscale funnel status` = `https://quiquepi.tail2b1864.ts.net` -> proxy `http://127.0.0.1:8080`,
  and `/api/status?token=watch` answers through that URL (sim running in `claude:sim`).

### 2026-09-12 19:25 (pi) boot layout restored, vnc-screens enabled, Funnel pending approval, simulator on the web
* This is a new Claude Code session on the Pi (tmux `claude:0`), continuing the paused VS Code Pi session.
* Why HDMI was on and DSI off after the reboot: at 18:54:21 someone saved an HDMI-only layout in the Control Centre's
  Screens panel over VNC. That panel rewrites `~/.config/kanshi/config` (anonymous profile HDMI on / DSI off, which
  matches first) **and** copies it to `/etc/xdg/labwc-greeter/config.kanshi` with sudo. Fix: `install.sh` now also
  installs `kanshi.config` (profiles `local` + `dsi-only`) to the greeter path. Re-ran it; backups of the panel
  versions are `*.bak-screens-panel`. **Don't save layouts in the Screens panel**; if someone does, re-run
  `scripts/pi/vnc-screens/install.sh`. kanshi was not reloaded (a VNC client was connected).
* `systemctl --user enable --now vnc-screens` at 19:17:20. With 1 client connected it logged `remote: enabled
  HDMI-A-1, wayvnc captures HDMI-A-1` without changing any output. The service log is in the system journal
  (`journalctl -b | grep vnc-screens.sh`), not in `journalctl --user`.
* VNC disconnect/reconnect test: armed, waiting for the user. The journal runs in tmux `claude:journal`; output state
  and wayvnc PID changes are logged every 0.5 s (wayvnc PID 1684 at the start). Results go in the next entry.
* Funnel: not enabled on the tailnet yet. The approval link was given to the user in the terminal. `sudo tailscale
  funnel --bg 8080` waits in tmux `claude:funnel` and finishes once it's approved. URL: `https://quiquepi.tail2b1864.ts.net/`.
* Simulator with the web page in tmux `claude:sim`: `./build/tello_race sim --headless --track random --episodes 100
  --seed 300 --set web.enabled=1` (the binary from 17:49 is newer than the last source commit).
  `curl -s localhost:8080/api/status?token=watch` returns JSON (`phase FOLLOW`, `fps 17.6`, `drone sim`).
* Pending: user approves Funnel, then phone test (operator/viewer tokens) per PI_PROMPT F; VNC cycle result.

### 2026-09-12 18:50 (pi) eth0 pinned to 192.168.137.10, Tailscale re-established
* User gave the go for step 0b. The change had already been applied at 18:40:38 by the laptop session over Tailscale
  SSH (`nmcli con modify netplan-eth0 ipv4.method manual ipv4.addresses 192.168.137.10/24 ipv4.gateway 192.168.137.1
  ipv4.dns 192.168.137.1 connection.autoconnect yes` + `con up`); the profile is `netplan-eth0` (NetworkManager with
  netplan backend, written to `/etc/netplan/90-NM-*.yaml`). Home Wi-Fi `netplan-wlan0-INFINITUM2C06` stays DHCP.
* Verified on the Pi: `ip -br addr` eth0 = 192.168.137.10/24; `ip route` = `default via 192.168.137.1 dev eth0 proto
  static metric 100` (wlan0 default metric 600 as backup); `ping 1.1.1.1` ok (~23 ms); `curl -sI https://github.com`
  -> HTTP/2 200; sshd and wayvnc listen on all addresses (22, 5900). `ping 192.168.137.1` gets no reply: Windows
  Firewall blocks ICMP on the ICS adapter, traffic through it works.
* Side effect: after the address change tailscaled kept timing out its control long-poll (`map response long-poll
  timed out`, node shown offline) although peer traffic worked. `sudo systemctl restart tailscaled` at 18:48 fixed
  it: `BackendState=Running`, online, `quiquepi.tail2b1864.ts.net` 100.112.7.35, `tailscale ping bulmacompu` direct
  via 192.168.1.75. If eth0 changes address again, restart tailscaled.
* Pending: user check that TigerVNC works at 192.168.137.10:5900; reboot for the HDMI/vnc-screens test (18:45 entry).

### 2026-09-12 18:45 (pi) wayvnc crash: root cause is a stuck HDMI-A-1 in labwc, script rewritten, reboot pending
* Read the 18:30 laptop entry. Timeline from the journal plus `~/.xsession-errors` (labwc log, its timestamps are
  time since labwc start, 15:10:14):
  - 18:13:56 first real disconnect -> old script: `wlr-randr --output DSI-2 --on` while HDMI was the only output
    failed (`Swapchain for output 'DSI-2' failed test`), the kanshi SIGHUP applied "local", HDMI-A-1 was switched off,
    the MacroSilicon dongle dropped hotplug, wayvnc logged `Selected output HDMI-A-1 went away`, the connector came back.
  - from 18:16:19 on, **every** output configuration that touches HDMI-A-1 fails in labwc:
    `drmCrtcGetSequence: Invalid argument` + `connector HDMI-A-1: liftoff_output_apply failed: Operation not permitted`.
    Because labwc applies all outputs at once, even `DSI-2 --on` (already on) failed, and wayvnc segfaulted on those
    failed configurations (18:25:37, 18:25:57). The kernel side is clean (no vc4 errors, connector `connected`,
    labwc is DRM master on card2); `echo on > /sys/class/drm/card2-HDMI-A-1/status` does not clear it. Tested again
    18:34 with the journal open: `wlr-randr --output HDMI-A-1 --on` still fails (wayvnc survived that one).
  - Conclusion: the crash trigger is a failed/no-op output commit, and the HDMI output stays broken until labwc
    restarts. Only a reboot (or logout) recovers it.
* Changes (all in `scripts/pi/vnc-screens/`, installed with `install.sh`):
  - `vnc-screens.sh` rewritten as suggested: sequential `wlr-randr` steps with 1 s pauses, each step skipped when it
    would change nothing, state checked after each step, never switches off the last enabled output, no kanshi
    SIGHUP (kanshi config now only holds the boot/hotplug layout: `local` + `dsi-only`). connect -> HDMI on, 1 s,
    `output-set HDMI-A-1`, 1 s, DSI off; last disconnect -> DSI on, 1 s, `output-set DSI-2`, 1 s, HDMI off.
    `VNC_SCREENS_TOGGLE=0` keeps both outputs on and only switches the capture (fallback test).
  - `ExecStopPost` calls `vnc-screens.sh --ensure-local` (turns DSI-2 on only if it is off: no no-op commit).
  - `/boot/firmware/cmdline.txt` += `video=HDMI-A-1:1920x1080@60D` (backup `cmdline.txt.bak-vnc-screens`) so the
    connector can not disappear when the dongle loses signal; the hotplug is what preceded the stuck state.
  - The service is **disabled** for the reboot; it is re-enabled only after the manual test with the journal.
* Next (after the user reboots): check both outputs exist, run the connect/disconnect sequence by hand with the journal
  in tmux window `claude:journal`, then with the service; if wayvnc still segfaults, `VNC_SCREENS_TOGGLE=0` and a
  wayvnc drop-in without `--gpu`.
* Manual recovery over SSH if both screens end up off:
  `WAYLAND_DISPLAY=wayland-0 XDG_RUNTIME_DIR=/run/user/1000 wlr-randr --output DSI-2 --on`
  (and `systemctl --user stop vnc-screens`).
* Not done: the laptop SSH key from PI_PROMPT 0a. The Pi session's permission check refused to add a key taken from
  a repository file; the user will confirm or add it. Tailscale SSH works meanwhile.

### 2026-09-12 18:30 (laptop) VNC crash found: wayvnc segfaults when vnc-screens switches outputs
* Journal on the Pi (`journalctl --since "10 min ago" | grep -iE "wayvnc|vnc-screens"`): on every VNC client connect
  (18:25:37, 18:25:57) `wayvnc[...]: Segmentation fault`, `wayvnc.service ... status=139`, at the same instant
  `vnc-screens.sh: failed to apply configuration` / `could not enable DSI-2`, then systemd restarts wayvnc and the
  script goes back to local. The user sees TigerVNC "readTLS: The TLS connection was non-properly terminated (-110)".
  Cause: the client-connected handler changes the outputs (kanshi SIGHUP applies enable HDMI + disable DSI atomically,
  plus `output-set`) while wayvnc (0.9.1, `--gpu --detached`) is capturing the output that gets disabled -> crash.
* Laptop action (reversible): `systemctl --user stop vnc-screens` so VNC works again on DSI-2 only. The service is
  still enabled; restart it once fixed.
* Suggested fix for the Pi session: never disable the output wayvnc is capturing in the same step as the switch. Do it
  sequentially with wlr-randr, no kanshi SIGHUP in the hot path (write the kanshi config for boot only):
  connect -> `wlr-randr --output HDMI-A-1 --on --mode 1920x1080`; sleep 1; `wayvncctl output-set HDMI-A-1`; sleep 1;
  `wlr-randr --output DSI-2 --off`. Disconnect (count 0) -> `wlr-randr --output DSI-2 --on`; sleep 1;
  `wayvncctl output-set DSI-2`; sleep 1; `wlr-randr --output HDMI-A-1 --off`. If wayvnc still segfaults, test with
  both outputs kept enabled (only `output-set`), and try wayvnc without `--gpu` (systemd drop-in overriding ExecStart
  of wayvnc.service / wayvnc-run.sh). Verify with the journal open in another tmux window while the user connects.
* Tailscale is up on both sides: laptop `bulmacompu` 100.111.200.37, Pi `quiquepi` 100.112.7.35; Tailscale SSH from
  the laptop works (`ssh quiquepi@100.112.7.35`, no key needed). HDMI-A-1 is a MacroSilicon HDMI device (capture
  dongle), reported `connected`, so no cmdline `video=` forcing is needed while it stays plugged in.

### 2026-09-12 (pi) setup, Linux build, scripts
* Pi: Raspberry Pi 5 Model B Rev 1.1, 4 cores, 16 GB RAM, 57 GB SD (46 GB free). Debian GNU/Linux 13 (trixie),
  kernel 6.18.34+rpt-rpi-2712 aarch64, labwc desktop (rpd-labwc), wayvnc 0.9.1, kanshi 1.5.1. No extra Wi-Fi adapter.
  Username **quiquepi**, hostname QuiquePi. SSH enabled (port 22). eth0 = laptop ICS (still 192.168.137.168 by DHCP
  at the time of writing; pinning to 192.168.137.10 is pending the user's go), wlan0 = home Wi-Fi (DHCP, untouched).
* Toolchain: gcc (Debian 14.2.0-19), cmake 3.31.6, ninja, system OpenCV **4.10.0** (Trixie, not 4.6).
* Linux build: `cmake --preset linux-release && cmake --build --preset linux-release` compiled with **no source
  changes needed** (only CMake deprecation warnings from civetweb's CMakeLists).
* Smoke test: `./build/tello_race sim --headless --no-realtime --track example --episodes 1 --seed 1` ->
  `FINISHED segments=4/4 time=46.8s` (30 s wall clock on the Pi).
* Drone profile created: `nmcli con add type wifi ifname wlan0 con-name tello ssid TELLO-FE18DB` with
  `ipv4.never-default yes`, `ipv6.method disabled`, `autoconnect no`. No firewall on the Pi (nftables empty, no ufw).
* New scripts: `scripts/pi/drone_wifi.sh up|down|status` (joins/leaves the drone network, prints routes, drone ping,
  internet, Tailscale) and `scripts/pi/ground_station.sh [start|stop|status] [tello_race options]` (runs
  `tello_race fly --headless --set web.enabled=1` in tmux window `claude:drone`, enables Funnel on 8080, prints URLs).
* Removed the operator token value from the 2026-09-12 laptop entry below (rule: never in HANDOFF). It is still in
  git history; consider changing it in `config/default.ini` before the page is public.
* In progress: Tailscale (installed 1.102.4, waiting for the user to authorize), fixed eth0 address, screen
  switching test, Funnel.

### 2026-09-12 (laptop) tokens and Pi deployment plan
* Network layout chosen on 2026-09-12: the Pi is wired to the laptop's Ethernet port and the laptop shares its
  Wi-Fi internet over the cable (Windows Internet Connection Sharing, laptop = 192.168.137.1, service set to start
  automatically). The Pi pins 192.168.137.10 on eth0 (step 0b of PI_PROMPT.md); its Wi-Fi is free for the drone.
  The laptop reaches the Pi at 192.168.137.10 (VNC 5900, SSH 22, web page 8080) with no internet hop.
* Tailscale: the laptop joined the tailnet as `bulmacompu` (100.111.200.37); the Pi is `quiquepi` (100.112.7.35).
  Laptop scripts `scripts/vnc_pi.ps1` and `scripts/pi_shell.ps1` now default to that address and user `quiquepi`.
* Laptop session's SSH public key (Pi: append to `~/.ssh/authorized_keys`, then the laptop reaches the Pi at
  192.168.137.10 without a password): `ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIISO7xlpSI2GqG78umyexfVa04bICZ8vUTTq3lPiuRgl claude-laptop-tello`
* Observed 2026-09-12 18:18 over VNC: `HDMI-A-1` vanished from the Screens panel and VNC fell back to `DSI-2`; the
  connector is only present while a powered display or dummy plug is attached. PI_PROMPT step 0c now asks to force it
  with `video=HDMI-A-1:1920x1080@60D` in cmdline.txt so remote sessions always have the big screen.
* Tokens decided by the user: operator (full access) `[web] token` and viewer `watch`, both in `config/default.ini`
  (private repo; never paste the operator token into HANDOFF, logs or a shared link).
* `docs/PI_PROMPT.md` is the complete brief for the Pi's Claude Code session: install, Linux build, Tailscale
  (`--ssh`), VNC, drone Wi-Fi + internet at the same time (`ipv4.never-default`), Funnel for port 8080, first
  real-drone session with the user present, reporting format. The Pi session appends its results below.
* Needed from the Pi (in this order): Tailscale IP/name + VNC port, Linux build result (portability fixes are
  welcome in src/tello, src/vision/FrameSource.cpp and CMake only), `sim` smoke test, network profile verified
  (`ip route` default stays on eth0/usb0 while `tello` is up), Funnel URL (without token), then `telemetry` on the
  real drone and the perception fps reported by the web page (must stay >= 10 fps; if lower, try
  `--set camera.width=640 --set camera.height=480` after `setresolution low`, and report).

### 2026-09-12 (laptop) web interface
* Embedded web interface added (civetweb, fetched by CMake with CMAKE_POLICY_VERSION_MINIMUM 3.5): `--set web.enabled=1`,
  page at port 8080, MJPEG stream of the annotated view, WebSocket telemetry at 10 Hz, commands start/land/
  emergency/record/camera/tag gated by `web.token`; `web.viewer_token` = watch only. Verified in the simulator.
* On the Pi: after building, run `./build/tello_race fly --dry-run --headless --set web.enabled=1 --set web.token=<secret>`
  and `tailscale serve --bg 8080` (tailnet) or `tailscale funnel --bg 8080` (public HTTPS link). Report the URL.
* Mission/manual toggle added to the page: `mode manual|mission`, `rc a,b,c,d` (10 Hz from the browser, dead-man 0.5 s in
  MissionLoop), `takeoff` (manual only). The state JSON carries `mode`, `flying` and the applied sticks in `rc`.
  Verified with `ws_test.py` against the simulator: viewer tokens get `{"error":"operator token required"}`.
* Page no longer uses confirm()/alert() (they are blocked in embedded browsers such as the VS Code Simple Browser and
  silently returned false, which is why MANUAL "did nothing"): two-click arming for START/TAKE OFF/EMERGENCY and a
  yellow message bar (#msg) for errors, "not connected" and "token required". One WebServer instance is now owned by
  runFly/runSim, so the browser stays connected across simulator episodes.

### 2026-09-12 (laptop)
* Repository pushed with the simulator-validated navigator (58/60 random courses), first contact with the real
  drone verified (telemetry 10 Hz, video 960x720 at ~24 fps), belly camera switch (`downvision`) added.
* Next on the laptop: embedded web interface (civetweb: MJPEG stream + WebSocket telemetry/commands) so the
  ground station can be operated from any browser.
* Needed from the Pi: Tailscale IP + SSH access, OS/OpenCV versions, build + `sim` smoke test result, the
  NetworkManager profile for `TELLO-FE18DB` with `ipv4.never-default yes`, and a dry-run `telemetry` result.
