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

### 2026-09-12 20:22 (pi) camera switch test with 01516db: front->belly works, belly->front leaves the drone mute
Binary rebuilt at 20:07:05 (after 01516db). Three power-ons, all joined and brought the page up without help.
* **drone-watch timing:** SSID seen -> joined -> ping OK -> ground station started in about 0.5 s every time
  (20:12:49.7, 20:16:01.9, 20:19:53.8). tello_race connected 0.1–6 s later (the first `command` often times out
  after power-on). Page up about 7 s after the SSID was seen. The drone going silent was handled as designed: the
  ground station stopped after 20 s unreachable (20:15:09) or when the network disappeared (20:19:31), then the Pi
  waited and rejoined on the next power-on.
* **Run 20:12:50** (bat 59 %): the user flew in MANUAL (takeoff +26, land +99). No camera command, no video gaps.
  The drone went silent at +121 s (battery swap).
* **Run 20:16:02** (bat 93 %), all commands from the page by the user:
  - +30.4 and +32.7 `camera` -> `WRN not switching cameras while flying` (guard OK, video unaffected at 24 fps).
  - Land +71.9, takeoff +79.9, land +99.6.
  - **Switch 1, front -> belly, on the ground: OK.** +104.67 `camera` -> +104.81 `video: downward (belly) camera`
    -> `re-requesting the stream (belly camera, attempt 1/3)` -> +105.88 `Opening video stream` -> +105.97
    `video: stream re-opened`. Video back after 1.3 s; **belly fps 15.9–17.2** (front 24). The user then took off
    on the belly camera (+109.5) and landed (+133.3).
  - **Switch 2, belly -> front, on the ground: FAILED.** +139.35 `camera` -> +139.45 `video: front camera` ->
    `re-requesting the stream (front camera, attempt 1/3)` -> +140.57 `Opening video stream` -> +141.01
    `video: stream re-opened`, but **no frames**: +153.31 `ERR video: no frames for a long time, stream ended?` ->
    `re-requesting the stream (stream stopped, attempt 1/3)` -> +154.34 `Opening video stream` -> OpenCV
    `Stream timeout triggered after 30027 ms` -> +184.37 `WRN Low-latency open failed, retrying with FFmpeg
    defaults`. The drone was powered off at ~20:19:12 (+190 s) while that second open was still blocking.
    tello_race **did not exit** (it was stopped by drone-watch when the network went away).
  - Replies: the log has **no** `command '...' answered '...'` warning and no timeout for `streamoff`,
    `downvision 0` or `streamon`. `TelloDrone::command` logs both, so all three were answered `ok` (the reply text is
    only printed at `--log-level debug`). Yet wlan0 received only **1–2 KiB/s** from +141 s on, which is
    telemetry alone: **the drone acknowledged streamon but sent no video after `downvision 0`.** The status `fps`
    stayed frozen at 15.9 (the last value; it isn't reset when frames stop).
* **Run 20:19:54** after the power cycle (bat 77 %): front camera **23.6–23.9 fps**, telemetry true. Power-off still
  clears it.
* For the laptop (nothing changed on the Pi side):
  1. The belly -> front direction is the problem. The drone acks `streamoff` / `downvision 0` / `streamon` and stays
     mute. Possible next steps: a longer settle after `downvision 0` (several seconds), `streamon` repeated until UDP
     bytes arrive, or checking UDP bytes on 11111 before opening FFmpeg instead of trusting `ok`. To see the replies,
     run with `--log-level debug` (`scripts/pi/ground_station.sh start --log-level debug` passes it through).
  2. **The recovery blocks the mission loop:** the FFmpeg open inside `restartVideo()` waited 30 s (+154 -> +184) and
     then started a second open. While it blocks, the loop can't step the navigator, send the dead-man rc or handle
     page commands. That's harmless on the ground, but any path that calls it while flying could leave the drone
     without a land. Open with a short timeout, or open on another thread.
  3. Minor: status `fps` should drop to 0 when frames stop, otherwise the page shows a healthy-looking 15.9.

### 2026-09-12 20:05 (laptop) root cause of the 502 loop: the camera switch kills the video stream
* The user reported the page returning HTTP 502 and said it started when he switched cameras. The log confirms it
  (`logs/ground_station.log`, run of 19:46:52, battery 79 %, six good minutes including a real manual take-off):
  `365.1 land` -> `378.8 web command: camera` -> `video: downward (belly) camera` -> `390.4 camera` ->
  `video: front camera` -> `403.4 ERR video: no frames for a long time, stream ended?` -> exit.
  Every restart after that got **zero** frames from t=0 and died at the same ~12.9 s (the 300-fail counter), so the
  drone had stopped sending video altogether; it only came back after the user powered the drone off.
* Why: the belly camera streams 320x240 grey, the front 960x720 H.264. `downvision` changes the format of the same
  UDP stream, and the FFmpeg decoder cannot follow that mid-stream. The toggle did stop and restart the source, but:
  1. `VideoCaptureSource::start()` never reset `ended_`, so a re-opened source could report "ended" forever;
  2. `start()` kept the previous frame, so a restart could serve one stale frame;
  3. `streamon` was re-sent without a `streamoff` first and with no settling time, so the Tello was left with a
     half-configured encoder;
  4. any video gap ended the whole mission (`end of video source` -> process exit), and `drone-watch` restarted it,
     which is what produced the 502 every few seconds.
* Fixed on the laptop (built and simulator-verified): `start()` clears `ended_`, the stale frame and the fps counter;
  a new `MissionLoop::restartVideo()` does `streamoff` -> (camera switch) -> 0.5 s -> `streamon` -> 0.5 s -> re-open,
  3 attempts; the camera toggle uses it; and a live stream that stops while the drone is on the ground is recovered
  the same way (up to 5 rounds) instead of ending the mission, so the page stays up. While airborne the old
  behaviour is kept: no video means land.
* **Pi: `git pull` and `cmake --build --preset linux-release` before the next power-on.** Then the test is: power on,
  wait for the page, press CAMERA front/belly twice with the drone on the ground, and check that the video comes back
  each time and `tello_race` does not exit. Report the log lines around each switch.
* Separate issue, not the cause: the drone sat powered on and idle and reached **85-88 C** with the battery down to
  44 %. Power it off between tests; it is also the way to clear a stuck video encoder.

### 2026-09-12 19:50 (laptop) first real power-on: automatic join and ground station verified end to end
* Drone powered on by the user at ~19:46:30. drone-watch journal: 19:46:51.7 `TELLO-FE18DB visible, joining`,
  19:46:52.3 joined (wlan0 192.168.10.2, default route stayed on eth0), 19:46:52.3 drone answers, 19:46:52.4
  ground station started, 19:46:56.5 `tello_race fly` running, port 8080 listening at 19:46:58. So about 7 s from
  SSID visible to page up; the "45 s" in the README is a safe upper bound.
* Live status on the Pi: `drone tello`, telemetry true, battery 77 %, video **24.0 fps** on the Pi, temp 44-46 C at
  power-on. From the laptop over the public Funnel URL (viewer token): page 12 KB in 0.05 s, status JSON, WebSocket
  hello role viewer with 25 state messages in 3 s, MJPEG frames flowing (real camera image confirmed).
* Idle drone on the ground: battery 77 -> 70 % and board temp 44 -> 76 C within 4 min. Power it off between tests.
* Nothing left on the laptop side for the manual flight. Before an autonomous run on the real track: README
  calibration checklist (calib-color, tag size, calib-ground, sign conventions), which needs the track.

### 2026-09-12 19:45 (pi) dry test of the start path without the drone: two bugs in ground_station.sh fixed
* After the reboot the laptop ran `boot_check.sh` and everything passed. Then `scripts/pi/ground_station.sh start`
  was run with drone-watch left running.
* Bug 1: `tmux new-window -t claude` targets window index 0 ("create window failed: index 0 in use"), so no `drone`
  window was created, yet the script printed "running". Fixed: `-t claude:` (next free index), a failed
  `new-window` now exits 1, and the script exits 1 when nothing listens on 8080 after 15 s.
* Bug 2: with no extra options, `printf ' %q' "$@"` passed an empty `''` argument -> tello_race `unknown option`
  and exit 1 before ever contacting the drone. This would also have broken drone-watch's start on race day. Fixed.
* tello_race output now also goes to `logs/ground_station.log` (header and exit code per run).
* Result after the fixes (19:41:37): window `claude:drone` created at +1 s, `Tello: entering SDK mode` attempts 1–4,
  each `timeout (3.0s) waiting for reply to 'command'`, then `ERR no answer from the Tello at 192.168.10.1` and
  `exit 1` at +13 s. The script reports "not listening on 8080" (rc 1). Funnel status unchanged (diff before/after),
  port 8080 free, drone-watch still active and waiting (drone_watch.sh unchanged, it calls the fixed script).
* Note: tello_race opens port 8080 only after the drone answers `command`, so the page gives 502 until then.
  That fits the race-day procedure.

### 2026-09-12 19:40 (pi) race day on power-on: drone-watch service, replay of real frames 28 fps, boot check script
**Race-day procedure (user):**
1. The Pi is powered and wired to the laptop (the laptop's internet sharing must be on). Nothing needs to be started on
   the Pi.
2. Power the drone on and wait about **45 s**. That is an estimate not yet measured with the drone: about 15 s for
   the drone to raise its Wi-Fi, up to 5 s until the next scan, about 10 s to join and get an answer from
   192.168.10.1, and a few seconds for tello_race to start.
3. Open **https://quiquepi.tail2b1864.ts.net/?token=OPERATOR** from any browser on any network. The operator token
   is `[web] token` in `config/default.ini` (never share it). Spectators use `?token=watch`. The header must read
   "connected". If the page shows 502, the ground station isn't up yet: wait and reload.
4. Fly (TAKE OFF / START need two clicks). After the run, power the drone off (it overheats sitting on the ground).
   About 20 s after the drone goes silent, the Pi stops tello_race and waits for the next power-on, so a battery
   swap is the same procedure.
* Watching it: `journalctl -t drone-watch -f` (every step is logged), `tmux attach -t claude` then window `drone`.
  Manual override: `systemctl --user stop drone-watch`, then `scripts/pi/drone_wifi.sh up` and
  `scripts/pi/ground_station.sh` as before.

**Implementation:**
* `scripts/pi/drone_watch.sh`, run by the user unit `drone-watch.service` (installed at
  `~/.config/systemd/user/`, enabled; linger is on for quiquepi, so it runs at boot without a login). Every 5 s, while
  `tello` isn't active: `nmcli dev wifi list --rescan yes ifname wlan0` (cached list if the rescan is refused). When
  TELLO-FE18DB is visible it checks `ipv4.never-default yes` (and sets it if missing), then runs
  `sudo -n nmcli --wait 30 con up tello` and waits up to 20 s for `ping 192.168.10.1`. Only when the drone answers
  does it stop any tello_race listening on 8080 (SIGINT, then SIGTERM; a non-tello_race holder is logged and left
  alone) and run `scripts/pi/ground_station.sh start` (tello_race fly in tmux `claude:drone`, Funnel ensured).
  - tello_race exits while the drone still answers: logged, restarted after 10 s.
  - drone unreachable for 20 s: ground station stopped, `nmcli con down tello` (wlan0 goes back to the home Wi-Fi),
    back to waiting.
  - tello profile no longer active: ground station stopped, back to waiting.
  `KillMode=process`, so restarting the watcher never kills tmux or tello_race. Not tested with the drone yet (the
  drone is off): verified that the service runs, logs `started: waiting for TELLO-FE18DB`, and detects the simulator
  on 8080 correctly.
* `scripts/pi/boot_check.sh`: checks eth0 = 192.168.137.10/24, default route on eth0, Tailscale Running and online,
  Funnel -> 127.0.0.1:8080, public `/api/status` HTTP code, drone-watch active/enabled, never-default. Before the
  reboot everything was OK (public HTTP 200 with the sim).

**Replay of the real camera path** (`fly --source file:... --headless --dry-run --set web.enabled=1`; `--dry-run`
added as a safety net, it doesn't touch perception, which runs on every frame even in IDLE):
* `/home/quiquepi/live_camera_60s.mp4` is really **6 fps** (ffprobe: 303 frames, 50.5 s, `r_frame_rate 6/1`), and
  file playback is paced at the file's rate, so the page showed `fps 5.9` (the source limit), with tello_race at 36 %
  of one core.
* To measure capacity: re-encoded to 30 fps with ffmpeg (duplicated frames, same pixels, 1515 frames) -> `fps` 28.1 /
  28.0 / 28.2 at 10/20/30 s, **tello_race 120–150 % CPU** (of 400 %), RES ~165 MB. So perception on real 960x720
  frames runs at >= 28 fps on the Pi, well above the 10 fps preflight limit; no lower camera resolution is needed.
* Output: no errors. The only warning is `DRY RUN: flight commands are logged, not sent`; `[null-drone] command:
  battery?` keep-alives every 8 s. Simulator restarted afterwards in `claude:sim` (page live).
* For the laptop: a live Tello stream should be ~30 fps; the 6 fps file is presumably the recorder's timestamps or a
  frame skip on the laptop side, worth checking.

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
