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

### 2026-10-12 (laptop) course mode deployed to the Pi over the cable; ready for the first real flight
* Pi at cf9fd6a+, rebuilt, `~/.config/drone-watch.env` now starts the station with
  `--profile config/course_tec.ini --log-level debug --set tello.wifi_ifname=wlan1 --set camera.flip=vertical`.
  **Gotcha:** `drone_watch.sh` reads GROUND_STATION_ARGS only when the service starts, so after editing the env
  file run `systemctl --user restart drone-watch` (done; the journal line "started: waiting for TELLO-FE18DB ...
  tello_race options: --profile ..." confirms what it will use). No `config/learned_course_tec.ini` yet: the
  first flight measures the mirror at tag 1.
* Vision measured on the Pi (headless sim): 4.2 ms per frame with the profile (tags 1.6, tape 2.5) against
  19.5 ms the old way. Real-time sim with the page open: 26 fps.
* Access checked end to end with a simulated station on the Pi's port 8080: page, state stream and video through
  the cable (`http://192.168.137.10:8080`), through the laptop's ssh tunnel (127.0.0.1:8082) and through the
  laptop's `tailscale serve` (`http://bulmacompu.tail2b1864.ts.net:8081`). The Pi itself has no internet on this
  network and its Tailscale is logged out, so the Funnel URL does not work today.
* Still nobody has seen the blue tape through the real camera. Before the first START: hold the drone about a
  metre over the tape and a tag and look at the page (thin white lines along the tape, tag outlined).

### 2026-10-11 (laptop) course mode: follow the tape from above with the ORIGINAL mirror clip; blue/yellow switch; self-calibration
* **What changed for the user.** He flies the original clip again (`Tello_Mirror_Clip_2.STL`, camera straight
  down, 96-99 deg) on a new course: white tarp, blue path, yellow decoy tapes, tags 1-2-3-4 where the tag turn
  meanings do NOT hold. New profile `config/course_tec.ini` (`--profile`, laid over default.ini) switches to a
  second navigator, `nav.mode = course` (`src/control/NavigatorCourse.cpp`, `src/vision/PathTracer.cpp`): the
  tape is traced top-down into metric centre-lines, the drone follows it holonomically at a fixed heading (no
  turns), flies over the tags in `course.tag_order`, takes the nearest other tape end as the next segment and
  lands on tag 4. Full description, first-flight checklist and numbers: **docs/COURSE_TEC.md**. The old
  line-ahead navigator is untouched (example course and seeds 300-319 give the same result to 0.1 s as before).
* **Page**: panel "Path to follow" with BLUE / YELLOW / BLACK (web command `linecolor`, refused while the mission
  flies), label and followed route drawn on the video, next tag and speed in the hint. State JSON has
  `lineColor`, `course`, `nextTag`, `speed`.
* **Simulation** on the photographed course (`courses/tec_tarp`, made by `tools/course_from_photo.py` from
  `courses/tec_tarp_photo.jpg`): first flight 20/20 in 29.8 s, later flights (everything learned) 20/20 in
  26.5 s, path error 2 cm mean / 6-8 cm at the worst moment, tags passed 2-3 cm from their centre. Altitude
  1.2 m (sweep in the doc). `config/course_tec_fast.ini` is a second profile for later: 23.1 s.
* **Self-calibration, new.** The mirror angle is fitted from where the tags are in space (floor-plane fit, one
  scale per tag), not from the tilt of a single tag any more (that read 3 deg high). On the first flight the
  drone moves a hand-span around tag 1 for ~5 s to get it; the same fit shows which way the telemetry counts
  pitch and roll (lean left out of the floor model until then) and the stick gain is measured from the tags.
  All of it is written to `config/learned_course_tec.ini` after landing (git-ignored, read at the next start).
  **Delete that file after the clip is re-seated.** It is also written when the drone is held by hand over a tag.
* **Ground-station load**: tags are searched in small windows (tracked tags + dark squares) and the whole frame
  only once a second; tag search every other frame; tape work at half width; 10 fps while nothing flies
  (`mission.idle_fps`). On the laptop the vision per frame went to about a fifth.
* **Needed from the Pi side** (the Pi was unreachable from the laptop on 2026-10-11: cable down, Tailscale
  offline, so nothing was deployed):
  1. `git pull` (or the bundle), `cmake --build --preset linux-release`.
  2. In `~/.config/drone-watch.env` add `--profile config/course_tec.ini` to `GROUND_STATION_ARGS` (keep the
     rest). Without it the station behaves exactly as before.
  3. Measure the vision on the Pi, no drone needed:
     `./build/tello_race sim --headless --no-realtime --profile config/course_tec.ini --track courses/tec_tarp --episodes 1 --seed 1 | grep "vision cost"`
     and once more with `--set tag.windows=0 --set tag.every_n_frames=1 --set line.proc_width=640` (the old way).
     Put both numbers here.
  4. Still valid: the Pi session never sends flight commands; the user starts the mission from the page.
* **Not verified on the real drone** (all handled in software, none seen in reality yet): telemetry pitch/roll
  signs, stick gain and response time (`course.response_tau_s = 0.5` is a careful guess), how the camera renders
  the white tarp and the blue tape (`line.white_auto` follows the exposure; the hue limits are from the photo),
  hover drift over a plain white tarp. `course.trace_hz = 5` writes a `CRS` line five times a second at debug
  level: after a real flight that log shows what the navigator saw and commanded.
* Other new keys: `tag.windows`, `tag.full_frame_every_s`, `ground.attitude_delay_s`, `ground.height_from_tags`,
  `ground.learned_file`, `ground.mount_verified`, `ground.imu_signs_verified`, `line.white_auto`,
  `tello.video_fps / video_bitrate / video_resolution` (SDK 3.0 video settings, empty = untouched, untested on the
  drone), and the `[course]` section. Simulator: `sim.lean_deg_per_ms2`, `sim.tof_scale`, `sim.focal_scale`,
  `sim.rc_deadband`, `sim.image_gain`, `sim.start_yaw_offset_deg`.

### 2026-09-22 (laptop) precise line following to the tag; live 3D simulation page; web robustness fix
* Navigator now follows the line right up to the tag with the mirror view (approach distance automatic =
  blind zone + 0.3 m, so ~0.35 m at 1.5 m; a forward camera keeps 1.4 m because it cannot see closer). The
  line-end direction is frozen once the tag is closer than 0.45 m (measured on the approach), the slow-down is
  measured before the approach point (`follow.tag_slow_dist_m = 0.5`), and a tag whose line end is in view is
  approached at once (also FORWARD tags, whose trigger window was a 10 cm slit from altitude). The simulator
  reports "path dev mean/max" per episode (distance to the nearest line point; ~0.4 m max is the floor from the
  80 cm gaps, which no drone can be closer to than 0.4 m). Final: 68 deg / 1.5 m seeds 300-319 19/20 (46.3 s)
  and 320-339 19/20 (46.5 s), typical deviation mean 0.06-0.09 m max 0.40-0.43 m; example course 43.3 s
  (was 47); stock 50 cm 10/10 (48.2 s). Before this work the drone left the line 1.4 m before every tag with
  deviation mean 0.12 max 0.52.
* Web: `/sim3d` shows a running simulation in 3D (floor texture, tags, Tello model, camera footprint, trail)
  with a panel to change camera angle / altitude / course / seed live (operator token). `web command sim`.
  Blocking go/turn commands are paced and streamed (no more teleporting in the views).
* **Ground station fix that matters for the real drone**: a websocket write to a browser that vanished
  (reload, phone asleep) blocked the control loop for up to 30 s. Socket write timeout now 2 s and a client
  whose write fails is dropped. Pi must rebuild.
* Simulator noise is now seeded per episode (batch and single runs agree exactly).

### 2026-09-21 (laptop) steep mirror views fixed; mirror holder settled (open underside, 36 deg, 13 up / 25 down)
* Bug: with the camera looking 70 deg or steeper the bottom of the image sees the floor under/behind the drone;
  the projector reported "no ground" (1e9) and the navigator kept its default 0.8 m blind zone, so it behaved as
  if the nearest 80 cm were invisible. Sim at 1.5 m went 0/10 at 70 deg. Fixed in CameraModel (blind zone 0 when
  the bottom ray lands at/behind the drone) and Navigator (clamp instead of ignore). Now 1.5 m: 67 deg 10/10
  (46.9 s), 70 deg 10/10, 74 deg 10/10; stock 50 cm 10/10. The measured mirror (68-69 deg) plus forward pitch in
  flight is inside that range.
* Mirror holder: the pane goes on the UNDERSIDE of the sloped plate (the face that looks at the lens). Final
  print: `Tello_Mirror_Clip_2_ext5_36deg_open.STL` (generator open_mirror_clip.py 5 36 25 13 28): arms +5 mm,
  36 deg plate 13 mm up-slope past the bar and 25 mm down, bar removed across 29 mm so nothing crosses the pane's
  path. Real frames: sharp, full coverage, tag decoded at the frame edge, calib pitch 68-69 flat. Also added
  `camera.blank_top_px` (paints image rows a short pane does not cover; 0 = off, not needed with this holder).
* First hover tests done 2026-09-20 with the user flying from the page: link -30..-50 dBm, 24 fps, battery sags
  ~1 %/3 s at hover, board 65-77 C (power off between flights). The climb to 1.2/1.5 m is still to do.

### 2026-09-20 19:30 (laptop) altitude navigator: 1.5 m with the 67 deg mirror view now flies random courses (19/20)
* Decision from the user: the belly camera is not used inside a run (front/belly switch takes 5-8 s), and the net is
  believed higher than 2.5 m, so the race plan is the mirror at ~67 deg with the drone at **1.5 m**
  (`mission.altitude_cm=150`, `camera.pitch_deg=67`; the sim needs `sim.cam_pitch_true_deg=67` too).
* Root cause of most altitude failures was an old bug in Mission.cpp: `setLineHint(nav.lineHint(hx,hy), hx, hy)`
  copied hx/hy before lineHint() wrote them (argument evaluation order), so the detector always got a (0,0) hint
  and locked onto the tail of the finished segment under the drone. Fixed. Also: the tag position after a tag
  maneuver is now dead-reckoned in the body frame (yaw + telemetry `vgx`/`vgy` + back steps, re-synced when the
  tag is in view) instead of "fixed behind the drone" (which also had the wrong sign after the back step); a
  short line running over the tag is rejected as the old tail; the tag just turned at cannot count as a
  "revisit" for 15 s. (Tried and reverted: taking the end-of-line direction from the component fallback when
  the tag is reached from the side; it cost 4 courses out of 20.) Nothing changed for the 50 cm configuration's outcomes: stock random 300-319 **20/20**
  (49.6 s mean), example 4/4 in 46.9 s.
* Results (random courses, seeds 300-319): **1.5 m / 67 deg: 19/20**, mean ~60 s (was 4/10 at the start of the
  day); 2.0 m / 67 deg: 5/10 (still fails: the turn at the tag is judged from a line seen 2 m away). Residual
  1.5 m failure (seed 319): two antiparallel segments 0.7 m apart both in the wide view; FOLLOW hops onto the
  neighbour. Example course at 1.5 m: 4/4 in ~55 s.
* **Two new calibration items for the real drone** (README step 5): `tello.vel_scale` (telemetry speed units to
  m/s, sim = cm/s -> 0.01) and `tello.vgy_sign` (which sign of `vgy` is LEFT). Read `v=(vgx,vgy,vgz)` in the HUD
  while pushing the stick slowly forward, then left, in MANUAL mode. Wrong values only degrade the acquisition
  after a tag (the anchor drifts), they cannot send the drone anywhere by themselves.
* Debug aids: `--set sim.trace_pose_s=2` logs the true pose and the segment geometry; `--log-level debug` shows
  the gate decisions (`gate: accept rule N` / `reject`) and the band-scan start (`bands:`).
* For the Pi session: pull, rebuild (`cmake --build --preset linux-release`), and set in GROUND_STATION_ARGS
  `--set mission.altitude_cm=150 --set camera.pitch_deg=67` (keep `camera.flip=vertical`). Report the `calib`
  pitch the mirror actually gives with a floor tag in view; if it is not 60-72 deg, tell me before any run.

### 2026-09-15 01:20 (pi) ce4fede on the drone via wlan1: takeoff false alarm gone, flip selector works; slow-loop warnings fire after blocking commands
* Joins through wlan1 since the first one: 00:01:16 -> 00:01:18 (2.1 s), 00:38:39 -> 00:38:41 (2.1 s), 01:11:47 ->
  01:11:51 (4.2 s). Default route always stayed on eth0. Strongest reading so far -24…-26 dBm (drone right next
  to the adapter, 00:39). After a landing the signal often drops to about -62 dBm on the floor, then recovers.
* Run 00:01:18: flight +15.1 -> land +45.5 (`ok` +50.3), in the air -46…-58 dBm, video ~150 KiB/s, `rcAge` <= 0.4 s.
* Run 01:11:51 (build 01:04:09 = ce4fede, options `--set tello.wifi_ifname=wlan1 --set camera.flip=vertical`,
  battery 68 %, board 51 °C): `camera.flip = vertical: live frames are flipped before perception (mirror mount)`.
  - Takeoff +17.73, land +72.77 -> notice. **No `no stick input` warning after takeoff: ce4fede fixes it.**
  - `mirror horizontal` +93.17 / `mirror vertical` +95.20 from the page, each with its notice.
  - **Page `wifi` vs `iw dev wlan1 link`:** 127 paired seconds, mean difference 0.3 dB (typical |diff| 3.7 dB from
    sampling at different moments). The readout follows the adapter.
  - **`slow loop` warnings:** `3.4 fps processed of 23.6 delivered; perception 18 ms/frame (tags 8, line 10),
    wait+perception 36 ms/frame` at +24.18, `2.2 of 24.4` at +77.39, `5.9 of 13.8` at +87.93, `1.9 of 43.9` at +102.42.
    +24.18 comes right after the 6.4 s blocking takeoff and +77.39 right after the 4.6 s blocking land; the others
    are around the landing and the mirror toggles. Status `fps` median 23.8 over the run, Pi load ~1.0, perception
    18–23 ms: these look like windows that include time blocked on a drone command, not slow perception. Exclude
    blocked time from the window, or skip the check right after takeoff/land/flip.
  - After the power-off (~01:14:10) `streamon -> 27 [not acknowledged]` still prints a stale reply (a battery
    value): the lastReply log bug from the 17:25 entry is still present.

### 2026-09-15 01:10 (laptop) mirror mount: flip = vertical, angle limit from the simulator, live selector
* The user mounted the mirror. From his screenshot the image is flipped **vertically only** (keyboard rows reversed
  top-to-bottom, letters in normal left-right order), so AprilTags cannot decode until un-mirrored. Set on the Pi:
  `--set camera.flip=vertical` appended to GROUND_STATION_ARGS in ~/.config/drone-watch.env (drone-watch restarted).
  The page now has a "mirror flip" selector (none/vertical/horizontal/both, command `mirror`) that applies live, for
  checking without restarts; the state JSON carries `mirror`.
* Camera angle matters more than the flip. Simulator with renderer and perception pitch aligned
  (`sim.cam_pitch_true_deg` = `camera.pitch_deg`), example course: 12 deg 4/4 in 46.8 s; **30 deg 4/4 in 40.5 s**;
  **45 deg: does not finish** (line lookahead lost: the view starts too close and the pursuit point is out of frame).
  So the mirror must leave the camera looking about 25-35 deg below the horizon, not straight down. The user's
  current view looks nearly vertical (keyboard seen from above) and would not fly the course; it is fine for the
  belly-style tag/colour view he wants, but not for the race. Auto-calibration now accepts pitch up to 89 deg and
  reports what the mirror actually gives (needs a floor tag in view, `calib n=` in the HUD).
* Other changes: the page dBm falls back to `iw dev <if> link` when the interface is not in /proc/net/wireless;
  a slow-loop diagnostic logs per-stage costs when the loop processes < 10 fps while the camera delivers more (the
  screenshot showed 4.8 fps processed at 24 delivered, suspect the backlit keyboard: hundreds of bright quads for
  the tag detector); take-off no longer trips the "no stick input" alarm (stick clock reset after the blocking
  take-off). Pi rebuilt at each step.
* For the Pi session: next power-on, confirm text reads correctly with the vertical flip, put a printed tag on the
  floor in view and report `calib n=` and the pitch it measures; if the slow-loop warning appears, report it.

### 2026-09-15 00:05 (pi) first power-on through wlan1 (Alfa): join OK, page dBm follows wlan1, three flights
* Run 2026-09-14 23:57:35 (battery 92 %): 23:57:31.5 `TELLO-FE18DB visible` on wlan1 -> **23:57:35.8 joined**
  (4.3 s; ~0.5 s on wlan0), 192.168.10.2/24, default route **stays on eth0** (wlan0 home route metric 600 as
  backup), drone answers, tello_race connected at once. dmesg: clean auth/assoc, no rtw88 errors.
* dBm source: once associated, `/proc/net/wireless` lists wlan1 and the page `wifi` field matches `iw dev wlan1
  link` (-46/-50 at the first sample). Link: channel 5 (2432 MHz), tx bitrate 54 Mbit/s.
* Flights (user, manual): +24.5 -> +106.0 and +113.9 -> +132.0 normal. `iw` signal on the ground -32…-52 dBm (36 s),
  in the air -32…-54 dBm (91 s); video under 20 KiB/s in 1 s out of 127. wlan0 on 09-12/13 indoors was -35…-57.
  Distances not known, so no fair comparison yet; the 1 m / 5 m / wall test is still to do.
* **Missed beacons on wlan1:** 118 at the first check -> 467 about 2.5 min later (~2/s). No disconnects, no video
  gaps, no driver errors. Watch it if the link ever drops.
* Third flight +151.6 -> at 00:00:22 `h 40, tof 298` (downward sensor suddenly ~3 m: over an edge/drop?) -> 00:00:24
  `h -80, tof 10`, no `land` sent -> +167.58 telemetry clear ("landed by itself"). Link -46…-58 dBm, video steady,
  no stick gaps: not a radio problem. Asked the user what happened. Battery 71 %, board 75 °C. Powered off 00:00:36.
* The takeoff false alarm (`no stick input ... for 9.3s` right after takeoff) is still there.

### 2026-09-14 23:50 (pi) drone link moved to the Alfa AWUS036ACS (wlan1); wlan0 stays on the home Wi-Fi
* Adapter: `0bda:0811` Realtek 8812AU/8821AU, driver `rtw88_8821au`, firmware 42.4.0, USB 2 (bus 003, 480M),
  MAC 00:c0:ca:b9:af:b4, `wlan1`. No driver errors in dmesg. A scan on wlan1 lists 8 networks (2.4 and 5 GHz).
* Regulatory: `iw reg get` global **MX** (2.4 GHz: 20 dBm); the on-board radio's phy#0 reports its own `country 99`.
  `iw dev wlan1 info`: **txpower 24.00 dBm** (the driver's figure; the regulatory limit on 2.4 GHz is 20 dBm).
* Changes:
  - `nmcli con modify tello connection.interface-name wlan1` (profile otherwise unchanged: open SSID,
    `ipv4.never-default yes`, `ipv6.method disabled`, `autoconnect no`).
  - `drone_watch.sh` and `drone_wifi.sh`: `IFACE=${DRONE_IFACE:-wlan0}`, so the radio comes from the environment.
  - `~/.config/drone-watch.env` on the Pi: `DRONE_IFACE=wlan1` and
    `GROUND_STATION_ARGS="--log-level debug --set tello.wifi_ifname=wlan1"`. drone-watch restarted:
    `started: waiting for TELLO-FE18DB on wlan1 ... tello_race options: --log-level debug --set tello.wifi_ifname=wlan1`.
  - Build 0a3feae (23:42:18) contains `tello.wifi_ifname`.
* To check on the first join: `/proc/net/wireless` currently lists only wlan0 (wlan1 not associated). If wlan1 does
  not appear there once joined, the page dBm readout (read from /proc/net/wireless) would be empty; `iw dev wlan1
  link` is logged per second as a second source.
* Fallback if the adapter misbehaves: `sudo nmcli con modify tello connection.interface-name wlan0` and
  `DRONE_IFACE=wlan0` plus `--set tello.wifi_ifname=wlan0` (or remove it) in `~/.config/drone-watch.env`, then
  `systemctl --user restart drone-watch`.
* Pending: power-on test (scan, join, default route on eth0, page up) and dBm at 1 m, 5 m and through a wall,
  against wlan0 on 2026-09-12/13 (-35…-57 dBm indoors).

### 2026-09-13 23:45 (pi) correction: takeoff refusals are NOT explained by heat; suspect one battery pack
* Run 23:39:50 (battery pack swapped, **100 %**): takeoff +11.93 accepted with the board at **72 °C**, no refusal
  in the whole run. Three flips, each followed by `notice: flip done` and spaced out (`f` +95.37, `r` +104.44,
  `l` +109.44), land +114.61. Battery 100 -> 68 %, board up to 81 °C.
* Takeoff outcomes today by board temperature at the attempt:
  refused at 52–57 °C (17:41 run, 100 %); refused at 66–90 °C (17:42, 23:33, 23:34, 23:38 runs; 84–92 %);
  accepted at 64–72 °C (17:50 run, 90 %; 23:39 run, 100 %).
  Neither temperature nor battery percentage separates the cases, so my "heat" reading in the 17:50 and 23:36
  entries is withdrawn. The change before this success was **a different battery pack**. Candidate: one pack the
  drone rejects at arming (a weak/unbalanced cell that sags when the motors spin up). Waiting for the user to say
  which pack was in during the refused runs.
* The preflight temperature suggestion (23:36 entry) is still harmless but won't fix this. A better preflight
  notice on `error`: "try another battery".
* Note: pressing EMERGENCY on the ground (run 23:38:34, +20.46) sends `emergency`; the Tello didn't reply within
  3 s, `NAV IDLE -> ABORTED (emergency)`; nothing else changed.

### 2026-09-13 23:36 (pi) first START (mission mode) on the real drone: takeoff refused, clean ABORTED; drone hot
* Run 23:33:13, drone power-cycled ~20 s after the previous flight (which ended at 71 °C). Battery 85 %.
* +41.94 `web command: start` -> `preflight OK: battery 84%, video 23.6 fps, height 10 cm` -> `NAV IDLE -> TAKEOFF
  (start)` -> +42.02 `takeoff` answered `error` -> **`NAV TAKEOFF -> ABORTED (takeoff failed)`**. Nothing else sent.
  The mission path handled it correctly.
* Manual retries +47.91 and +52.71: `error` again. Board **78–79 °C**, link -48…-55 dBm, video 21–23 fps.
  Consistent with the heat pattern (refusals at 66–90 °C in the 17:41–17:46 runs). Powered off ~23:34:30.
* For the laptop: the preflight could check `temph` (warn or refuse at >= 75 °C, "let it cool") so START
  explains itself before the drone refuses.

### 2026-09-13 23:35 (pi) run 23:31:12: land-refused fix verified on the drone; calibration log spam
* Battery 100 %, board 53 °C. On the ground before takeoff the link was weak (`wifi -61`, fps 9.3); in flight
  -35…-45 dBm, fps 24. Takeoff +20.84 -> `ok` +26.65 (the takeoff false alarm again, `... for 6.9s`).
* **Land-refused fix: works.** Low hover (h 10, tof 30–32); the drone touched down at ~23:32:35 (h 10 -> -30 -> 0,
  tof 10). The user pressed LAND at +82.19 -> `Tello << error` +82.21 -> `land refused while the telemetry says on
  the ground -> clearing the flying flag` -> `notice: land refused by the drone: error (it is on the ground)` ->
  `notice: drone is on the ground: TAKE OFF is available again` (+82.26). No stuck state.
* **Log spam:** `Ground auto-calibration: camera pitch=11.4 roll=… height(tag)=0.37m ratio(tag/tof)=1.17 (60 samples)`
  is logged at INFO on every frame while a floor tag is in view: 561 lines in ~40 s. Values stable (pitch 11.4°,
  roll 1.3–2.2°, tag height 0.36–0.38 m, ratio 1.17). Log only on a meaningful change or at most once per second.
* Powered off ~23:32:50 (recovery timeouts against the dead drone as usual).

### 2026-09-13 17:50 (pi) the drone refuses every takeoff (23/23 `error`) with a full battery; board up to 90 °C on the ground
* Run 17:26:23 (battery 68 %): normal. Takeoff +31.1 ok (takeoff false alarm `... for 20.1s` again), +67.6 `flip f` ->
  `notice: flip refused: the Tello needs more than about 50 % battery` (page-side check, nothing sent), land +115.6
  ok. 68 -> 32 % in ~1.5 min of flight.
* Run 17:41:11 (battery **100 %**): **5/5** `takeoff` answered `error` (+16.58 … +38.20) within 7–50 ms, on the
  ground (`h 0`, `tof 10`), link -43…-51 dBm, video normal, board 52–57 °C.
* Run 17:42:35 (battery 92 %) after a power cycle: **18/18** `takeoff` answered `error` (+8.5 … +175.9). Board
  66 °C at +20 s, 71 °C at +50 s, **85 °C at +180 s, max 90 °C**, all on the ground. Powered off ~17:46:30.
* Not battery level, not link, not software (the takeoff reaches the drone and is answered immediately). The SDK
  gives no reason. Candidates, not yet separated: the new mirror mount (touching a propeller, covering the
  downward vision/ToF sensors, or unbalancing it), an IMU needing calibration / not level, or heat (the first 5
  refusals were at only 52–57 °C, so heat alone doesn't explain those). Asked the user to cool it, remove the mount,
  try once on a flat surface, then use the official app for the reason / IMU calibration.
* For the laptop: when `takeoff` is answered `error`, the notice could add the likely checks (level surface,
  propellers/mount, temperature `temph`, "open the Tello app for the reason"). The operator pressed TAKE OFF 23
  times because the notice gives no hint what to do.
* Run 17:50:22 (battery 90 %, board 64 °C at start, ~4 min after the previous power-off): **takeoff accepted**
  (+19.09 -> `ok` +25.70). What the user changed (mount off / surface / cooling) is not known yet. Then three
  **`flip f` from the page, all `ok` -> `notice: flip done`** (+46.37, +60.96, +65.49; ~3.5 s blocking each), land
  +80.72 -> `ok` +85.81 (the telemetry clear fired at +85.69 with the misleading "(it landed by itself)" label).
  90 -> 71 % battery, board 73–78 °C. Powered off ~17:52:30.
* Run 17:54:35 (battery 100 %, board 68 °C at start): takeoff +38.85 -> `ok` +43.70 (takeoff false alarm `... for
  7.5s`). **Queued flip clicks are replayed back-to-back:** `flip f` +63.017 -> `ok` +66.612, then `flip b` sent at
  **+66.612** (same ms) -> `ok` +70.235, then another `flip b` sent at **+70.235** (same ms) -> `ok` +73.806. The
  clicks made while a 3.6 s flip was blocking were queued and executed one after another without pause: unplanned
  extra flips. Land +78.18 -> `ok` +81.01. Powered off ~17:56:07. Board 76–78 °C.
  **For the laptop:** drop flip/takeoff commands that arrive while a blocking command is running (or de-duplicate
  them), and ignore FLIP for ~1 s after a flip finishes; same class of problem as the late takeoff in the 22:10
  entry (item 3).

### 2026-09-13 17:25 (pi) 5548699 on the drone (12:23 session): heartbeat, 4 s capture timeout and ground recovery confirmed
Binary built 01:27:55 (after 5548699); 74b5970 (print files only) pulled, no rebuild needed. Data: per-second state
JSON samples plus the wlan0 byte rate, and the debug log. Run 2026-09-13 12:23:17, battery 100 % at connect.
* **Ground stream recovery: works.** On the ground the link was weak (-52…-60 dBm, wlan0 38–56 KiB/s, fps 1.7–7),
  then +46.06 OpenCV `Stream timeout triggered after 4009 ms` -> +47.58 `no frames for a long time` -> +47.70
  `re-requesting the stream (stream stopped, attempt 1/3, settle 3.0s)` -> `streamoff` ok, `streamon` ok ->
  **+54.54 `frames flowing (stream stopped, 3.0s settle)`**, wlan0 back to ~151 KiB/s. No power cycle.
* **Capture timeouts: honoured.** 4009 ms and 4043 ms instead of 30 s.
* **Stick heartbeat: works.** Manual flight +67.1 (`takeoff` ok at +75.84, 8.7 s blocking) to +153.2 (`land` ok,
  notice "on the ground"): `rcAge` 0.0–0.3 s and `stale false` throughout the flight. Link -38…-48 dBm, video
  137–184 KiB/s, fps 23–24.
* **False alarm right after takeoff:** `manual: no stick input or heartbeat from the page for 19.7s (page in
  background, or operator link?)` at +75.84, the instant the 8.7 s takeoff returned. The clock counts the time
  before the heartbeat starts (on the ground / during the blocking takeoff). Start it when airborne, or reset it on
  takeoff.
* **Battery:** 89 -> 61 % in about 90 s of flight (~0.3 %/s). Mirror mount weight or the worn pack? (User: which
  battery?)
* End: drone powered off at ~12:26:10 (wlan0 0, `wifi` 0), then `battery?` / `streamoff` / `streamon` timeouts from the
  ground recovery against the powered-off drone. drone-watch stopped the run.
* Run 17:19:42 (battery 84 %): `tof` 36–48 at times on the ground was the drone being **held/moved by hand**
  (perception `endtangent` debug lines with changing yaw for ~2.5 min in mission mode, `tof` back to 10 in between).
  Not a sensor or mount problem; ignore my first reading.
  - Board temperature 51 -> 73 °C in those 2.5 min powered on the ground.
  - Manual takeoff +150.0 (`ok` +154.36) -> the takeoff false alarm again: `no stick input or heartbeat from the
    page for 6.4s` at +154.38.
  - Five real warnings, rate-limited every 2 s: `... for 0.5s` +174.28, `0.6s` +176.29, `0.7s` +178.31, `0.7s`
    +180.39, `0.8s` +182.41. Short operator-side gaps (link -39…-49 dBm, video ~155 KiB/s throughout).
  - Land +231.42 `ok` -> notice. Battery 73 -> 44 % in ~80 s of flight (~0.36 %/s), fast again.
  - Drone powered off ~17:23:52: `battery?` timeout, then the ground recovery ran against a dead drone:
    `streamoff -> timeout`, `streamon -> 44 [not acknowledged]`. **Log bug:** after a timeout it prints
    `lastReply` from an earlier command ("44" was a battery value), not the streamon reply.

### 2026-09-13 00:05 (laptop) answers to the Pi's 23:10 report; heartbeat, real capture timeouts, hot-drone notice
* Thanks: flag clearing and notices confirmed on the drone; the worn battery explains the self-landings (user:
  retire that pack). Camera switch at 81 C failed with bytes arriving on attempt 1: the page now warns when
  `temph >= 75` before a switch ("let it cool"); the successful switches were all at 46-72 C.
* Correction to the "stick gaps" reading: `stale` was NOT evidence of link loss. The page only sent sticks while
  they were non-zero (plus one zero on release), so `rcAge` 20 s with `stale true` was simply the user not touching
  the pads. That also explains why `manual sticks released` never logged (it required non-zero sticks). Fixed: while
  airborne in manual mode the page heartbeats the current sticks (zeros included) 4x/s, so `stale` now means
  "no message from the page for 0.5 s", and the log warning fires on that (rate-limited 2 s).
* Likely outdoor mechanism on the operator side, now handled: phones lock the screen and browsers throttle hidden
  tabs to about one timer per second, which trips the 0.5 s dead-man. The page requests a screen wake-lock in manual
  mode, stops the sticks when it goes to the background, and reports "the page was in the background for N s" when
  it returns. Combined with the drone-link dBm this separates the three causes on the next outdoor flight.
* The open timeout: the environment variables were ignored by OpenCV (your measurement, 30.05 + 30.03 s). Replaced
  by the capture properties `CAP_PROP_OPEN_TIMEOUT_MSEC=6000` / `CAP_PROP_READ_TIMEOUT_MSEC=4000` passed to
  `VideoCapture::open`, which are honoured by OpenCV 4.10. Worst case for a failed switch is now ~40 s, not 2 min.
* Pi: pull and rebuild first thing (this was pushed after the Pi went off).

### 2026-09-12 23:40 (laptop) session close: mirror-mount hook, native flips, calibration range
* Everything was powered off for the night. State: race-day automation verified with the real drone, camera switch
  works both ways (belly -> front slow: measured 4.5-6 s settle), outdoor dead-sticks diagnosed (Pi antenna is the
  weak transmitter; keep-alive no longer blocks in flight; link dBm / input age / STICKS RELEASED on the page),
  stuck flying flag fixed, page notices for ignored actions.
* Tomorrow the user mounts a 3D-printed mirror so the FRONT camera looks down (colour view for the line and tags).
  Prepared: `camera.flip = none|horizontal|vertical|both` (config) flips live frames before perception, because a
  mirror produces a mirror image and AprilTags do not decode mirrored. The ground auto-calibration accepted pitch
  only up to 60 deg; raised to 85 so the ~60-80 deg mirror view calibrates. After mounting: choose the flip that
  makes printed text readable, run `calib-ground` (or let the auto-calibration from floor tags set the pitch),
  check the blind-zone figures, prefer a front-surface mirror.
* Acrobatics: the SDK `flip f|b|l|r` is wired to four FLIP buttons in the manual panel (two-click; manual mode,
  airborne, battery >= 55 % checks; the drone refuses below ~50 %). Needs ~2 m clear all round and good light for
  the vision positioning. Also available in SDK 3.0: `throwfly`; with the RMTT expansion kit (not mounted): LED
  matrix and RGB LED effects.
* **Pi: pull and rebuild first thing** (`git pull --ff-only && cmake --build --preset linux-release`); the Pi was
  off when this was pushed. The drone-watch service uses the built binary, so the new page needs that rebuild.

### 2026-09-12 23:10 (pi) 21a4e08 on the drone: flag clearing and notices work; a worn battery; stick gaps not logged; belly switch failed 3/3
Data: per-second samples of the state JSON (`wifi`, `rcAge`, `stale`, `h`, `tof`, `bat`, notices) plus the wlan0 byte
rate, and the tello_race log (debug). Byte counters only. The user flew in MANUAL, indoors/nearby (link never
weaker than -57 dBm), so this isn't the far outdoor case yet.
* **Header dBm:** the JSON carries `wifi` (-31…-57 dBm while joined, 0 once the drone is off). `/proc/net/wireless`
  lists **only wlan0**, which is the drone link while joined, so the value is right. The page header itself is for
  the user to confirm.
* **Flag clearing from telemetry: works.** Runs 22:57:17 (+65.49) and 22:59:13 (+39.99):
  `Tello: telemetry says on the ground for 1.5 s -> clearing the flying flag (it landed by itself)` ->
  `notice: drone is on the ground: TAKE OFF is available again` (noticeId 1). Normal land in run 23:01:00 (+297.4
  `land` ok) also produced that notice.
* **Notices for refused take-off: work.** Takeoffs right after those self-landings were sent and answered `error`
  (+69.15, +71.02 and +47.88, +49.22) -> `manual takeoff refused: error` -> `notice: take-off refused by the drone:
  error` (noticeId 2, 3). The mission-mode TAKE OFF notice wasn't pressed (the user was in MANUAL each time).
* **Worn battery (not software).** The battery used in runs 22:57 and 22:59 showed 53 -> 32 % in 60 s, then 44 %
  after a rest, then 44 -> 21 % in 55 s, still ~1 %/s on the ground, 15 % at power-off. The drone **landed by
  itself** at ~40 % and ~30 % (h 60 -> 0 in 2–4 s, no `land` sent) and then answered `error` to `takeoff`. Low-voltage
  protection. With a fresh battery (run 23:01:00, 100 %) the user flew 4.5 min, 100 -> 57 %, no self-landing, no
  refused take-off. **User: keep that battery out of use.**
* **Stick gaps from the page while airborne: many, and no log warning.** `stale true` samples: 279 in run
  23:01:00. Longest gaps while flying: `rcAge` 20.7 s (23:03:45–04:00, h 70), 11.5 s (23:04:15–26), 3.7–3.9 s
  (23:01:48, 23:02:23), 16–20 s in run 22:59 (22:59:39–43) and 21 s in run 22:57 (22:58:13–22). The drone link was
  fine during all of them (-35…-52 dBm, video ~150 KiB/s). So these are **operator-link / page-side gaps**, which
  matches "the drone stops answering the sticks". **`manual sticks released` never appeared in the log**
  (0 lines over all three runs), although `stale` was true for up to 20 s: the log warning doesn't fire.
* **battery? timeouts:** only when the drone was being powered off (+92.6/+100.6 in run 22:57, +75.3/+83.3 in run
  22:59; wlan0 0 KiB/s, `wifi 0`). None while the drone was up. The in-flight query fix can't be judged from these
  flights because there were no marginal-link moments.
* **Min wifi:** -56 dBm (run 22:57), -46 (22:59), -57 (23:01).
* **Belly switch after the flight (run 23:01:00, on the ground, drone at 81 °C): failed all 3 attempts.**
  +341.72 `camera` -> attempt 1 (3.0 s) `streamon` ok +344.86 -> wlan0 **~100 KiB/s for 5 s (video arrived)** ->
  `no frame within 4 s`. Attempt 2 (4.5 s) `streamon` ok +354.14 -> **wlan0 1–2 KiB/s (silent)** -> open blocked
  30.05 s + 30.03 s -> `re-open failed` +414.22. Attempt 3 (6.0 s) `streamon` ok +420.79 -> silent -> 30.03 s +
  30.03 s -> +481.35 `ERR the drone did not resume the stream after 3 attempts (belly camera)`. The loop was
  blocked ~2 min in total. The open timeout is still OpenCV's 30 s (see 22:10 entry, item 2). Temperature may
  matter: the successful switches were at 46–72 °C, this one at 81 °C.
* For the laptop: (1) the `manual sticks released` log line doesn't fire; (2) look at why the page stops sending
  sticks for 10–20 s while connected (tab throttling in background? the laptop/phone's own link?), possibly log
  WebSocket disconnects/reconnects with timestamps; (3) open timeout (`CAP_PROP_OPEN_TIMEOUT_MSEC`) is still the
  biggest risk during a camera switch.

### 2026-09-12 23:05 (laptop) stuck `flying` flag fixed, page notices for ignored actions
* Implements the Pi's 22:18 analysis. `TelloDrone`: the flag is cleared when telemetry has said on-the-ground
  (`h <= 0 && tof <= 10`) for 1.5 s while the flag was set, and when `land` is answered `error` while telemetry says
  grounded (the Tello refuses `land` when it is already down). Both cases are logged. Until now only a power cycle
  cleared it, so after a self-landing (15 s without commands on a lost link, low battery, a bump) TAKE OFF was
  silently never sent, CAMERA was refused and LAND answered `error`.
* New one-line notice channel to the page (`noticeId`/`notice` in the state JSON, shown once per id as the yellow
  message): take-off ignored because the drone reports flying, take-off in mission mode, take-off/land refused by the
  drone with its reply, camera switch refused while flying, "drone is on the ground: TAKE OFF is available again".
  Verified in the simulator.
* Outdoor picture now consistent: the Pi 5 antenna is the weak transmitter, so far from the Pi the drone's video
  still arrives while the Pi's 20 Hz rc packets are lost -> the Tello hovers ("does not respond"), a retried `land`
  eventually gets through, and after 15 s without any command the Tello lands by itself -> the flag stuck. Hardware
  remedy if outdoor range matters: a USB Wi-Fi adapter with an external antenna on the Pi dedicated to the drone
  (the `tello` profile then binds to that interface), or simply keep the Pi close to the flying area.

### 2026-09-12 22:40 (laptop) outdoor flying: range, dead sticks, speed slider
* User report: camera switching now works both ways (belly -> front is slow: that is the measured 4.5-6 s settle).
  Outdoors the drone stops answering the sticks but still lands and still streams video; the speed slider "does
  nothing".
* Slider: verified in the real page with headless Chrome: slider 100 + W = [0,100,0,0], slider 25 + pad = [25,13,0,0].
  It works; the Tello's rc response is strongly non-linear (below ~30 it barely moves) and wind dominates outdoors.
* Log of the outdoor runs (22:13-22:18): no page reconnects, `battery?` replies prompt during the 22:18 flight but
  three 3 s timeouts right after landing: the drone link was marginal outdoors while video (fire-and-forget from the
  drone) kept arriving. `land` is a retried command, so it still got through.
* Design flaw fixed: the ground-station keep-alive sent a blocking `battery?` every 8 s whenever the *navigator*
  was idle, i.e. also during manual flight. A blocking command pauses the rc stream until the reply or the 3 s
  timeout, so on a marginal link the sticks are dead 3 s out of every 8. Now no query is sent while the drone is
  airborne in any mode (the rc stream keeps the SDK alive in flight).
* Instrumentation added so the next outdoor flight is diagnosable: state JSON has `wifi` (drone link dBm from
  /proc/net/wireless, wlan0), `rcAge` (s since the page last sent sticks) and `stale` (dead-man released the
  sticks); the page header shows "link -49 dBm / input 0.1s" and turns red with "STICKS RELEASED" plus a message
  when the operator link drops; the log gets `manual sticks released: no input from the page for X s`.
* Range: the drone is the access point. The Pi's on-board antenna is the weak end; through walls tens of metres at
  best, open air 50-100 m. Keep the Pi (and the laptop it is wired to) near a window or take them outside on a
  power bank; the operator's phone link to the Funnel matters too (0.5 s without sticks = hover).
* Next outdoor flight: watch the header. Red "STICKS RELEASED" = your phone/laptop link; dBm falling below about
  -75 with video artefacts = drone link; neither, and the drone just drifts = wind (raise the slider to 80-100).

### 2026-09-12 22:18 (pi) stuck `flying` flag: the drone landed by itself, TAKE OFF ignored, LAND answered `error`
Run 22:13:15 (battery 100 %, b93d21c, debug):
* +16.58 `takeoff` -> `ok` (+23.83). No `land` was sent after that. By +68 s the drone was on the ground (status
  `flying true`, **`h 0`, `tof 10`**, bat 75 %), apparently after landing by itself.
* +68.11, +69.58 and +95.71: the user pressed TAKE OFF. `web command: takeoff` was logged, but **no `Tello >> takeoff`
  was sent**, because tello_race still believed it was flying. CAMERA would also have been refused ("while flying").
* +84.42: LAND from the page -> `Tello << error` -> `WRN command 'land' answered 'error'`. `TelloDrone::command()`
  clears `flying_` only on `ok` (and on `emergency`), so the flag stayed true.
* Cleared only by a drone power cycle (22:14:34 unreachable -> drone-watch stopped at 22:15:18 -> new run 22:15:29
  `flying false`).
* For the laptop:
  1. When telemetry says the drone is on the ground (e.g. `h == 0` and `tof <= 10` for ~1 s while `flying_`),
     clear `flying_` (and the navigator's flying state) and log it.
  2. `land` answered `error` while `flying_` is true should also clear it, or trigger that telemetry check. The Tello
     answers `error` to `land` when it is already on the ground.
  3. TAKE OFF ignored because "already flying" should give the page a visible message instead of doing nothing.

### 2026-09-12 22:10 (pi) b93d21c on the drone: both directions recovered, each on attempt 2 (4.5 s settle)
Run 22:05:09 (battery 89 %, `--log-level debug`), drone on the ground for both switches; wlan0 bytes/s from
`/sys/class/net/wlan0/statistics/rx_bytes` (video ~150 KiB/s, telemetry 1–2 KiB/s). Before the test the user flew
three short hops; the drone was power-cycled at 22:04:40 and drone-watch rejoined and restarted in 1 s (22:05:08.6
SSID -> 22:05:09.3 connected).
* **front -> belly (+80.88, one click).** `streamoff` ok, `downvision 1` ok, `streamoff` ok, 3.0 s settle,
  `streamon` ok (+84.11) -> **wlan0 86–108 KiB/s (the drone sent belly video)** -> `WRN no frame within 4 s of
  streamon (settle 3.0s)` (+88.81). Attempt 2: `streamoff` ok (1–2 KiB/s), 4.5 s settle, `streamon` ok (+93.45) ->
  **`stream re-opened, frames flowing (belly camera, 4.5s settle)` at +94.00**, 0.55 s after streamon. Belly fps
  15.8–16.3; the status now shows `camera belly`. As in run 21:46:52, the failed belly attempt had video arriving
  and nothing decoded.
* **belly -> front (+124.61, one click, 6 s after landing).** `streamoff` ok, `downvision 0` ok, `streamoff` ok, 3.0 s
  settle, `streamon` ok (+127.79) -> **wlan0 1–2 KiB/s for 67 s (22:07:15–22:08:21): the drone sent no front video**.
  The open blocked **30028 ms** (+157.8, `Low-latency open failed`), then the FFmpeg-defaults open another
  **30027 ms** (+187.85, `re-open failed`). The 4 s check never ran because the open itself doesn't return, and
  the open timeout is still not taking effect. Attempt 2: `streamoff` ok, 4.5 s settle, `streamon` ok (+192.92) ->
  **`stream re-opened, frames flowing (front camera, 4.5s settle)` at +193.85** (wlan0 42 -> 151–160 KiB/s from
  22:08:22). Front fps 23.0–24.3 afterwards. Attempt 3 (6 s) wasn't needed. tello_race never exited.
* Observations for the laptop:
  1. **front: 3.0 s is too short, 4.5 s worked**, and the drone is silent until the good `streamon`. **belly: 3.0 s
     had video arriving but no decoded frame within 4 s, 4.5 s worked.** Both directions succeeded on the first
     4.5 s attempt in this run. One run is not proof; a 4.5 s first settle looks like the candidate.
  2. **The open timeout is still 30 s + 30 s** (`Stream timeout triggered after 30028 ms` twice). With a silent drone,
     a 3.0 s first attempt costs 60 s of blocked loop. Either `OPENCV_FFMPEG_OPEN_TIMEOUT` isn't read by this OpenCV
     (4.10.0, Debian) or it's set after the backend has read it. On 4.10 the open timeout can be passed as
     `CAP_PROP_OPEN_TIMEOUT_MSEC` in the `VideoCapture::open(url, CAP_FFMPEG, {params})` call.
  3. **Clicks made during the blocked loop run late.** A `takeoff` clicked while the loop was stuck executed at
     +193.85, in the same instant frames came back (the log shows `frames flowing` and `web command: takeoff`
     with the same timestamp). Then another `takeoff` +202.56 and `land` +202.71. Commands queued during a stall
     should be dropped (at least takeoff), not replayed late.
  4. Status `fps` went `0.0` once right after the switch (good), but stayed at the old belly value during the
     60 s stall.

### 2026-09-12 22:00 (pi) run 21:46:52 (aae4bd9) measured on the wire: belly failures had video arriving, front had none
Correction to the 21:58 laptop entry: the counters did **not** show "no datagrams" during every failure. The two
directions fail differently:
* **front -> belly (+60.9 s, on the ground).** Replies: `streamoff` ok, `downvision 1` ok, `streamoff` ok, then
  `streamon` ok after a 1.0 s settle (+62.09), `no frame yet` -> 2nd `streamon` ok (+65.16) -> `still no frames`
  (+70.2). Attempt 2: 2.0 s settle, `streamon` ok (+72.34) -> 2nd `streamon` ok (+75.86) -> no frames.
  **Measured during attempt 2 (21:48:07–14): 744 UDP datagrams of 1460 bytes from 192.168.10.1 to 192.168.10.2:11111,
  wlan0 143–161 KiB/s, no ICMP for 11111.** So the drone *was* streaming and tello_race decoded nothing. The rate
  fell to 1 KiB/s at 21:48:15, right after attempt 3's `streamoff` (+80.96). Attempt 3: 3.0 s settle, `streamon` ok
  (+83.97) -> **`stream re-opened, frames flowing (belly camera)` at +84.95.**
  Reading: for belly the 1.0/2.0 s attempts didn't fail because the drone ignored `streamon`. The stream arrived
  but no frame came out of the decoder within the 3 s + 5 s windows. The extra `streamoff` / `streamon`
  cycles may have kept restarting the encoder before a decodable keyframe arrived. The 3.0 s attempt is also the
  first one with no 2nd `streamon` inside the window (frames came 1 s after the first). 3 s is not proven to be what
  fixed it.
* **belly -> front (queued click, +84.95).** `streamoff`, `downvision 0`, `streamoff` ok, 1.0 s settle, `streamon` ok
  (+86.16), open. **Measured 21:48:38–45: 0 datagrams to 11111, wlan0 1–2 KiB/s** (telemetry) with the socket
  bound. The open still blocked `30027 ms` (+116.2), so `OPENCV_FFMPEG_OPEN_TIMEOUT=6000` didn't bound it. The front
  failure is a silent drone.
* Side finding: all the time, including while belly frames flowed, the drone sends a datagram per second to
  **UDP 8899** on the Pi. Nothing listens there, so the Pi answers `ICMP port 8899 unreachable`. It doesn't stop
  the belly stream. The laptop may know what 8899 is (the TT expansion module?).
* Status JSON stayed at `fps 24.3, camera front` through both switches: it follows neither the camera change nor
  the loss of frames.
* After this run: drone-watch and the ground station were stopped for an isolation experiment, which the laptop
  called off before it ran (the drone was never power-cycled for it). From now on only the wlan0 byte counter is
  used (per-second rx logger, no packet capture). drone-watch is running again with `--log-level debug`, build
  b93d21c (21:56:10). Next: front -> belly, belly image + 10 s, belly -> front.

### 2026-09-12 21:58 (laptop) measured: the drone honours streamon only ~3 s after the camera change
* Run 21:46:52 (battery 39 %), belly switch: settle 1.0 s -> `streamon` ok, no frames; 2.0 s -> ok, no frames;
  **3.0 s -> ok, `frames flowing` 1 s later**. So the drone answers `ok` while it is still switching and ignores
  the request; ~3 s after `streamoff` + `downvision` it streams. The Pi's byte counters showed no datagrams at all
  during the failures, so the earlier ICMP port-unreachable theory (21:35 entry) was wrong. Retracted.
* Then a second CAMERA click, queued by the user during the 24 s of retries, was applied 50 ms after the belly
  stream came up and switched straight back to the front with a 1.0 s settle, which failed again. From the page it
  looked as if nothing had changed.
* Shipped (this commit): settle before `streamon` is now 3.0 / 4.5 / 6.0 s per attempt, socket bound right after
  `streamon`, `waitFirstFrame(4 s)` proves a decoded frame; the second `streamon` nudge never helped and is gone
  (fewer commands hammering the drone); CAMERA clicks arriving within 3 s of a finished switch are dropped.
* Expected now: a switch takes ~4-5 s and the page freezes for that time (the restart runs on the mission loop);
  acceptable on the ground. The belly camera is not used by the mission or by manual flight, so this is a
  convenience feature, not a competition blocker.
* Pi test protocol: drone on the ground, `--log-level debug`; front -> belly, wait until the page shows the belly
  image, wait 10 s more, belly -> front. Report the `frames flowing (... settle)` lines. To check whether video is
  arriving use the interface counter only (`/sys/class/net/wlan0/statistics/rx_bytes` per second: ~300+ KiB/s with
  video, ~2 KiB/s telemetry only). No packet-capture tools.
* Housekeeping: the drone was at 39 % and was wedged (Wi-Fi up, DHCP/SDK/video dead) after abrupt power-offs;
  charge it and power it off between tests.

### 2026-09-12 21:35 (laptop) regression found: the delay after streamon is what makes the drone go mute
* Test with 3f74dc2 (user, drone on the ground, `--log-level debug`): the **belly switch now fails too**, all three
  attempts. That is a regression: with 01516db the same switch worked in 93 ms. Log: `downvision 1` ok, `streamoff`
  ok, `streamon` ok, open, then no frame in 3 s, second `streamon`, no frame in 5 s, three times over.
* The only behavioural difference between the working and failing versions is the pause **after** `streamon` and
  before the receiving socket is bound: 0.5 s when it worked, 1.5 s (then 3.0 s, 4.5 s) when it fails.
* Mechanism that fits every observation: the Tello starts sending video the instant it answers `ok` to `streamon`.
  Nothing is bound on UDP 11111 yet, so the Pi's kernel answers ICMP port-unreachable and the drone stops sending
  altogether. That explains why it acks every command and stays mute, why longer settles made it strictly worse,
  and why a power cycle is the only thing that clears it.
* Fix shipped: the settle now happens **before** `streamon` only (1.0 / 2.0 / 3.0 s per attempt, which is what the
  encoder needs to change camera), and the socket is bound immediately after `streamon` with no pause at all.
* Second defect fixed: one open blocked 30 s and the FFmpeg-defaults fallback another 30 s, 60 s in total, because
  OpenCV's **open** timeout is separate from its read timeout. `OPENCV_FFMPEG_OPEN_TIMEOUT=6000` is now set too.
  (The opens that returned fast and then produced nothing had almost certainly probed leftover buffered packets.)
* Unrelated and working correctly: at ~21:32 the drone stopped answering ping entirely. drone-watch stopped the
  ground station after 20 s unreachable, left the network and rejoined when the SSID came back, exactly as designed.
* Next test, drone on the ground: switch front -> belly, then belly -> front. Expect "frames flowing" within a
  second or two. If it still goes mute with the socket bound immediately, the ICMP explanation is wrong and the next
  step is the isolation experiment: front camera only, no `downvision`, `streamoff` -> wait -> `streamon`.

### 2026-09-12 20:40 (laptop) stronger restart after the Pi's evidence: longer settle + a second streamon
* The Pi's 20:22 entry settles the mechanism: the drone answers `ok` to `streamoff`, `downvision 0` and `streamon`
  and then sends **nothing** (wlan0 at 1-2 KiB/s, telemetry only). This is drone-side, not a decoder problem: the
  Tello acknowledges while its 960x720 encoder is still coming up and never starts sending.
* Shipped in 4f2b486 + 3f74dc2: `restartVideo()` no longer trusts `cap_.open()`, it waits for a decoded frame via the
  new `IFrameSource::waitFirstFrame()`; `OPENCV_FFMPEG_READ_TIMEOUT=4000` so a silent stream fails in ~4 s instead of
  blocking the mission loop for 30 s; the settle between `streamoff`, `downvision`, `streamon` and the re-open is
  1.5 / 3.0 / 4.5 s per attempt; if no frame arrives within 3 s, `streamon` is sent **again** while the receiving
  socket is bound and the code waits 5 s more; and both replies are logged, including "not acknowledged".
* On the Pi's safety point (a blocking recovery could leave the drone without a land): both callers are guarded. The
  camera toggle refuses with "not switching cameras while flying" and the recovery path requires
  `!nav_.flying() && !drone_.isFlying()`, so `restartVideo()` cannot run while airborne.
* Next test, drone on the ground, run with `--log-level debug`: front -> belly, then belly -> front. Report the
  `video: streamoff -> ...` / `video: streamon -> ...` lines, whether "frames flowing" or "still no frames after two
  streamon requests" appears, and the fps after each switch. **If it is still mute, change no code**: instead run the
  discriminating experiment, front camera only and no `downvision` at all, `streamoff` -> wait 5 s -> `streamon`, and
  report whether the front stream comes back. That separates "the front stream cannot be restarted at all" from
  "only the camera switch breaks it".

### 2026-09-12 20:30 (laptop) camera switch: belly works, front does not; first fix was incomplete
* User report: switching to the belly camera works, switching back to the front camera kills the video. The log
  confirms the asymmetry and shows the first fix (01516db) reported success too early:
  `139.448 video: front camera` (downvision 0 accepted) -> `141.013 video: stream re-opened` -> `153.311 ERR video:
  no frames for a long time` (12 s, **zero** frames ever decoded) -> re-open -> `Stream timeout triggered after
  30027 ms`, which blocked the mission loop and froze the page for 30 s.
  The belly direction opened in 93 ms and streamed fine, so the commands are accepted either way; only the 960x720
  front encoder fails to resume.
* Three defects in the first fix, all addressed now:
  1. `restartVideo()` returned true when `cap_.open()` succeeded, without a single decoded frame. It now calls the
     new `IFrameSource::waitFirstFrame(4 s)` and only reports success when a frame actually arrives.
  2. A silent stream blocked `open()`/`read()` for OpenCV's default 30 s. `OPENCV_FFMPEG_READ_TIMEOUT=4000` (ms) is
     now set for live streams, so a dead stream fails in ~4 s.
  3. The drone's answers to `streamoff` / `streamon` were never logged, so there was no way to tell whether the
     Tello acknowledged the restart. Both replies are logged now, including "not acknowledged".
* Also: the settle time between `streamoff`, `downvision`, `streamon` and the re-open now backs off per attempt
  (0.8 s, 1.6 s, 2.4 s). Hypothesis to confirm: the front encoder needs longer to re-initialise than the belly one
  and ignores a `streamon` that arrives too early.
* **Pi: `git pull` and rebuild before the next power-on.** Test with the drone ON THE GROUND: switch to belly, then
  back to front, and report the new `video: streamoff -> ...` / `video: streamon -> ...` reply lines plus whether
  "frames flowing" appears. If the front camera still never delivers a frame, the replies will tell us whether the
  drone refused the command or accepted it and stayed silent; do not change anything before reporting that.
* Known remaining limitation: `restartVideo()` runs on the mission loop, so the page still stalls for a few seconds
  during a switch. Bounded now to roughly 4 s per attempt instead of 30 s.

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
