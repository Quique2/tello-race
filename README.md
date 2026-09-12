# tello_race - autonomous DJI Tello line + AprilTag racer (MR3003C competition)

C++17 flight software for the *DJI Tello Autonomous Drone Competition* (Tecnológico de Monterrey, MR3003C):
take off on the start marker, follow the dark line on the white strip, read the AprilTag at the end of each
segment (ID 1 forward, 2 left, 3 right, 4 finish), find the next segment 80 cm away, and land on the finish tag,
as fast as reliability allows.

* Real drone, video replay and a physics/vision **simulator** share the same navigator (`tello_race sim`).
* Metric perception: pixels are projected onto the floor using the ToF height and the camera tilt, which is
  **self-calibrated from the floor tags** while flying.
* Safety first: rc watchdog, video watchdog, bounded search patterns, battery gate, checkpoint resume.
* Read [docs/STRATEGY.md](docs/STRATEGY.md) for the rules analysis, architecture and state machine.

## Repository layout

```
CMakeLists.txt, CMakePresets.json   build (MinGW GCC preset for the portable toolchain in C:\dev)
config/default.ini                  every tunable, with comments (copied next to the executable at build time)
src/tello      UDP SDK client (commands, telemetry, rc keep-alive/watchdog), NullDrone for replays
src/vision     FrameSource (low-latency FFmpeg), GroundProjector, LineDetector, TagDetector, Perception
src/control    Navigator state machine + controllers
src/sim        Track generator/renderer and the Simulator (drone kinematics + camera rendering)
src/app        Mission loop (HUD, keys, recording), calibration tools
scripts/       build.ps1, run_sim.ps1, fly.ps1, firewall_allow.ps1, setup_toolchain.ps1
docs/          STRATEGY.md
```

## Build (Windows, no admin rights needed)

The project was developed with a portable toolchain in `C:\dev` (winlibs GCC 16, CMake, Ninja, OpenCV 4.14
built with the FFmpeg wrapper). `scripts\setup_toolchain.ps1` recreates it on another PC; then:

```powershell
powershell -ExecutionPolicy Bypass -File scripts\build.ps1
# -> C:\dev\build\tello-race\tello_race.exe   (config/ and all DLLs are copied next to it)
```

Any other toolchain works too: the only dependency is OpenCV (core, imgproc, imgcodecs, videoio with FFmpeg,
highgui, calib3d); the AprilTag library is fetched and built from source by CMake. With Visual Studio:
`cmake --preset msvc-release` after setting `OpenCV_DIR` in `CMakePresets.json`.

## Run

```powershell
# simulator (window + top view). Keys work the same as with the real drone.
C:\dev\build\tello-race\tello_race.exe sim --track example
C:\dev\build\tello-race\tello_race.exe sim --track random --episodes 20 --seed 1 --headless --no-realtime

# real drone: connect the PC to the TELLO-xxxxxx Wi-Fi first
C:\dev\build\tello-race\tello_race.exe telemetry --duration 10      # checks telemetry + video (firewall!)
C:\dev\build\tello-race\tello_race.exe fly --record                 # 't' = take off + start, 'l' = land

# work on recorded videos without the drone
C:\dev\build\tello-race\tello_race.exe replay --source file:logs\run_20260911_120000.avi
```

Keys in the mission window: `t` start, `l`/space land, `e` EMERGENCY motor stop, `r` record, `1-3` resume
from that checkpoint tag on the next start (`0` = start line), `m` mask, `d` debug overlay, `q` quit.

Useful options: `--set section.key=value` (any config value, repeatable), `--dry-run` (video + telemetry, no
flight commands), `--start-at-tag N`, `--dump-every N` (save annotated frames to `logs/frames`).

## First contact with the real drone (verified 2026-09-12)

Tello Talent `TELLO-FE18DB`, no expansion kit mounted, direct Wi-Fi mode: `tello_race telemetry` entered SDK mode on
the first attempt (`sdk=30`), telemetry arrived at 10 Hz and the 960x720 video decoded at ~24 fps (the H.264 warnings
printed during the first second are the decoder waiting for the first keyframe and are harmless). Windows created
inbound allow rules for the program on the Public profile. `scripts\wifi_tello.ps1` joins the drone's network and
`scripts\wifi_tello.ps1 -Back` returns to the home network. Notes from the manual: the aircraft's Micro USB port
is charge-only (1.5 A adapter, aircraft switched off), and a yellow blinking LED means "no client connected" /
"no vision positioning", not an error; it turns purple once the SDK is connected.

## Before the competition: calibration checklist (practice track)

1. **Connectivity**: `tello_race telemetry --duration 10`. Both "telemetry: OK" and a video fps > 20 are required.
   If either is missing, allow the program through Windows Firewall (`scripts\firewall_allow.ps1` from an
   elevated PowerShell, or click "Allow" in the firewall pop-up) and make sure the PC is on the drone's Wi-Fi.
2. **Line colour**: `tello_race calib-color` (drone on the floor or held over the track). Click the line in several
   places (near and far), press `w` and click the white strip, then `s` to save. Black tape gives `color_mode = dark`.
3. **Tag family and size**: check the printed tags with `tello_race calib-ground` (the detector must show `ID n`).
   Measure the black square edge with a ruler and set `tag.size_m`; the tool also prints the size implied by the
   ToF height (press `z` to save it).
4. **Camera tilt**: in the same tool, hover the drone 1-2 m in front of a floor tag and press `s` when
   `samples` > 20; this stores `camera.pitch_deg`/`camera.roll_deg`. It is also refined automatically in flight.
5. **SDK sign conventions** (once per drone): after a `fly --dry-run` check that telemetry `pitch` is positive
   nose-up while you tilt the drone by hand (`ground.imu_pitch_sign`), and verify with a manual `go 0 50 0 30`
   from the Tello app or a script that +y moves LEFT (`tello.go_y_left`).
6. **Speeds**: fly the practice track with the defaults, then raise `follow.v_max` until runs start to fail,
   and back off one step. Record everything (`--record`) and replay to inspect.

## Competition day

* Battery > 60 % on every attempt; the software refuses to take off under 25 %.
* Judge places the drone on the start marker facing the line, you press `t`.
* If an attempt is stopped, place the drone on the last tag reached and press the tag number (`1`, `2` or `3`)
  then `t`: the drone takes off, performs that tag's turn and continues (20 s penalty, but the segments count).
* `l` or space lands at any time; `e` cuts the motors (only if a crash is imminent).

## What the simulation looks like

![Eight moments of a simulated run of the example course](docs/media/sim_example_course_sheet.jpg)

Left half of each tile: the simulated Tello camera with the perception overlay (line chain in yellow/magenta, tag
box, pursuit point, HUD with phase, telemetry, timers and the last state transitions; the small black panel is the
line mask). Right half: top view of the track with the drone, its camera footprint (blue) and its trail (yellow).
`--dump-every N` writes these annotated frames for any run; `--record` stores the raw camera video and a CSV of
telemetry and commands, which `tello_race replay` can process again offline.

## Simulation results

`tello_race sim --track random --episodes N --headless --no-realtime` prints one line per course and a summary.
The simulator renders the camera view from a textured track with tags, lines of other colours, video latency,
sensor noise and imperfect `go`/turn execution, so it exercises exactly the same navigator code that flies the
real drone. Latest validation with the default configuration (see docs/STRATEGY.md section 7 for the history):

| Run | Result |
|---|---|
| Example course from the slides | 4/4 segments, 46.8 s |
| 30 random rule-compliant courses (seeds 300-329) | 29/30 completed, mean 49.8 s (the failure took the FINISH tag one segment early) |
| 30 random rule-compliant courses (seeds 400-429) | 29/30 completed, mean 50.6 s (the failure: first segment only 1 m long and bending at once, invisible from the start position) |
