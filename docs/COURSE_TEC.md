# Course mode: the taped course seen through the mirror

Profile `config/course_tec.ini`: white tarp, blue path, yellow decoy tapes, tags 1 → 2 → 3 → 4, flown with the
**original mirror clip** (`Tello_Mirror_Clip_2.STL`, 45° seat).

![simulated flight over the photographed course](media/course_tec_sim.jpg)

*A simulated flight over the photographed course. Green: less than 3 cm from the tape centre, yellow: less than
6 cm. White dots: one per second.*

## What it does

With that clip the front camera looks straight down, a little backwards (96-99° measured; 90° is straight
down). From 1.2 m it sees a patch of floor about 1.25 m wide and 0.95 m long with the drone above its front third:

| mirror angle | floor seen ahead | behind | to each side |
|---|---|---|---|
| 96° | 0.33 m | 0.62 m | 0.63 m |
| 97° | 0.31 m | 0.65 m | 0.63 m |
| 99° | 0.26 m | 0.70 m | 0.63 m |

The line-ahead-of-the-nose navigator has nothing to work with in that view, so this profile switches to a second
one (`nav.mode = course`):

* every piece of tape of the chosen colour in the picture becomes a centre-line on the floor, in metres;
* the drone follows the tape it is on, slows down for corners, flies over the tag at its end, crosses to the
  nearest other tape end (the start of the next segment) and goes on, in the order `course.tag_order`;
* it lands on the last tag;
* the nose either stays where it pointed at take-off, the drone moving forwards, sideways and backwards
  (`course.heading = fixed`), or turns to point along the route (`course.heading = path`). The path following
  underneath is the same; the page switches between the two.

The turn meaning of the tags (ID1 forward, ID2 left, ID3 right) is **not** used: on this course it does not match
the layout (after ID3 the path leaves about 100° to the left). The rule is "follow the chosen colour, visit the
tags in order".

## Running it

```
# simulator, on the photographed course (add --set web.enabled=1 to watch it in the browser, /sim3d for 3D)
tello_race sim --profile config/course_tec.ini --track courses/tec_tarp

# real drone
tello_race fly --profile config/course_tec.ini --set web.enabled=1
```

On the ground-station page the panel **Path to follow** has three buttons, BLUE / YELLOW / BLACK. The video shows
which one is active ("FOLLOW BLUE") and draws the route the drone is following in that colour, with a green cross
on the next tag or line start. Under them, **NOSE FIXED / NOSE FOLLOWS PATH / NOSE TURNS AHEAD** chooses whether the
drone keeps its take-off heading, turns with the route, or turns ahead of it (the video label says `NOSE: FIXED`,
`NOSE: PATH` or `NOSE: AHEAD`). Both can be changed on the ground or in MANUAL, not while the mission flies. Keys in
the desktop window: `b`, `y`, `k`, and `n` for the nose (fixed, path, ahead, fixed ...).

## Placing the drone

On the first tape, **about a hand-span (15 cm) after its beginning, nose along the tape**. The camera looks down
and backwards, so it needs some tape behind the drone to start from. ±15° of heading is fine; turned by 90° or
180° it is not (the camera would be blind in the direction of travel).

## First flight with a newly mounted mirror

The exact angle of the mirror is not known until tags have been seen. So on the first flight:

1. the first segment is flown with the configured guess (`camera.pitch_deg = 96`): the drone may run up to
   10 cm beside the tape there;
2. at tag 1 it stops and moves a hand-span around the tag for about 5 s ("measuring the camera angle over the
   tag"). The tag is seen all over the picture, which gives the mirror angle to a few tenths of a degree and
   shows which way the telemetry counts pitch and roll;
3. from there on the flight is the normal one.

After landing, what was measured (mirror angle, lean signs, stick gain) is written to
`config/learned_course_tec.ini` and loaded at the next start: later flights skip step 2 and are exact from the
first metre. **Delete that file after re-seating the clip or changing the drone.** If the mirror has moved a few
degrees since, the flight still works: the angle keeps being refined from every tag.

The same measurement can be made without flying: hold the drone level about a metre above a tag and move it
slowly so the tag crosses the whole picture a few times. The page video shows `calib ... [applied]` and the file
is written within seconds.

If the measured angle is outside 84-104° the mission lands on tag 1 and says so: beyond that the camera sees
almost nothing ahead of (or behind) the drone. Re-seat the clip.

Before pressing START the first time, on the page:

* the floor **ahead** of the drone is at the **top** of the video (`camera.flip = vertical`). If the image is the
  other way up the drone would fly against its commands; the navigator notices that within a second and lands
  ("image reversed: check camera.flip"), but it is cheaper to look first;
* the small black-and-white inset shows the blue tape as clean white strokes, and thin white lines run along the
  tape in the video. If the tape is missing there, the colour limits in `[line]` need adjusting to the real
  camera (`blue_lo`, `blue_hi`; hue 0-180);
* battery above 25 % (below that the mission refuses to take off).

## Why 1.2 m

Altitude sweep in the simulator (mirror angle known, 10 flights each):

| altitude | course time | path error (mean) | distance to the tag centres |
|---|---|---|---|
| 0.9 m | 26.1 s | 1.9 cm | 2.7 cm |
| 1.0 m | 25.6 s | 2.0 cm | 2.6 cm |
| **1.2 m** | **24.5 s** | 2.0 cm | 2.7 cm |
| 1.5 m | 25.2 s | 1.8 cm | 3.1 cm |
| 1.8 m | 26.2 s | 1.9 cm | 3.4 cm |

* Accuracy is flat: what limits it is not the picture. (In this batch the stick gain was still the guess, so the
  times are 2 s shorter than the later-flight figure below; the comparison between heights holds.)
* Lower is shorter (less climbing after take-off, shorter landing) until the view gets too small: below about
  1 m the start of the next tape (12-32 cm beside each tag) is close to the edge of the picture, and the drone
  has to slow down because it sees too little tape ahead.
* Every degree of error in the mirror angle or in the lean of the drone moves the floor by 2.1 cm at 1.2 m,
  2.6 cm at 1.5 m.
* At 1.2 m a tag is 123 pixels across and the tape 38: plenty.

So 1.2 m, and the last 60 cm before the finish tag are flown coming down to 0.85 m (`course.finish_altitude_cm`),
which shortens the landing.

## What the simulator says

> The numbers in this section and the two after it were made before the real drone's answer to the sticks was
> measured. The simulator of that day answered about three times faster than the real drone does, so its times
> are too good by a few seconds and its path errors by a centimetre or two. The comparisons between settings
> still hold. Current numbers: "The two nose modes" and "The recorded flight" below.

The simulator flies the photographed course itself (the photo is rectified with the four tags and used as the
floor), with motion blur, 130 ms of video latency, telemetry at 10 Hz in whole degrees, a drone that leans when it
accelerates, a start position off by 5 cm and 5°, an unknown stick gain (configured 0.015, real 0.020 m/s per
unit) and a random drift of up to 8 cm/s.

| | finished | course time | path error mean / worst moment | tag centres |
|---|---|---|---|---|
| first flight (mirror guessed 96°, really 97°; stick gain guessed) | 20 / 20 | 29.8 s | 2.1 cm / 7.7 cm | 2.4 cm |
| later flights (mirror, lean signs and stick gain learned) | 20 / 20 | 26.5 s | 1.8 cm / 6.1 cm | 3.0 cm |
| ... without the drift | 20 / 20 | 26.0 s | 1.2 cm / 4.7 cm | 2.6 cm |

Course time is take-off to landed; about 3 s of it are the take-off and 3.5 s the landing, and on the first flight
5 s are the measurement at tag 1. The route is 5.95 m. (The first flight flies faster than intended until the stick
gain has been measured: the guess is 25 % low on purpose, the careful side.)

Things that were varied one at a time and still finished every time. Twenty first flights each: mirror really at
92° or at 102°; telemetry pitch and roll signs both wrong; drone answering the sticks in 0.7 s instead of 0.35;
video latency 300 ms; picture 40 % darker. Six to ten flights each: mirror roll -3° / +4°; stick gain 0.010 and
0.030; sideways gain different from forward; sticks dead below 4 or 8 units; height sensor reading 7 % low or
high; focal length 5 % off; telemetry at 5 Hz; twice the image noise; 1/25 s exposure; picture 20 % brighter;
drift doubled; start 12 cm and 15° off; mirror moved to 101° after having been learned at 97°. A slow drone or a
long latency costs accuracy (4-5 cm mean instead of 2), not the course.

Where it stops working: mirror beyond about 103° (the mission lands on tag 1 with a message), a picture less than
half as bright as the photo, the drone put down facing sideways or backwards.

## Speed against precision

| preset | settings | course time | path error (mean) |
|---|---|---|---|
| careful | `lat_accel_ms2 = 0.5`, `accel_ms2 = 0.7`, `v_cruise_ms = 0.45` | 28.7 s | 1.7 cm |
| **profile** | 0.8 / 0.9 / 0.60 | 26.5 s | 1.8 cm |
| fast (`config/course_tec_fast.ini`) | 1.2 / 1.2 / 0.80, `v_gap_ms = 0.35`, `response_tau_s = 0.35` | 23.1 s | 2.1 cm |
| faster | 1.8 / 1.5 / 1.0, `v_gap_ms = 0.45`, `response_tau_s = 0.35` | 21.9 s | 2.2 cm |

(Later flights, everything learned, 20 flights each.) The fast one is ready as a second profile:
`--profile config/course_tec.ini --profile config/course_tec_fast.ini`.

All in `[course]`:

* `lat_accel_ms2` sets the speed in corners (higher = faster and rounder). This is the main knob: the zigzag
  before tag 3 is where the time goes.
* `v_cruise_ms` only matters on the two long straights.
* `response_tau_s` is how quickly the navigator believes the drone answers the sticks. 0.5 is the careful guess
  for a drone nobody has measured; if the real one turns out snappy, 0.35 gains 1 s. The other way round is
  worse: 0.35 assumed with a drone that takes 0.7 s lost the course in 3 flights of 6.
* `k_cross` / `k_bias` pull the drone back onto the tape and hold it against a steady drift.
* `finish_altitude_cm = 0` lands from the cruise height instead.

Fly the fast presets only after the profile has flown well on the real drone: the simulator's drone is a model.

## What the vision costs, and the battery

The vision runs on the ground station (the Pi), not on the drone. The drone's battery goes into the motors; what
the ground station computes does not change how long it flies. What does: the flight being short (26.5 s instead
of the 45-50 s of the turn-and-go navigator on a course of this size), and not leaving the drone switched on,
streaming, on the ground (it also overheats there).

For the ground station itself, the vision per camera frame, measured on the laptop with nothing else running.
The absolute times swing by a factor of four with the laptop's power state, so two sittings are shown; the
proportion is what holds:

| | sitting A | sitting B |
|---|---|---|
| whole frame searched for tags, every frame, tape at 640 px | 7.5 ms | 37.9 ms |
| tags searched in small windows, every frame | 4.0 ms | 15.3 ms |
| ... and only every other frame | 2.8 ms | 11.3 ms |
| ... and the tape at 480 px (the profile) | | 7.4 ms |

About a fifth of what it was. The speckled floor around the tarp is what made the tag search expensive
(thousands of little shapes to reject): the search now runs only around tags already followed and around dark
squares of the right size, and over the whole frame once a second as a safety net (`tag.windows`,
`tag.full_frame_every_s`). The tape is traced at half the camera width (`line.proc_width = 480`): half the cost,
same tracking in the simulator (10/10 flights, 2.1 cm).

On the Pi 5 itself (headless simulator, 2026-10-12): **4.2 ms per frame** with the profile (tags 1.6 ms, tape
2.5 ms) against 19.5 ms the old way (tags 14.0 ms, tape 5.4 ms). Flying the simulated course in real time and
serving the page to a browser, the Pi kept 26 frames a second. To repeat it, no drone needed:

```
./build/tello_race sim --headless --no-realtime --profile config/course_tec.ini --track courses/tec_tarp --episodes 1 --seed 1 | grep "vision cost"
```

While nothing is flying the ground station now processes 10 pictures a second instead of 30
(`mission.idle_fps`).

`tello.video_fps`, `tello.video_bitrate` and `tello.video_resolution` pass the SDK 3.0 video settings to the
drone (empty = untouched, the default). A lower bitrate is a little less work and heat for the drone. They have
not been tried on the real drone; keep the resolution `high` (the vision expects 960x720).

## What makes it land early (on purpose)

* the drone moves against its commands for more than half a second: the image is upside down (`camera.flip`);
* mirror angle outside 84-104° once measured;
* no tape of the chosen colour within 1.2 m after take-off (after creeping forward 40 cm);
* the path lost for 5 s; no next tape found near a tag after rising twice by 30 cm;
* more than `course.max_range_m` (3.5 m) from where it took off;
* battery under 25 % at the start; LAND or EMERGENCY from the page at any time.

## The two nose modes

| | fixed | follows the path |
|---|---|---|
| simulator with the real drone's response, later flights (20 each) | 20 / 20, 26.6 s, 3.1 cm | 20 / 20, 27.6 s, 3.7 cm |
| the same, first flight (20 each) | 20 / 20, 34.3 s | 20 / 20, 34.5 s |

With the nose fixed the camera's long side (it sees further behind than ahead) looks along whatever direction the
drone happens to travel in; with the nose on the path the short side always looks ahead, so every tag comes into
view late and is approached slowly, and at a tag the drone may turn on the spot to find the next tape. That is
where the extra second goes. `course.heading_look_m` (how far ahead the nose aims) and `course.max_yaw_rc` (how
fast it turns, 40) shape how it looks. A drone keeps going the way it was moving while its nose swings round, so the
navigator's motion model lives in a frame that does not turn, and the stick direction is worked out for the heading
the nose will have when the command acts.

`course.camera_ahead_m = 0.06`: the mirror looks down from 6 cm ahead of the drone's centre. It is the centre that
is kept on the tape and over the tags, in both modes.

## The third nose mode: turning ahead (`course.heading = lead`)

In the recorded path-mode flight the nose lagged the direction of travel: within 15° of it for 34 % of the flight,
more than 45° off for 41 %. The real drone turns 0.65°/s per yaw stick unit (`sim.rc_yaw_rate`, now in the profile;
the simulator assumed 1.2) and path mode limits the yaw stick to 40. With that rate the simulator shows the same lag.

`lead` is a mode of its own, so that the two modes that have flown the real course fly exactly as they did: for
`fixed` and `path` the simulator output is the same as before, event for event (8 seeds each, first and later
flights). What `lead` does differently from `path`:

* the yaw stick goes up to 100 (65°/s) at 1.5 stick units per degree (`lead_max_yaw_rc`, `lead_k_yaw`);
* the nose takes the direction the route *has* at a point ahead (`lead_look_m` + `lead_look_s` × speed beyond the
  preview point) instead of pointing *at* a point: it turns before a corner, and across the gap after a tag it takes
  the direction of the next tape itself;
* the drone flies slower while the nose is more than `lead_slow_deg` (25°) from where it should point, down to
  `lead_slow_min` (0.45) of the speed, reached 60° further off;
* what is remembered from earlier pictures is turned by the heading change between the two pictures, about the
  drone's centre (the camera is 6 cm ahead of it).

**The nose does not look past a tag.** Until a tag is flown over, the nose stays along the tape that leads to it and
turns towards the next tape only after. As first written the nose swung towards the next tape before the tag, and 1
simulated flight in 30 flew back along the tape it had come on at tag 3. The cause is in the vision and concerns
every mode: when the last straight piece of a tape lies at 40 - 60° to the drone's axes, the thinning of the
top-down grid (Zhang-Suen) eats it from its free end, up to the whole piece back to the nearest bend. In the last
1.5 s before tags 2 and 3 (18 flights each of `path`, `lead` and `lead` as first written) the end was lost in 74 of
the 526 frames in which that piece lay at 30 - 60° to an axis (40 % of those at 40 - 50°), and in 1 of 3060 within
30° of an axis. The tape that leads to tag 3 bends by 55° a hand-span before its end: with the end lost, the
direction of travel in the frame the tag is reached is that of the piece before the bend, the tape just flown no
longer looks like the one the drone came on, and the next tape does. How the tape end lies to the drone's axes in the
last 0.2 s before a tag (18 flights each):

| | tag 2: median / worst | tag 3: median / worst | frames with the end lost |
|---|---|---|---|
| path | 29° / 45° | 18° / 30° | 6 of 229 |
| lead | 0° / 13° | 5° / 19° | 0 of 238 |

Simulator with the real drone's response, 30 flights each:

| | finished | course time | path error mean / worst moment | nose within 15° of the travel direction | more than 45° off |
|---|---|---|---|---|---|
| path, later flights | 30 / 30 | 26.8 s | 3.5 cm / 12.1 cm | 28 % | 43 % |
| lead, later flights | 30 / 30 | 31.7 s | 3.7 cm / 12.5 cm | 54 % | 23 % |
| lead, first flight | 30 / 30 | 41.1 s | 3.8 cm / 14.8 cm | 54 % | 24 % |
| the real flight in path mode | | | | 34 % | 41 % |

The 5 s more are the slowing while the nose comes round (without it: 27.6 s, 46 % / 31 %). Of the moments beyond
45°, 90 % fall in the two seconds after a tag, 70 % in the gap itself: there the nose already looks along the next
tape while the drone still flies to its start, and it starts to turn late (the yaw stick is slewed like the others,
0.4 s to full, and in the simulator the yaw answers 0.35 s after that). With a simulated drone that differs from the
navigator's model, 12 flights each, all finished: answering 30 % slower or faster, yaw rate 0.45 or 1.0°/s per
unit, yaw time constant 0.5 s, sticks below 8 units doing nothing (path error 4.0 - 5.0 cm mean).

How the values were chosen (12 flights each, all finished; 51 % / 24 % for the chosen ones at that stage, before
the nose took the next tape's own direction across a gap): a gain of 2.5 swings the nose 10 - 20° past and aims no
better (49 % / 25 %); a stick limit of 70 costs little (47 % / 26 %), 50 gives half of it away (41 % / 29 %);
slowing down to 0.3 of the speed gains nothing (51 % / 25 %).

`lead` has flown in the simulator only.

## The first real flights (2026-10-12)

Five missions with the nose fixed, from the page. The fifth flew the whole course, tags 1-2-3-4 and a landing on
tag 4, in 28.7 s. What the Pi's log showed about the other four, and what was changed:

* two were ended by the range limit ("3.5 m from the take-off point") while inside the course: the dead reckoning
  it relies on over-reads by half when the stick gain is still the guess. Limit now 6 m;
* after a tag the next tape was sometimes taken to be the far end of a piece of the tape just flown (the real tape
  breaks into pieces under glare and where the yellow one crosses it). A tape now qualifies only if it runs away
  from the tag;
* while waiting over a tag for the next tape to come into sight, the drone held a remembered point and drifted off
  it. It now holds on the tag it sees;
* the mirror measurement at tag 1 ran again on a later flight. Once per run now;
* the drone flew with the point under the camera on the tape, not its own centre (6 cm behind). See above.

Measured on the real drone: mirror 92.5-94.3° pitch, 1-2.4° roll (the fit moves about 1° from flight to flight);
telemetry pitch and roll both count the lean the way the model assumes; stick gain 0.010-0.015 m/s per unit; blue
tape, tags and the white tarp are seen as in the simulator; 23-24 frames a second on the Pi in flight. The drone's
board went from 42 to 77 °C over those ten minutes: switch it off between flights.

## The recorded flight and what it changed (2026-10-12)

A sixth mission was started from the laptop with the station recording (`logs/run_20261012_140921.avi/.csv/.log` on
the Pi): tags 1-2-3-4, landed on tag 4, 38.3 s. It ran 36 cm past tag 1, circled tag 2 for 5 s with a 25 cm wobble,
left the third tape by 30 cm at a corner and ran 40 cm past tag 3, off the tarp, where it rose twice before it
found the last tape again.

**The drone answers the sticks far more slowly than assumed.** The recording has the tags in the picture (their
true motion) and the sticks that were sent. `tools/flight_response.py` fits the response:

| | sideways | forwards | what the controller assumed |
|---|---|---|---|
| nothing happens for | 0.51 s | 0.5 - 0.6 s | 0.13 s |
| then 63 % of the new speed after | 0.80 s | 0.5 - 0.75 s | 0.5 s |
| speed per stick unit | 0.0133 m/s | 0.012 - 0.014 m/s | 0.0112 (learned in flight) |
| drift with the sticks at rest | 1 cm/s | 2 cm/s | |

Of the dead time, 0.15 s is the age of the picture (the height sensor and the tag size in the picture, compared
during the climb, are 0.06 - 0.10 s apart); the rest is the drone. The fit reproduces the tag tracks to 1 - 3 cm;
the response the controller believed in is off by 6 - 12 cm on the same tracks. With that response in the simulator
(`sim.tau`, `sim.rc_delay_s`, `sim.rc_vel_ms`, now in the profile) the controller as flown completes 3 flights of 12.

What the navigator does about it:

* `course.cmd_delay_s = 0.35`, `course.response_tau_s = 0.75`, `course.latency_s = 0.15`: the commands already on
  their way say where the drone will be, and how fast, at the moment a new command starts to act. Everything
  (preview point, braking, "over the tag") is worked out for that moment;
* `course.shape_tau_s = 0.3`: the sticks ask for more than is wanted, by what is still missing, so the drone answers
  as if its time constant were 0.3 s;
* the drift integral takes only the sideways part of the error and is small (`k_bias = 0.3`, at most 6 cm/s): in the
  flight it had wound up to 9 cm/s forwards from being behind along the tape, and pushed the drone past the tags.

Checked in the simulator against a drone 30 % slower or faster than measured, a stick gain of 0.010 - 0.017, a
yaw that answers half or twice as fast, and sticks below 8 units doing nothing: 12 of 12 flights each (11 of 12 in
two of the cases). Not known: whether small sticks really do less than their share. While searching after tag 3 the
drone was sent -10 forwards for 3 s and moved at 5 cm/s, a third of what the fit says; it was climbing at the time.

**A lamp's reflection washes the tape pale.** The first 25 cm of the tape after tag 1 came out light blue
(saturation 35 - 79, the limit was 80) and the colour mask lost them; where the reflection sat moved with the drone,
so the "start of the next tape" ran away from it for two seconds. `line.tape_s_auto = true`: the saturation limit
comes down to just above the tarp's own saturation (median + 22, here 37; never above the configured limit). The
tarp's 95th percentile was 25. The dark floor beyond the tarp stays out of the mask.

**After a tag with no next tape in sight** the drone now returns over the tag first and only then rises for a wider
view (the next tape starts next to the tag).

## Not verified on the real drone yet

* everything in "The recorded flight" above: the new controller, the paler tape limit and the return over the tag
  have flown in the simulator only;
* the nose-follows-path mode has only flown in the simulator, and how the real drone's yaw answers is not measured
  (record a flight in that mode: the same fit will tell);
* the `lead` nose mode has flown in the simulator only. Of the real yaw only the rate is measured (0.65°/s per
  stick unit, from a flight limited to 40): whether it holds up to a stick of 100, and how late the yaw answers,
  a recorded flight in that mode will tell.

To record a flight: press RECORD on the page before START (video, telemetry table and log go to `logs/` on the Pi).

## Another course from another photo

```
uv run tools/course_from_photo.py photo.jpg courses/<name> [--tag-size 0.16] [--order 1,2,3,4]
tello_race sim --profile config/course_tec.ini --track courses/<name>
```

The photo must show all the tags; they give the scale and the perspective. The tool writes the floor image, the
course description (`course.txt`) and `debug.png`, which shows what it understood: check it. The simulated drone
starts 15 cm along the first segment (`--start-in`).
