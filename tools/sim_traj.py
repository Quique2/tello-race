# /// script
# requires-python = ">=3.11"
# dependencies = ["opencv-python-headless", "numpy"]
# ///
"""Fly one simulated course and draw the true trajectory over the floor image.

  uv run tools/sim_traj.py OUT.png [--seed N] [--track courses/tec_tarp] [--profile config/course_tec.ini] [section.key=value ...]

The trajectory is coloured by its distance to the route (green < 3 cm, yellow < 6 cm, red beyond); a white dot
every second. Prints the result row and where the largest deviations happened; the full log goes to OUT.png.log.
(On a first flight the hand-span move around tag 1 that measures the mirror shows up as a deviation here.)

The executable is taken from TELLO_RACE_EXE, else C:/dev/build/tello-race/tello_race.exe, else build/tello_race.
"""
import os, re, subprocess, sys
from pathlib import Path
import numpy as np, cv2

REPO = Path(__file__).resolve().parents[1]
EXE = next((p for p in (os.environ.get("TELLO_RACE_EXE", ""), "C:/dev/build/tello-race/tello_race.exe",
                        str(REPO / "build" / "tello_race"), str(REPO / "build" / "tello_race.exe")) if p and Path(p).exists()), None)
if not EXE:
    sys.exit("tello_race executable not found: set TELLO_RACE_EXE")
if len(sys.argv) < 2:
    sys.exit(__doc__)
out_png = sys.argv[1]
seed, sets, track, profile = 1, [], str(REPO / "courses" / "tec_tarp"), str(REPO / "config" / "course_tec.ini")
argv = sys.argv[2:]
i = 0
while i < len(argv):
    if argv[i] == "--seed": seed = int(argv[i + 1]); i += 2
    elif argv[i] == "--track": track = argv[i + 1]; i += 2
    elif argv[i] == "--profile": profile = argv[i + 1]; i += 2
    else: sets.append(argv[i]); i += 1
cmd = [EXE, "sim", "--headless", "--no-realtime", "--config", str(REPO / "config" / "default.ini"), "--profile", profile,
       "--track", track, "--episodes", "1", "--seed", str(seed), "--set", "sim.trace_pose_s=0.033", "--set", "sim.max_time_s=90"]
for s in sets:
    cmd += ["--set", s]
out = subprocess.run(cmd, capture_output=True, text=True, timeout=900, cwd=str(Path(EXE).parent)).stdout
open(out_png + ".log", "w").write(out)

# course
world, ppm, segs, tags = 4.5, 300, [], []
for ln in open(os.path.join(track, "course.txt")):
    p = ln.split()
    if not p: continue
    if p[0] == "world_size": world = float(p[1])
    elif p[0] == "px_per_m": ppm = float(p[1])
    elif p[0] == "segment": segs.append(np.array([[float(v) for v in q.split(",")] for q in p[1:]]))
    elif p[0] == "tag": tags.append((int(p[1]), float(p[2]), float(p[3])))
route = []
for k, sg in enumerate(segs):
    route += [tuple(q) for q in sg]
    if k < len(tags): route.append((tags[k][1], tags[k][2]))
route = np.array(route)

pose = np.array([(float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4)))
                 for m in re.finditer(r"SIM pose t=([\-\d.]+) x=([\-\d.]+) y=([\-\d.]+) z=([\-\d.]+)", out)])
if len(pose) < 2:
    sys.exit("no flight in the log: see " + out_png + ".log")


def dist_route(p):
    a, b = route[:-1], route[1:]
    d = b - a
    l2 = (d ** 2).sum(1)
    t = np.clip(((p - a) * d).sum(1) / np.maximum(l2, 1e-12), 0, 1)
    return np.hypot(*(p - (a + d * t[:, None])).T).min()


img = cv2.imread(os.path.join(track, "world.png"))
S = 3                                   # drawn at 3x, cropped to the course
xs, ys = route[:, 0], route[:, 1]
x0, x1, y0, y1 = xs.min() - 0.35, xs.max() + 0.35, ys.min() - 0.35, ys.max() + 0.35
half = world * ppm / 2
c0, c1, r0, r1 = int(half + x0 * ppm), int(half + x1 * ppm), int(half - y1 * ppm), int(half - y0 * ppm)
crop = cv2.resize(img[r0:r1, c0:c1], None, fx=S, fy=S, interpolation=cv2.INTER_CUBIC)
crop = (crop * 0.75).astype(np.uint8)


def px(p):
    return int(round((half + p[0] * ppm - c0) * S)), int(round((half - p[1] * ppm - r0) * S))


cv2.polylines(crop, [np.array([px(q) for q in route], np.int32)], False, (255, 255, 255), 1, cv2.LINE_AA)
devs = []
for k in range(1, len(pose)):
    if pose[k, 3] < 0.5: continue        # on the ground or just leaving it
    d = dist_route(pose[k, 1:3])
    devs.append((d, pose[k, 0], pose[k, 1], pose[k, 2]))
    col = (0, 220, 0) if d < 0.03 else (0, 220, 255) if d < 0.06 else (0, 0, 255)
    cv2.line(crop, px(pose[k - 1, 1:3]), px(pose[k, 1:3]), col, 3, cv2.LINE_AA)
for k in range(len(pose)):
    if abs(pose[k, 0] - round(pose[k, 0])) < 0.017:
        cv2.circle(crop, px(pose[k, 1:3]), 5, (255, 255, 255), -1, cv2.LINE_AA)
cv2.imwrite(out_png, crop)
m = re.search(r"episode\s+\d+ seed.*", out)
print(m.group(0) if m else "no result row")
devs.sort(reverse=True)
seen = []
for d, t, x, y in devs:
    if all(abs(t - s) > 0.7 for s in seen):
        seen.append(t)
        print("  %.1f cm from the route at t=%.1f s  (x=%.2f y=%.2f)" % (100 * d, t, x, y))
    if len(seen) >= 6: break
