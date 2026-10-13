"""Does the navigator get through the simulated course, and what does it complain about on the way?

  python tools/sim_events.py [--seeds 1-20] [--jobs 6] [--first] [--real-camera] [--logs DIR] [--exe tello_race] [--repo DIR] CONFIG [CONFIG ...]

Every CONFIG is "label: section.key=value section.key=value ..." (just "label:" for the profile as it is). Each one is
flown once per seed as a later flight (mirror angle and lean signs already measured; --first: as a first flight). One
line per CONFIG: courses finished / flown, mean course time and path error of the finished ones, and in how many flights
each of the navigator's "something went wrong" messages appeared:

  flown     this tape was flown before: going back to the tag            (commit e043a8f only)
  back      this way leads back to a tag already flown over
  6s        the start of the next line was not reached in 6 s
  rise      no further tape in sight: rising 30 cm
  lost      lost the tape for ... s
  ends      the tape ends here and no tag was read
  wrongtag  flew over tag IDx where IDy was expected

--real-camera switches on what the real camera shows and a clean rendering does not: the values that stand as comments
between the two "real camera" marks in the [sim] section of config/course_tec.ini (measured on recorded flights, see
docs/COURSE_TEC.md).

With --repo and --exe two builds of the navigator can be flown over the same simulated camera, for example the one in
the tree against an older commit in a git worktree with the simulator commits picked on top:

  python tools/sim_events.py --seeds 1-20 "fixed: course.heading=fixed sim.rc_yaw_rate=0.65" "path: course.heading=path sim.rc_yaw_rate=0.65"
  python tools/sim_events.py --seeds 1-20 --repo ../wt-old --exe ../b-old/tello_race.exe "fixed: ..." "path: ..."

The executable is taken from --exe, else TELLO_RACE_EXE, else C:/dev/build/tello-race/tello_race.exe, else build/tello_race.
"""
import os, re, subprocess, sys, statistics as st
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

argv = sys.argv[1:]
repo, exe = Path(__file__).resolve().parents[1], None
seeds, jobs, logs, first, real_camera = "1-20", 6, None, False, False
configs = []
i = 0
while i < len(argv):
    a = argv[i]
    if a == "--exe": exe = argv[i + 1]; i += 2
    elif a == "--repo": repo = Path(argv[i + 1]).resolve(); i += 2
    elif a == "--seeds": seeds = argv[i + 1]; i += 2
    elif a == "--jobs": jobs = int(argv[i + 1]); i += 2
    elif a == "--logs": logs = argv[i + 1]; i += 2
    elif a == "--first": first = True; i += 1
    elif a == "--real-camera": real_camera = True; i += 1
    else: configs.append(a); i += 1
if not configs:
    sys.exit(__doc__)
exe = next((p for p in (exe, os.environ.get("TELLO_RACE_EXE", ""), "C:/dev/build/tello-race/tello_race.exe",
                        str(repo / "build" / "tello_race"), str(repo / "build" / "tello_race.exe")) if p and Path(p).exists()), None)
if not exe:
    sys.exit("tello_race executable not found: --exe or TELLO_RACE_EXE")
if "-" in seeds:
    lo, hi = seeds.split("-")
    seed_list = list(range(int(lo), int(hi) + 1))
else:
    seed_list = [int(s) for s in seeds.split(",")]
# a later flight: the mirror angle is known (and is what it really is), the telemetry signs are confirmed
LATER = ["ground.mount_verified=1", "ground.imu_signs_verified=1", "camera.pitch_deg=93", "camera.roll_deg=1.5", "sim.cam_pitch_true_deg=93"]
CAMERA = []
if real_camera:   # "# key = value   # why" between the marks -> sim.key=value
    inside = False
    for line in (Path(__file__).resolve().parents[1] / "config" / "course_tec.ini").read_text(encoding="utf-8").splitlines():
        if "real camera ---" in line: inside = "end of" not in line; continue
        m = re.match(r"#\s*(\w+)\s*=\s*([^#]+)", line) if inside else None
        if m: CAMERA.append("sim.%s=%s" % (m.group(1), m.group(2).replace(" ", "")))
    if not CAMERA:
        sys.exit("no real-camera values found in config/course_tec.ini")
ROW = re.compile(r"episode\s+\d+ seed (\d+): (\w+)\s+segments=(\d+)/(\d+) time=\s*([\d.]+)s\s+path dev mean ([\d.]+)")
MSG = {"flown": "this tape was flown before", "back": "leads back to a tag already flown over", "6s": "was not reached in 6 s",
       "rise": "rising 30 cm", "lost": "lost the tape for", "ends": "the tape ends here and no tag was read", "wrongtag": "where ID"}


def run(job):
    label, sets, seed = job
    # a flight that goes wrong is cut short: a good one takes 25-35 s of simulated time
    cmd = [exe, "sim", "--headless", "--no-realtime", "--config", str(repo / "config" / "default.ini"), "--profile", str(repo / "config" / "course_tec.ini"),
           "--track", str(repo / "courses" / "tec_tarp"), "--episodes", "1", "--seed", str(seed), "--set", "sim.max_time_s=90", "--set", "sim.idle_limit_s=30"]
    for s in ([] if first else LATER) + CAMERA + sets:
        cmd += ["--set", s]
    if logs: cmd += ["--log-level", "debug"]
    try:      # warnings go to stderr
        out = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=900, cwd=str(Path(exe).parent)).stdout
    except subprocess.TimeoutExpired:
        return label, seed, None, ""
    if logs:
        os.makedirs(logs, exist_ok=True)
        Path(logs, re.sub(r"[^A-Za-z0-9_.-]+", "_", label) + "_s%d.log" % seed).write_text(out, encoding="utf-8")
    row = None
    for ln in out.splitlines():
        m = ROW.search(ln)
        if m: row = m
    return label, seed, row, out


todo, order = [], []
for c in configs:
    label, _, rest = c.partition(":")
    order.append(label.strip())
    todo += [(label.strip(), rest.split(), s) for s in seed_list]
res = {}
with ThreadPoolExecutor(max_workers=jobs) as ex:
    for label, seed, row, out in ex.map(run, todo):
        res.setdefault(label, []).append((seed, row, out))
print("%-30s %-6s %6s %6s  %s" % ("config", "ok", "time", "dev cm", "flights with: " + " ".join(MSG)))
for label in order:
    rs = res[label]
    fin = [r for r in rs if r[1] and r[1].group(2) == "FINISHED"]
    t = st.mean(float(r[1].group(5)) for r in fin) if fin else 0
    d = 100 * st.mean(float(r[1].group(6)) for r in fin) if fin else 0
    print("%-30s %2d/%-3d %6.1f %6.1f  %s" % (label[:30], len(fin), len(rs), t, d, " ".join("%s=%d" % (k, sum(1 for r in rs if v in r[2])) for k, v in MSG.items())))
    for seed, row, out in rs:
        if not (row and row.group(2) == "FINISHED"):
            why = re.search(r"episode.*\((.*)\)\s*$", out, re.M)
            print("      seed %d: %s %s  %s" % (seed, row.group(2) if row else "no result", (row.group(3) + "/" + row.group(4)) if row else "", why.group(1)[:90] if why else ""))
