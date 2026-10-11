"""Batch runner for the course simulation: fly several settings over several seeds and compare them.

  python tools/sim_sweep.py [--seeds 1-8] [--jobs 6] [--track courses/tec_tarp] [--profile config/course_tec.ini] [-v] CONFIG [CONFIG ...]

Every CONFIG is "label: section.key=value section.key=value ..." (or just "label: nav.mode=course" for the profile
as it is). Each one is flown once per seed; one line per CONFIG:

  ok      courses finished / flown
  time    mean course time of the finished ones, take-off to landed (s)
  dev     path-tracking error: mean / rms / mean of the per-flight maxima / worst maximum (cm)
  tags    closest approach to the tag centres: mean / worst (cm)
  v       mean speed over the whole flight (m/s)
  cam     mean error of the camera pitch found in flight (deg) / vision ms per frame (only meaningful with --jobs 1)

Example: what does the altitude do?
  python tools/sim_sweep.py "1.0 m: mission.altitude_cm=100" "1.2 m: mission.altitude_cm=120" "1.5 m: mission.altitude_cm=150"

The executable is taken from TELLO_RACE_EXE, else C:/dev/build/tello-race/tello_race.exe, else build/tello_race.
"""
import os, re, subprocess, sys, statistics as st
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
EXE = next((p for p in (os.environ.get("TELLO_RACE_EXE", ""), "C:/dev/build/tello-race/tello_race.exe",
                        str(REPO / "build" / "tello_race"), str(REPO / "build" / "tello_race.exe")) if p and Path(p).exists()), None)
if not EXE:
    sys.exit("tello_race executable not found: set TELLO_RACE_EXE")

argv = sys.argv[1:]
seeds, jobs, track, profile, verbose = "1-8", 6, str(REPO / "courses" / "tec_tarp"), str(REPO / "config" / "course_tec.ini"), False
configs = []
i = 0
while i < len(argv):
    a = argv[i]
    if a == "--seeds": seeds = argv[i + 1]; i += 2
    elif a == "--jobs": jobs = int(argv[i + 1]); i += 2
    elif a == "--track": track = argv[i + 1]; i += 2
    elif a == "--profile": profile = argv[i + 1]; i += 2
    elif a == "-v": verbose = True; i += 1
    else: configs.append(a); i += 1
if not configs:
    sys.exit(__doc__)
if "-" in seeds:
    lo, hi = seeds.split("-")
    seed_list = list(range(int(lo), int(hi) + 1))
else:
    seed_list = [int(s) for s in seeds.split(",")]

ROW = re.compile(r"episode\s+\d+ seed (\d+): (\w+)\s+segments=(\d+)/(\d+) time=\s*([\d.]+)s\s+path dev mean ([\d.]+) rms ([\d.]+) max ([\d.]+) m\s+tag pass ([\d/]+) cm\s+speed ([\d.]+) m/s\s+\((.*)\)")
COST = re.compile(r"vision cost per frame: ([\d.]+) ms .*camera found at pitch ([\-\d.]+) roll ([\-\d.]+) deg, height tags/sensor ([\d.]+) \((\d+) sightings; true ([\-\d.]+)\)\s+stick gain ([\d.]+)")


def run(job):
    label, sets, seed = job
    # a flight that goes wrong is cut short: a good one takes 25-35 s of simulated time
    cmd = [EXE, "sim", "--headless", "--no-realtime", "--config", str(REPO / "config" / "default.ini"), "--profile", profile,
           "--track", track, "--episodes", "1", "--seed", str(seed), "--set", "sim.max_time_s=90", "--set", "sim.idle_limit_s=30"]
    for s in sets:
        cmd += ["--set", s]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=900, cwd=str(Path(EXE).parent)).stdout
    except subprocess.TimeoutExpired:
        return label, seed, None, None, "timeout"
    row = cost = None
    for ln in out.splitlines():
        m = ROW.search(ln)
        if m: row = m
        m = COST.search(ln)
        if m: cost = m
    return label, seed, row, cost, out


jobs_list, parsed = [], []
for c in configs:
    if ":" in c.split("=")[0]:
        label, rest = c.split(":", 1)
    else:
        label, rest = c, c
    sets = rest.split()
    parsed.append((label.strip(), sets))
    for s in seed_list:
        jobs_list.append((label.strip(), sets, s))

results = {}
with ThreadPoolExecutor(max_workers=jobs) as ex:
    for label, seed, row, cost, out in ex.map(run, jobs_list):
        results.setdefault(label, []).append((seed, row, cost, out))

print("%-34s %-6s %6s  %-24s %-10s %5s  %s" % ("config", "ok", "time", "dev mean/rms/max/worst cm", "tags cm", "v", "cam err / cost"))
for label, _ in parsed:
    rs = results[label]
    fin = [r for r in rs if r[1] and r[1].group(2) == "FINISHED"]
    n = len(rs)
    bad = [(r[0], (r[1].group(2) + " " + r[1].group(3) + "/" + r[1].group(4) + " " + r[1].group(11)) if r[1] else "no result") for r in rs if not (r[1] and r[1].group(2) == "FINISHED")]
    if fin:
        t = st.mean(float(r[1].group(5)) for r in fin)
        dm = 100 * st.mean(float(r[1].group(6)) for r in fin)
        dr = 100 * (st.mean(float(r[1].group(7)) ** 2 for r in fin)) ** 0.5
        dx = [100 * float(r[1].group(8)) for r in fin]
        tp = [float(v) for r in fin for v in r[1].group(9).split("/")]
        v = st.mean(float(r[1].group(10)) for r in fin)
        cams = [abs(float(r[2].group(2)) - float(r[2].group(6))) for r in fin if r[2] and int(r[2].group(5)) > 0]
        cost = st.mean(float(r[2].group(1)) for r in fin if r[2]) if any(r[2] for r in fin) else 0
        print("%-34s %2d/%-3d %6.1f  %4.1f /%4.1f /%5.1f /%5.1f   %4.1f /%3.0f  %5.2f  %s / %.1f ms" % (
            label[:34], len(fin), n, t, dm, dr, st.mean(dx), max(dx), st.mean(tp), max(tp), v,
            ("%.1f" % st.mean(cams)) if cams else "-", cost))
    else:
        print("%-34s %2d/%-3d" % (label[:34], 0, n))
    for seed, why in bad:
        print("      seed %d: %s" % (seed, why))
    if verbose:
        for r in sorted(rs, key=lambda r: r[0]):
            if r[1]: print("      ", r[1].group(0)[:200])
