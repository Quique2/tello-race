"""Does this build decide on the frames of a recorded flight what the flight itself decided?

  python tools/replay_check.py [--exe tello_race.exe] [--profile config/course_tec.ini] [--tol 0.3] [--jpeg 95,80] [--clean] [--jobs 3] [-v] RECORDING [RECORDING ...]

Every RECORDING is "path/run_YYYYMMDD_HHMMSS: section.key=value section.key=value ...": the three files the ground
station's RECORD wrote (.avi .csv .log, named without the extension) and the settings that flight STARTED with (the
nose mode, and what the learned file held then: camera.pitch_deg, camera.roll_deg, course.rc_gain_ms, the ground.*
flags). Each one is replayed offline (tello_race replay --telemetry) and the NAV lines of the replay are compared with
the ones in the recorded .log. One line per RECORDING:

  SAME       the same tags reached in the same order, each within --tol seconds of the flight's time, the same phase
             changes in between, every "next line starts D m" within 5 cm, and no trouble the flight did not have
  DIFFERENT  anything else; the line says where the two first part
  FRAGILE    with --jpeg: some of the replays were the same and some were not

Trouble is a NAV line with "flown before", "leads back", "lost" or "not reached" in it.

The recording holds the frames as the recorder's MJPG left them, not quite the ones the flight saw, and a decision
that hangs on a few pixels can fall the other way for that alone. --jpeg 95,80 replays every RECORDING again with
the frames through one more JPEG generation of each quality: what all of them decide alike does not hang on pixels.

--clean is for a flight that went wrong, or was flown by other code: the replay is not held against the log, it only
has to reach its tags with rising IDs and without trouble (CLEAN / TROUBLE).

The replay is open-loop (the video shows what the real drone did, whatever is commanded now): past the first
difference the rest of the line says little. A replay whose tags lie centimetres from where the flight saw them was
given the wrong camera angle (the line says so): compare nothing before that is put right.

Exit code 0 when every RECORDING is SAME (CLEAN with --clean), 1 otherwise.

Example, the flight of 2026-10-12 18:34:
  python tools/replay_check.py --jpeg 95,80 "logs/run_20261012_183452: course.heading=path camera.pitch_deg=94.92 camera.roll_deg=2.85 course.rc_gain_ms=0.0139 ground.mount_verified=1 ground.imu_signs_verified=1 ground.imu_pitch_sign=1 ground.imu_roll_sign=1"
"""
import os, re, subprocess, sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
TROUBLE = ("flown before", "leads back", "lost", "not reached")

argv = sys.argv[1:]
exe, profile, tol, jpeg, clean, jobs, verbose = os.environ.get("TELLO_RACE_EXE", ""), str(REPO / "config" / "course_tec.ini"), 0.3, [], False, 3, False
recordings = []
i = 0
while i < len(argv):
    a = argv[i]
    if a == "--exe": exe = argv[i + 1]; i += 2
    elif a == "--profile": profile = argv[i + 1]; i += 2
    elif a == "--tol": tol = float(argv[i + 1]); i += 2
    elif a == "--jpeg": jpeg = [int(q) for q in argv[i + 1].split(",") if q]; i += 2
    elif a == "--jobs": jobs = int(argv[i + 1]); i += 2
    elif a == "--clean": clean = True; i += 1
    elif a == "-v": verbose = True; i += 1
    else: recordings.append(a); i += 1
if not recordings:
    sys.exit(__doc__)
EXE = next((p for p in (exe, "C:/dev/build/tello-race/tello_race.exe", str(REPO / "build" / "tello_race"),
                        str(REPO / "build" / "tello_race.exe")) if p and Path(p).exists()), None)
if not EXE:
    sys.exit("tello_race executable not found: give --exe or set TELLO_RACE_EXE")

NAV = re.compile(r"^\[\s*[\d.]+ (?:INF|WRN|ERR)\] NAV (.*)$")
TIMED = re.compile(r"^t=\s*([\d.]+)\s+(.*)$")
ANY_T = re.compile(r"\] (?:NAV|CRS|LEADIN|RPL) t=\s*([\d.]+)")
NEXT = re.compile(r"next line starts ([\d.]+) m from the tag")


def events(text):
    """The NAV lines of a log as (mission time, kind, key, text); kind is tag | phase | next | trouble | note."""
    raw, last = [], 0.0
    for n, ln in enumerate(text.splitlines()):
        m = ANY_T.search(ln)
        if m: last = float(m.group(1))
        m = NAV.match(ln.rstrip())
        if not m: continue
        body = m.group(1).strip()
        tm = TIMED.match(body)
        raw.append([n, float(tm.group(1)) if tm else None, last, tm.group(2).strip() if tm else body])
    out = []
    for k, (n, t, last, body) in enumerate(raw):
        if t is None:
            # a line without its own time belongs to the timed one printed right after it (the next line that was
            # chosen and the phase change it causes), else to the last time seen in the log
            nxt = raw[k + 1] if k + 1 < len(raw) else None
            t = nxt[1] if nxt and nxt[1] is not None and nxt[0] - n <= 2 else last
        tag, ph, nx = re.match(r"tag ID(\d+) reached", body), re.match(r"(\w+) -> (\w+)\b", body), NEXT.search(body)
        if tag: out.append((t, "tag", int(tag.group(1)), body))
        elif ph: out.append((t, "phase", (ph.group(1), ph.group(2)), body))
        elif nx: out.append((t, "next", float(nx.group(1)), body))
        elif any(w in body for w in TROUBLE): out.append((t, "trouble", next(w for w in TROUBLE if w in body), body))
        else: out.append((t, "note", None, body))
    return out


def steps(ev):
    """What is compared one by one: tags, next lines and the phase changes of the flight itself (not the start and the end)."""
    return [e for e in ev if e[1] in ("tag", "next") or (e[1] == "phase" and e[2][1] not in ("TAKEOFF", "LANDED", "ABORTED"))]


def compare(rec, rpl):
    """First difference between the flight's NAV lines and the replay's (None: none) and the trouble only the replay has."""
    a, b = steps(rec), steps(rpl)
    first = None
    for x, y in zip(a, b):
        same = x[1] == y[1] and (x[2] == y[2] if x[1] != "next" else abs(x[2] - y[2]) <= 0.05)
        if not same:
            first = "t=%.2f: \"%s\" where the flight has \"%s\" (t=%.2f)" % (y[0], y[3], x[3], x[0])
        elif abs(x[0] - y[0]) > tol:
            first = "\"%s\" at t=%.2f, in the flight at t=%.2f" % (y[3], y[0], x[0])
        if first: break
    if not first and len(a) != len(b):
        longer, who = (b, "the replay goes on with") if len(b) > len(a) else (a, "the replay stops before the flight's")
        e = longer[min(len(a), len(b))]
        first = "t=%.2f: %s \"%s\"" % (e[0], who, e[3])
    new = []
    for w in TROUBLE:
        nr, nf = sum(1 for e in rpl if e[1] == "trouble" and e[2] == w), sum(1 for e in rec if e[1] == "trouble" and e[2] == w)
        if nr > nf: new.append("\"%s\" x%d" % (w, nr) + (" (flight x%d)" % nf if nf else ""))
    return first, new


def in_order(ev):
    """Tags reached with rising IDs, none skipped over (the taped course has them in that order)."""
    ids = [e[2] for e in ev if e[1] == "tag"]
    return all(q > p for p, q in zip(ids, ids[1:])) and not any("where ID" in e[3] for e in ev)


def replay(job):
    base, sets, q = job
    cmd = [EXE, "replay", "--headless", "--no-realtime", "--config", str(REPO / "config" / "default.ini"), "--profile", profile,
           "--source", "file:" + base + ".avi", "--telemetry", base + ".csv", "--log-level", "debug"]
    for s in sets + (["replay.jpeg_quality=%d" % q] if q else []):
        cmd += ["--set", s]
    try:
        return subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, errors="replace", timeout=600,
                              cwd=str(Path(EXE).parent)).stdout
    except subprocess.TimeoutExpired:
        return ""


def tags_text(ev):
    t = [e for e in ev if e[1] == "tag"]
    return (" ".join("ID%d %.2f" % (e[2], e[0]) for e in t) or "none") + " s"


parsed = []
for r in recordings:
    base, _, rest = r.partition(": ")
    base = base.strip().rstrip(":")
    if base[-4:] in (".avi", ".csv", ".log"): base = base[:-4]
    for ext in (".avi", ".csv", ".log"):
        if not Path(base + ext).exists(): sys.exit("replay_check: %s%s not found" % (base, ext))
    parsed.append((base.replace("\\", "/"), rest.split()))
jobs_list = [(base, sets, q) for base, sets in parsed for q in [0] + jpeg]
with ThreadPoolExecutor(max_workers=max(1, jobs)) as ex:
    outs = list(ex.map(replay, jobs_list))       # in the order of jobs_list: one recording may be given twice, with other settings

bad = 0
for k, (base, sets) in enumerate(parsed):
    rec = events(open(base + ".log", errors="replace").read())
    results = []      # per replay: (label, same or clean, what was found, its NAV lines, its output)
    for m, q in enumerate([0] + jpeg):
        out = outs[k * (1 + len(jpeg)) + m]
        rpl = events(out)
        label = "jpeg %d" % q if q else "as recorded"
        if not any(e[1] == "phase" for e in rpl):
            results.append((label, False, "the replay did not run: " + (out.strip().splitlines() or ["no output"])[-1][:160], rpl, out))
            continue
        first, new = compare(rec, rpl)
        trouble = ["\"%s\" x%d" % (w, sum(1 for e in rpl if e[1] == "trouble" and e[2] == w)) for w in TROUBLE if any(e[1] == "trouble" and e[2] == w for e in rpl)]
        if clean:
            ok = not trouble and in_order(rpl)
            what = "tags " + tags_text(rpl) + ("; trouble " + ", ".join(trouble) if trouble else "") + ("" if in_order(rpl) else "; tags out of order")
        else:
            ok = first is None and not new
            what = "tags " + tags_text(rpl) + (" (flight " + tags_text(rec) + ")" if not q else "")
            if first: what += "; first difference " + first
            if new: what += "; trouble the flight did not have: " + ", ".join(new)
        angle = re.search(r"replay: the tags lie (\d+) cm from where the flight saw them", out)
        if angle: what += "; CAMERA ANGLE NOT THE FLIGHT'S (tags %s cm off): see the replay's own warning" % angle.group(1)
        results.append((label, ok, what, rpl, out))
    n_ok = sum(1 for r in results if r[1])
    verdict = ("CLEAN" if clean else "SAME") if n_ok == len(results) else "FRAGILE" if n_ok else ("TROUBLE" if clean else "DIFFERENT")
    if n_ok != len(results): bad += 1
    line = "%-22s %-9s " % (Path(base).name, verdict)
    if len(results) > 1:      # the replay of the frames as they are in full, the others as yes / no (-v shows them all)
        line += "%d of %d replays %s (%s); as recorded: " % (n_ok, len(results), "clean" if clean else "same",
                                                             ", ".join("%s %s" % (r[0], "yes" if r[1] else "no") for r in results))
    print(line + results[0][2])
    if verbose:
        print("      flight:")
        for e in rec:
            if e[1] != "note": print("        t=%6.2f  %s" % (e[0], e[3]))
        for label, ok, what, rpl, out in results:
            print("      replay, %s:" % label)
            for e in rpl:
                if e[1] != "note": print("        t=%6.2f  %s" % (e[0], e[3]))
            for ln in out.splitlines():
                if "] replay: sticks" in ln or "] replay: tags" in ln: print("        " + ln.split("] ", 1)[1])
sys.exit(1 if bad else 0)
