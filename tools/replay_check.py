"""Does this build decide on the frames of a recorded flight what the flight itself decided?

  python tools/replay_check.py [--exe tello_race.exe] [--profile config/course_tec.ini] [--tol 0.3] [--replays 8] [--clean] [--tags N] [--strict] [--jobs 3] [-v] RECORDING [RECORDING ...]

Every RECORDING is "path/run_YYYYMMDD_HHMMSS: section.key=value section.key=value ...": the three files the ground
station's RECORD wrote (.avi .csv .log, named without the extension) and the settings that flight STARTED with (the
nose mode, and what the learned file held then: course.rc_gain_ms and the ground.* flags; the camera angle is read
off the recording itself). Each one is replayed offline (tello_race replay --telemetry) and the NAV lines of the
replay are compared with the ones in the recorded .log.

One replay is not enough. The file holds the frames as the recorder's MJPG left them, with its block grid in them,
not the frames the flight saw, and where a colour sits on a limit of the tape mask the mask follows the blocks (a
piece of "tape" traced inside a tag, a tape end that grows a hook). So every RECORDING is replayed --replays + 1
times: once with the frames as recorded, and --replays times through one more JPEG generation each on a block grid
moved by another number of px (replay.jpeg_shift: the recorder's grid goes, a far weaker one comes, elsewhere). None
of them shows the flight's own pixels. What more than half of them decide is taken as what the build decides on
this flight; the line says how many did. One line per RECORDING:

  SAME       more than half of the replays reach the same tags in the same order, each within --tol seconds of the
             flight's time, with the same phase changes in between, every "next line starts D m" within 5 cm, and
             no trouble the flight did not have
  DIFFERENT  half of them or more do not; the line says where one of those first parts from the flight

Trouble is a NAV line with "flown before", "leads back", "lost" or "not reached" in it, or the navigator giving up
(a change to ABORTED for any reason but a landing that was asked for).

--clean is for a flight that went wrong, or was flown by other code: the replay is not held against the log. It has
to reach its tags with rising IDs, at least as many as the flight reached (--tags N: at least N), without trouble
and without giving up (CLEAN / TROUBLE, by the same majority).
--strict wants every replay to pass, not more than half.

The replay is open-loop (the video shows what the real drone did, whatever is commanded now): past the first
difference the rest of the line says little.

Exit code 0 when every RECORDING is SAME (CLEAN with --clean), 1 otherwise.

Example, the flight of 2026-10-12 18:34:
  python tools/replay_check.py "logs/run_20261012_183452: course.heading=path course.rc_gain_ms=0.0139 ground.mount_verified=1 ground.imu_signs_verified=1 ground.imu_pitch_sign=1 ground.imu_roll_sign=1"

The executable is taken from --exe, else TELLO_RACE_EXE, else C:/dev/build/tello-race/tello_race.exe, else build/tello_race.
"""
import os, re, subprocess, sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
TROUBLE = ("flown before", "leads back", "lost", "not reached")
GAVE_UP = "gave up"
# Grid shifts in the order they are used: odd ones first (neither the 8 px brightness blocks nor the 16 px colour
# blocks stay where they were), never 8 (the brightness blocks would).
SHIFTS = (3, 5, 11, 13, 1, 7, 9, 15, 2, 6, 10, 14, 4, 12)

argv = sys.argv[1:]
exe, profile, tol, replays, clean, tags_min, strict, jobs, verbose = os.environ.get("TELLO_RACE_EXE", ""), str(REPO / "config" / "course_tec.ini"), 0.3, 8, False, None, False, 3, False
recordings = []
i = 0
while i < len(argv):
    a = argv[i]
    if a == "--exe": exe = argv[i + 1]; i += 2
    elif a == "--profile": profile = argv[i + 1]; i += 2
    elif a == "--tol": tol = float(argv[i + 1]); i += 2
    elif a == "--replays": replays = max(0, min(len(SHIFTS), int(argv[i + 1]))); i += 2
    elif a == "--tags": tags_min = int(argv[i + 1]); i += 2
    elif a == "--jobs": jobs = int(argv[i + 1]); i += 2
    elif a == "--clean": clean = True; i += 1
    elif a == "--strict": strict = True; i += 1
    elif a == "-v": verbose = True; i += 1
    elif a.startswith("--"): sys.exit("replay_check: unknown option %s" % a)
    else: recordings.append(a); i += 1
if not recordings:
    sys.exit(__doc__)
EXE = next((p for p in (exe, "C:/dev/build/tello-race/tello_race.exe", str(REPO / "build" / "tello_race"),
                        str(REPO / "build" / "tello_race.exe")) if p and Path(p).exists()), None)
if not EXE:
    sys.exit("tello_race executable not found: give --exe or set TELLO_RACE_EXE")
# an executable built before the flight replay existed would play the video without the telemetry and say nothing
usage = subprocess.run([EXE], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, errors="replace").stdout
if "--telemetry" not in usage:
    sys.exit("replay_check: %s has no flight replay (no --telemetry in its usage): rebuild it, or give another with --exe" % EXE)

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
        elif ph:
            out.append((t, "phase", (ph.group(1), ph.group(2)), body))
            # the navigator ending the flight by itself (the operator's landing, and the replay's own at the end of
            # the recording, say "land requested")
            if ph.group(2) == "ABORTED" and "land requested" not in body: out.append((t, "trouble", GAVE_UP, body))
        elif nx: out.append((t, "next", float(nx.group(1)), body))
        elif any(w in body for w in TROUBLE): out.append((t, "trouble", next(w for w in TROUBLE if w in body), body))
        else: out.append((t, "note", None, body))
    return out


def steps(ev):
    """What is compared one by one: tags, next lines and the phase changes of the flight itself (not the start and the end)."""
    return [e for e in ev if e[1] in ("tag", "next") or (e[1] == "phase" and e[2][1] not in ("TAKEOFF", "LANDED", "ABORTED"))]


def troubles(ev):
    """How often each kind of trouble is in a log."""
    return {w: sum(1 for e in ev if e[1] == "trouble" and e[2] == w) for w in TROUBLE + (GAVE_UP,)}


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
    nr, nf = troubles(rpl), troubles(rec)
    new = ["\"%s\" x%d" % (w, nr[w]) + (" (flight x%d)" % nf[w] if nf[w] else "") for w in nr if nr[w] > nf[w]]
    return first, new


def in_order(ev):
    """Tags reached with rising IDs, none skipped over (the taped course has them in that order)."""
    ids = [e[2] for e in ev if e[1] == "tag"]
    return all(q > p for p, q in zip(ids, ids[1:])) and not any("where ID" in e[3] for e in ev)


def replay(job):
    base, sets, shift = job
    cmd = [EXE, "replay", "--headless", "--no-realtime", "--config", str(REPO / "config" / "default.ini"), "--profile", profile,
           "--source", "file:" + base + ".avi", "--telemetry", base + ".csv", "--log-level", "debug"]
    for s in sets + (["replay.jpeg_shift=%d" % shift] if shift else []):
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
shifts = [0] + list(SHIFTS[:replays])
jobs_list = [(base, sets, k) for base, sets in parsed for k in shifts]
with ThreadPoolExecutor(max_workers=max(1, jobs)) as ex:
    outs = list(ex.map(replay, jobs_list))       # in the order of jobs_list: one recording may be given twice, with other settings

bad = 0
for k, (base, sets) in enumerate(parsed):
    rec = events(open(base + ".log", errors="replace").read())
    need_tags = tags_min if tags_min is not None else sum(1 for e in rec if e[1] == "tag")
    results = []      # per replay: (label, same or clean, what was found, its NAV lines, its output)
    for m, shift in enumerate(shifts):
        out = outs[k * len(shifts) + m]
        rpl = events(out)
        label = "grid moved %d px" % shift if shift else "as recorded"
        if not any(e[1] == "phase" for e in rpl):
            results.append((label, False, "the replay did not run: " + (out.strip().splitlines() or ["no output"])[-1][:160], rpl, out))
            continue
        if "replay: the video ends before the csv" in out:
            results.append((label, False, "the video ends before the csv: not all of the flight was replayed", rpl, out))
            continue
        first, new = compare(rec, rpl)
        if clean:
            found = troubles(rpl)
            trouble = ["\"%s\" x%d" % (w, found[w]) for w in found if found[w]]
            n_tags = sum(1 for e in rpl if e[1] == "tag")
            ok = not trouble and in_order(rpl) and n_tags >= need_tags
            what = "tags " + tags_text(rpl) + ("; trouble " + ", ".join(trouble) if trouble else "") + ("" if in_order(rpl) else "; tags out of order")
            if n_tags < need_tags: what += "; %d of the %d tags wanted" % (n_tags, need_tags)
        else:
            ok = first is None and not new
            what = "tags " + tags_text(rpl)
            if first: what += "; first difference " + first
            if new: what += "; trouble the flight did not have: " + ", ".join(new)
        # with replay.follow_camera=0 only: the replay says so itself when the configured angle is not the flight's
        angle = re.search(r"replay: the tags lie (\d+) cm from where the flight saw them", out)
        if angle: what += "; CAMERA ANGLE NOT THE FLIGHT'S (tags %s cm off): see the replay's own warning" % angle.group(1)
        results.append((label, ok, what, rpl, out))
    n_ok = sum(1 for r in results if r[1])
    passed = n_ok == len(results) if strict else 2 * n_ok > len(results)
    if not passed: bad += 1
    line = "%-22s %-9s %d of %d replays %s" % (Path(base).name, ("CLEAN" if clean else "SAME") if passed else ("TROUBLE" if clean else "DIFFERENT"),
                                              n_ok, len(results), "clean" if clean else "the same as the flight")
    failed = [r for r in results if not r[1]]
    if failed and n_ok: line += " (not: %s)" % ", ".join(r[0] for r in failed)
    line += "; flight tags " + tags_text(rec)
    # one replay in full: one that did not pass if there is any (-v shows them all)
    shown = failed[0] if failed else results[0]
    print(line + "; %s: %s" % (shown[0], shown[2]))
    if verbose:
        print("      flight:")
        for e in rec:
            if e[1] != "note" and e[2] != GAVE_UP: print("        t=%6.2f  %s" % (e[0], e[3]))
        for label, ok, what, rpl, out in results:
            print("      replay, %s: %s" % (label, ("clean" if clean else "same") if ok else what))
            for e in rpl:
                if e[1] != "note" and e[2] != GAVE_UP: print("        t=%6.2f  %s" % (e[0], e[3]))
            for ln in out.splitlines():
                if "] replay: sticks" in ln or "] replay: tags" in ln or "] replay: camera" in ln: print("        " + ln.split("] ", 1)[1][:400])
sys.exit(1 if bad else 0)
