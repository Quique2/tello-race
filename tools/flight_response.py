"""How does the real drone answer the sticks? From a recorded flight: tags seen in every frame give the true motion,
the csv gives the sticks that were sent.

  uv run --with opencv-python-headless --with numpy --with scipy python tools/flight_response.py logs/run_YYYYMMDD_HHMMSS [tag_size_m]

Model per axis:  v' = (K (u(t - d) - u0) - v) / tau,   tag as seen = (T - P) + c H v'/g   (the lean moves the view)
"""
import csv, sys, json, os
import cv2, numpy as np
from scipy.optimize import least_squares

pre = sys.argv[1]
TAG = float(sys.argv[2]) if len(sys.argv) > 2 else 0.16
rows = list(csv.DictReader(open(pre + ".csv")))
n = len(rows)
seq = np.array([int(r["seq"]) for r in rows], float)
span = float(rows[-1]["t"]) - float(rows[0]["t"])
fps = (seq[-1] - seq[0]) / span
t = (seq - seq[0]) / fps
u = {"x": np.array([float(r["rc_fb"]) for r in rows]), "y": -np.array([float(r["rc_lr"]) for r in rows])}  # y = left
phase = [r["phase"] for r in rows]
print(f"{n} frames, delivered {fps:.2f} fps, {span:.1f} s")

cache = pre + ".tags.json"
if os.path.exists(cache):
    det = json.load(open(cache))
else:
    cap = cv2.VideoCapture(pre + ".avi")
    d = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), cv2.aruco.DetectorParameters())
    det = []
    for i in range(n):
        ok, f = cap.read()
        if not ok:
            break
        cs, ids, _ = d.detectMarkers(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY))
        fr = {}
        if ids is not None:
            for c, k in zip(cs, ids.ravel()):
                c = c.reshape(4, 2)
                side = float(np.mean([np.linalg.norm(c[j] - c[(j + 1) % 4]) for j in range(4)]))
                fr[str(int(k))] = [float(c[:, 0].mean()), float(c[:, 1].mean()), side]
        det.append(fr)
    json.dump(det, open(cache, "w"))
W, Hh = 960, 720
seen = sum(1 for f in det if f)
print(f"tags found in {seen} of {len(det)} frames;", {k: sum(1 for f in det if k in f) for k in "1234"})

# tracks: tag k seen in consecutive frames (holes up to 3 frames) -> metric position of the tag in the camera's floor frame
tracks = []
for k in "1234":
    cur = []
    for i, f in enumerate(det):
        if k in f:
            cx, cy, side = f[k]
            s = TAG / side
            if cur and i - cur[-1][0] > 3:
                tracks.append((k, cur)); cur = []
            cur.append((i, (Hh / 2 - cy) * s, (W / 2 - cx) * s, s * 920.0))
    if cur:
        tracks.append((k, cur))
tracks = [(k, np.array(c)) for k, c in tracks if len(c) >= 12]
for k, c in tracks:
    print(f"  tag {k}: frames {int(c[0, 0])}..{int(c[-1, 0])} ({len(c)}), t {t[int(c[0, 0])]:.1f}..{t[int(c[-1, 0])]:.1f} s, height {c[:, 3].mean():.2f} m")

G = 9.81
i_air = next(i for i, p in enumerate(phase) if p not in ("IDLE", "FIND_LINE", "TAKEOFF"))
dt_sim = 0.01
ts = np.arange(t[i_air], t[-1] + 0.5, dt_sim)


def simulate(ax, K, tau, d, u0):
    uu = np.interp(ts - d, t, u[ax])
    v = np.zeros_like(ts); a = np.zeros_like(ts)
    for j in range(1, len(ts)):
        a[j - 1] = (K * (uu[j - 1] - u0) - v[j - 1]) / tau
        v[j] = v[j - 1] + a[j - 1] * dt_sim
    a[-1] = a[-2]
    return np.cumsum(v) * dt_sim, v, a


def resid(par, ax, lag2=False):
    K, tau, d, u0, c = par
    P, v, a = simulate(ax, K, tau, d, u0)
    out = []
    for k, tr in tracks:
        idx = tr[:, 0].astype(int)
        m = tr[:, 1] if ax == "x" else tr[:, 2]
        tt = t[idx]
        ok = tt >= ts[0]
        if ok.sum() < 8:
            continue
        r = m[ok] + np.interp(tt[ok], ts, P) - c * tr[ok, 3] * np.interp(tt[ok], ts, a) / G
        out.append(r - r.mean())
    return np.concatenate(out)


res = {}
for ax in "xy":
    best = None
    for tau0 in (0.3, 0.6, 1.0):
        for d0 in (0.1, 0.3):
            f = least_squares(resid, [0.02, tau0, d0, 0.0, 1.0], args=(ax,),
                              bounds=([0.003, 0.05, 0.0, -15, -1.0], [0.08, 3.0, 0.8, 15, 4.0]))
            if best is None or f.cost < best.cost:
                best = f
    K, tau, d, u0, c = best.x
    rms = float(np.sqrt(np.mean(best.fun ** 2)))
    res[ax] = best.x
    print(f"axis {ax}: gain {K:.4f} m/s per unit, tau {tau:.2f} s, delay {d:.2f} s, stick offset {u0:+.1f} units "
          f"(drift {-K * u0:+.3f} m/s), lean coefficient {c:.2f}, residual {rms * 100:.1f} cm rms")
    # how much worse are the values the controller believes?
    for name, par in (("as flown (0.0112, tau 0.5, 0.13 s, no lean)", [0.0112, 0.5, 0.13, 0, 0.0]),
                      ("same with the fitted gain", [K, 0.5, 0.13, 0, 0.0]),
                      ("fitted, lean off", [K, tau, d, u0, 0.0])):
        r = resid(par, ax)
        print(f"     {name}: {np.sqrt(np.mean(r ** 2)) * 100:.1f} cm rms")
json.dump({a: list(map(float, v)) for a, v in res.items()}, open(pre + ".sysid.json", "w"))
