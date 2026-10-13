"""What does the real camera show that the simulator's does not? The same measurements on any recording, real
(logs/run_*.avi from the ground station) or simulated (tello_race sim --record --log-dir DIR ...), so that the two can
be put side by side. The [sim] values in config/course_tec.ini come from these numbers (docs/COURSE_TEC.md).

  uv run --with opencv-python-headless --with numpy python tools/camera_vs_sim.py WHAT VIDEO [...]

  tint   VIDEO [STEP]                the black of the tags: its colour, how blue against green varies over a tag, whether
                                     that pattern stays put on the tag from frame to frame, tag by tag
  share  VIDEO [STEP] [OUT.jpg]      how much of each tag the blue-tape mask takes (OUT.jpg: the worst cases)
  layout VIDEO FRAME TAG OUT.png     the floor around a tag from straight above, in the tag's own coordinates: where
                                     tapes begin and end relative to it (a grid line every 10 cm)
  haze   VIDEO TAG AXIS FRAME...     colour along a tape's axis from the tape into the tag it ends at (AXIS = the
                                     half-axis of the tag the tape lies on: up | right | down | left, see layout)
  noise  VIDEO FIRST LAST STEP       picture noise on plain tarp, pixel by pixel and in blocks; the dark band along the
                                     bottom of the picture (the mirror's lower edge)
  floor  VIDEO FRAME...              brightness of the floor beyond the tarp relative to the tarp
  glare  RUN TAG FIRST LAST STEP PITCH ROLL [OUT.jpg]
                                     the stretch of tape washed pale by a lamp: where it lies relative to the tag and to
                                     the camera, frame by frame, and the lamp positions that explain how it moved
                                     (RUN = recording without extension, its .csv gives the height; PITCH ROLL = the
                                     mirror angles of that flight)
"""
import csv, math, sys
import cv2, numpy as np

BLUE_LO, BLUE_HI = (95, 80, 40), (132, 255, 255)                       # [line] of config/course_tec.ini
WHITE_S_MAX, WHITE_V_MIN, TAPE_S_MIN, TAPE_S_MARGIN = 70, 150, 34, 22
DET = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), cv2.aruco.DetectorParameters())


def tape_mask(bgr):
    """The blue-tape mask as src/vision/LineDetector.cpp builds it in course mode (half size, limits following the white)."""
    proc = cv2.resize(bgr, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(proc, cv2.COLOR_BGR2HSV)
    sub = hsv[::4, ::4].reshape(-1, 3)
    vmin, expo, v95 = WHITE_V_MIN, 1.0, 0
    w = sub[sub[:, 1] <= WHITE_S_MAX]
    if len(w) > 200:
        v95 = int(np.sort(w[:, 2])[::-1][len(w) // 20])
        vmin = min(vmin, max(60, round(0.72 * v95)))
        expo = min(1.0, v95 / 210.0)
    slo = -1
    w2 = sub[(sub[:, 2] >= vmin) & (sub[:, 1] <= WHITE_S_MAX)]
    if len(w2) > 200:
        slo = max(TAPE_S_MIN, int(np.median(w2[:, 1])) + TAPE_S_MARGIN)
    lo = (BLUE_LO[0], min(BLUE_LO[1], slo) if slo >= 0 else BLUE_LO[1], max(12, BLUE_LO[2] * expo))
    raw = cv2.inRange(hsv, lo, BLUE_HI)
    white = cv2.inRange(hsv, (0, 0, vmin), (180, WHITE_S_MAX, 255))
    white = cv2.dilate(white, cv2.getStructuringElement(cv2.MORPH_RECT, (55, 55)))      # 6 cm at 1.2 m
    mask = raw & white
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))
    return proc, hsv, mask, v95, lo


def frames(video, step=1, first=0, last=10 ** 9):
    cap = cv2.VideoCapture(video)
    if first: cap.set(cv2.CAP_PROP_POS_FRAMES, first)
    fi = first
    while fi <= last:
        ok, f = cap.read()
        if not ok: break
        if (fi - first) % step == 0: yield fi, f
        fi += 1


def frame(video, fi):
    cap = cv2.VideoCapture(video)
    cap.set(cv2.CAP_PROP_POS_FRAMES, fi)
    ok, f = cap.read()
    if not ok: sys.exit("no frame %d in %s" % (fi, video))
    return f


def tags(f):
    corners, ids, _ = DET.detectMarkers(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY))
    return [] if ids is None else [(int(i), c[0]) for c, i in zip(corners, ids.flatten())]


def onto_tag(f, q, n, margin=0):
    """The picture warped so that the tag's black square fills n x n pixels (plus a margin all round)."""
    dst = np.float32([[margin, margin], [n + margin, margin], [n + margin, n + margin], [margin, n + margin]])
    return cv2.warpPerspective(f, cv2.getPerspectiveTransform(q.astype(np.float32), dst), (n + 2 * margin, n + 2 * margin), flags=cv2.INTER_LINEAR)


def cmd_tint(video, step=1, maxmove=12.0):
    N = 128                                         # 1 px = 1.25 mm on the tag
    prev, sight = {}, []
    for fi, f in frames(video, step):
        for i, q in tags(f):
            if q.min() < 12 or q[:, 0].max() > f.shape[1] - 12 or q[:, 1].max() > f.shape[0] - 12: continue
            cen = q.mean(0)
            mv = np.linalg.norm(cen - prev[i][1]) / max(1, fi - prev[i][0]) if i in prev else 99
            prev[i] = (fi, cen)
            if mv > maxmove: continue               # sharp sightings only
            w = onto_tag(f, q, N).astype(np.float32)
            w2 = onto_tag(f, q, N, 16).astype(np.float32)
            paper = np.median(np.concatenate([w2[2:8].reshape(-1, 3), w2[-8:-2].reshape(-1, 3)]), 0)
            g = w.mean(2)
            dark = g < 0.5 * (np.percentile(g, 5) + np.percentile(g, 95))
            dark = cv2.erode(dark.astype(np.uint8), np.ones((7, 7), np.uint8)) > 0      # away from the cells' edges
            if dark.sum() < 800: continue
            sight.append((fi, i, w[..., 0] - w[..., 1], dark, w[dark].mean(0), paper))
    if not sight: sys.exit("no sharp tag sightings")
    bgr = np.array([s[4] for s in sight]); pap = np.array([s[5] for s in sight])
    d = bgr[:, 0] - bgr[:, 1]
    print("%d sharp sightings (tag moving less than %.0f px per frame)" % (len(sight), maxmove))
    print("  dark cells B G R %s, paper %s: dark / paper %.2f, dark B-R %.1f" % ([round(float(x), 1) for x in bgr.mean(0)], [round(float(x), 1) for x in pap.mean(0)],
                                                                                bgr.mean() / pap.mean(), (bgr[:, 0] - bgr[:, 2]).mean()))
    print("  B-G of a whole tag: mean %+.2f, std between sightings %.2f" % (d.mean(), d.std()))
    for i in sorted(set(s[1] for s in sight)):
        sel = np.array([k for k, s in enumerate(sight) if s[1] == i])
        print("     tag %d: %3d sightings, B-G %+.2f (std %.2f), dark V %.0f" % (i, len(sel), d[sel].mean(), d[sel].std(), bgr[sel].max(1).mean()))
    print("  B-G within one tag's dark cells: std %.2f (median over the sightings)" % np.median([s[2][s[3]].std() for s in sight]))
    print("  rms difference of B-G between two points of the dark cells, by their distance on the tag:")
    for dist in (2, 4, 8, 16, 32, 64):
        acc = []
        for s in sight[::max(1, len(sight) // 40)]:
            D, m = s[2], s[3]
            for dx, dy in ((dist, 0), (0, dist)):
                a, b = D[:N - dy, :N - dx], D[dy:, dx:]
                mm = m[:N - dy, :N - dx] & m[dy:, dx:]
                if mm.sum() > 100: acc.append(np.sqrt(np.mean((a[mm] - b[mm]) ** 2) / 2))
        if acc: print("     %4.1f cm: %.2f" % (dist * 0.125, np.median(acc)))
    print("  correlation of the B-G pattern between sightings of the same tag n frames apart (does it stay put on the tag?):")
    for n in (1, 2, 4, 8, 16):
        cc = []
        for a in range(len(sight)):
            for b in range(a + 1, min(len(sight), a + 40)):
                if sight[b][1] == sight[a][1] and sight[b][0] - sight[a][0] == n:
                    m = sight[a][3] & sight[b][3]
                    if m.sum() > 500: cc.append(np.corrcoef(cv2.blur(sight[a][2], (9, 9))[m], cv2.blur(sight[b][2], (9, 9))[m])[0, 1])
        if cc: print("     n = %2d: %.2f  (%d pairs)" % (n, np.median(cc), len(cc)))


def cmd_share(video, step=2, out=None):
    rows, crops, prev = [], [], None
    for fi, f in frames(video, step):
        found = tags(f)
        if not found: continue
        proc, hsv, mask, v95, lo = tape_mask(f)
        for i, q in found:
            h = (q * 0.5).astype(np.int32)
            if h.min() < 4 or h[:, 0].max() > proc.shape[1] - 4 or h[:, 1].max() > proc.shape[0] - 4: continue     # whole tag in the picture
            tm = np.zeros(mask.shape, np.uint8)
            cv2.fillConvexPoly(tm, h, 255)
            inside = hsv[tm > 0]
            dk = inside[:, 2] < 0.55 * v95
            share = 100.0 * np.count_nonzero(mask[tm > 0]) / max(1, len(inside))
            cen = q.mean(0)
            mv = np.linalg.norm(cen - prev[1]) / max(1, fi - prev[0]) if prev is not None and prev[2] == i else -1
            prev = (fi, cen, i)
            hd = np.median(inside[dk], 0).astype(int)
            rows.append((fi, i, cen[0], cen[1], share, hd[0], hd[1], hd[2], mv))
            x0, y0 = max(0, int(cen[0] / 2 - 60)), max(0, int(cen[1] / 2 - 60))
            cr, cm = proc[y0:y0 + 120, x0:x0 + 120], cv2.cvtColor(mask[y0:y0 + 120, x0:x0 + 120], cv2.COLOR_GRAY2BGR)
            if out and cr.shape[:2] == (120, 120): crops.append((share, fi, np.hstack([cr, cm])))
    print("frame id   u    v  share%  dark cells H   S   V   moved px/frame")
    for r in rows: print("%5d %2d %4.0f %4.0f %6.1f              %3d %3d %3d  %5.1f" % r)
    if len(rows) > 5:
        sh = np.array([r[4] for r in rows])
        print("share of a tag taken for blue tape: none in %d %% of the sightings; median %.1f %%, p75 %.1f, p90 %.1f, max %.1f" % (
            100 * np.mean(sh < 0.5), np.median(sh), np.percentile(sh, 75), np.percentile(sh, 90), sh.max()))
    if out and crops:
        crops.sort(key=lambda c: -c[0])
        tiles = []
        for share, fi, im in crops[:12] + crops[len(crops) // 2 - 2:len(crops) // 2 + 2]:
            im = cv2.resize(im, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
            cv2.putText(im, "%d: %.0f%%" % (fi, share), (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            tiles.append(im)
        while len(tiles) % 4: tiles.append(np.zeros_like(tiles[0]))
        cv2.imwrite(out, np.vstack([np.hstack(tiles[k:k + 4]) for k in range(0, len(tiles), 4)]), [cv2.IMWRITE_JPEG_QUALITY, 85])


def cmd_layout(video, fi, tid, out, half=0.5):
    f = frame(video, fi)
    q = dict(tags(f)).get(tid)
    if q is None: sys.exit("tag %d not in frame %d" % (tid, fi))
    ppm = 400.0
    N, s = int(2 * half * ppm), 0.08 * ppm
    c = N / 2
    dst = np.float32([[c - s, c - s], [c + s, c - s], [c + s, c + s], [c - s, c + s]])
    w = cv2.warpPerspective(f, cv2.getPerspectiveTransform(q.astype(np.float32), dst), (N, N), flags=cv2.INTER_LINEAR)
    hsv = cv2.cvtColor(w, cv2.COLOR_BGR2HSV)
    for name, m in (("blue", cv2.inRange(hsv, (100, 70, 30), (130, 255, 255))), ("yellow", cv2.inRange(hsv, (16, 80, 80), (38, 255, 255)))):
        for ax, (dx, dy) in (("up", (0, -1)), ("right", (1, 0)), ("down", (0, 1)), ("left", (-1, 0))):
            hits = []
            for d in range(int(0.09 * ppm), int(half * ppm) - 5):
                x, y = int(c + dx * d), int(c + dy * d)
                hits.append(m[y - 4:y + 5, x - 4:x + 5].mean() > 128)
            runs, start = [], None
            for k, h in enumerate(hits + [False]):
                if h and start is None: start = k
                if not h and start is not None:
                    if k - start >= 6: runs.append("%.0f-%.0f" % ((start + 0.09 * ppm) / ppm * 100, (k + 0.09 * ppm) / ppm * 100))
                    start = None
            if runs: print("  %-6s on the half-axis %-5s of the tag: %s cm from its centre" % (name, ax, ", ".join(runs)))
    for k in range(-int(half * 10), int(half * 10) + 1):
        p = int(c + k * 0.1 * ppm)
        cv2.line(w, (p, 0), (p, N - 1), (0, 255, 0) if k == 0 else (160, 160, 160), 1)
        cv2.line(w, (0, p), (N - 1, p), (0, 255, 0) if k == 0 else (160, 160, 160), 1)
    cv2.imwrite(out, w)


def cmd_haze(video, tid, axis, fis):
    k = {"up": 0, "right": 1, "down": 2, "left": 3}[axis]
    ppm, N = 800.0, 400
    c, s = N / 2, 0.08 * ppm
    dst = np.float32([[c - s, c - s], [c + s, c - s], [c + s, c + s], [c - s, c + s]])
    for fi in fis:
        f = frame(video, fi)
        q = dict(tags(f)).get(tid)
        if q is None: print(fi, "no tag"); continue
        w = np.ascontiguousarray(np.rot90(cv2.warpPerspective(f, cv2.getPerspectiveTransform(q.astype(np.float32), dst), (N, N), flags=cv2.INTER_LINEAR), k))
        hsv = cv2.cvtColor(w, cv2.COLOR_BGR2HSV)                      # the tape's axis now points up
        rows = []
        for d in np.arange(0.16, 0.045, -0.005):
            y = int(c - d * ppm)
            hh = hsv[y - 2:y + 2, int(c - 0.01 * ppm):int(c + 0.01 * ppm)].reshape(-1, 3)
            take = np.mean((hh[:, 0] >= 95) & (hh[:, 0] <= 132) & (hh[:, 1] >= TAPE_S_MIN) & (hh[:, 2] >= 31))
            rows.append((d, np.median(hh, 0), take))
        print("%5d cm from centre " % fi + " ".join("%4.1f" % (d * 100) for d, *_ in rows))
        print("      V              " + " ".join("%4d" % h[2] for _, h, _ in rows))
        print("      S              " + " ".join("%4d" % h[1] for _, h, _ in rows))
        print("      taken blue %   " + " ".join("%4d" % (100 * t) for *_, t in rows))


def cmd_noise(video, first, last, step):
    prof, noise = [], {k: [] for k in (1, 2, 4, 8, 16)}
    for fi, f in frames(video, step, first, last):
        hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)
        V = hsv[..., 2].astype(float)
        ref = np.median(V[200:420])                                   # the tarp's brightness in the middle of the picture
        prof.append([[np.median(V[r, c]) / ref for r in range(480, 720, 8)] for c in (slice(40, 320), slice(340, 620), slice(640, 920))])
        best, bp = 1e9, None                                          # the plainest 64 x 64 patch of tarp
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY).astype(np.float32)
        for y in range(40, 420, 32):
            for x in range(40, 860, 32):
                p = g[y:y + 64, x:x + 64]
                if hsv[y:y + 64, x:x + 64, 1].max() > 45 or p.min() < 0.8 * ref: continue
                sd = cv2.blur(p, (16, 16)).std()
                if sd < best: best, bp = sd, (y, x)
        if bp is None: continue
        patch = f[bp[0]:bp[0] + 64, bp[1]:bp[1] + 64].astype(np.float32)
        patch -= cv2.blur(patch, (33, 33))                            # the slow shading of the tarp taken out
        for k in noise:
            q = patch.reshape(64 // k, k, 64 // k, k, 3).mean((1, 3))
            noise[k].append([q[..., 0].std(), q[..., 1].std(), q[..., 2].std(), (q[..., 0] - q[..., 1]).std()])
    prof = np.median(np.array(prof), 0)
    print("brightness of the rows 480..712 (every 8th) relative to the tarp, left / middle / right third of the picture:")
    for name, p in zip(("left ", "mid  ", "right"), prof):
        rows = np.arange(480, 720, 8)
        print("  %s %s   75 %% at row %d" % (name, " ".join("%.2f" % x for x in p), rows[np.argmax(p < 0.75)] if (p < 0.75).any() else -1))
    print("noise on plain tarp (std, levels), averaged over blocks of n x n pixels (white noise halves at every step):")
    print("     n     B     G     R    B-G")
    for k in noise:
        if noise[k]: print("   %3d  " % k + "  ".join("%4.2f" % x for x in np.median(np.array(noise[k]), 0)))


def cmd_floor(video, fis):
    for fi in fis:
        f = frame(video, fi)
        hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)[:560]                # not the bottom rows: the mirror's edge
        lowsat = hsv[hsv[..., 1] <= WHITE_S_MAX]
        v95 = np.sort(lowsat[:, 2])[::-1][len(lowsat) // 20]
        dark = cv2.erode((hsv[..., 2] < 0.6 * v95).astype(np.uint8), np.ones((41, 41), np.uint8)) > 0      # large dark areas: not tags, not tape
        if dark.sum() < 3000: print(fi, "no floor in sight"); continue
        hv = hsv[dark]
        print("%5d: white level %3d; floor %4.1f %% of the picture, V median %3d = %.2f of the white, S median %d, H median %d" % (
            fi, v95, 100.0 * dark.mean(), np.median(hv[:, 2]), np.median(hv[:, 2]) / v95, np.median(hv[:, 1]), np.median(hv[:, 0])))


def cmd_glare(run, tid, first, last, step, pitch, roll, out=None):
    FX = FY = 920.0; CX, CY = 480.0, 360.0
    th, ph = math.radians(pitch), math.radians(roll)                  # the floor model of src/vision/CameraModel.cpp
    Ry = np.array([[math.cos(th), 0, math.sin(th)], [0, 1, 0], [-math.sin(th), 0, math.cos(th)]])
    Rx = np.array([[1, 0, 0], [0, math.cos(ph), -math.sin(ph)], [0, math.sin(ph), math.cos(ph)]])
    R = Rx @ Ry @ np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])

    def ground(u, v, h):                                              # pixels -> floor, drone axes (x ahead, y left)
        d = (R @ np.vstack([(np.asarray(u, float) - CX) / FX, (np.asarray(v, float) - CY) / FY, np.ones(np.size(u))])).T
        return d[:, :2] * (-h / d[:, 2])[:, None]

    tel = list(csv.DictReader(open(run + ".csv")))
    rows, tiles = [], []
    for fi, f in frames(run + ".avi", step, first, last):
        q = dict(tags(f)).get(tid)
        if q is None or fi >= len(tel): continue
        h = float(tel[fi]["tof"]) / 100.0
        gc = ground(q[:, 0], q[:, 1], h)
        T = gc.mean(0)
        ex = gc[1] - gc[0] + gc[2] - gc[3]; ex /= np.linalg.norm(ex)  # the tag's own x axis
        ey = np.array([-ex[1], ex[0]])
        to_tag = lambda p: np.stack([(p - T) @ ex, (p - T) @ ey], -1)
        C = to_tag(np.zeros((1, 2)))[0]
        hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)
        pale = cv2.morphologyEx(cv2.inRange(hsv, (100, 30, 95), (128, 95, 255)), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        n, lab, stats, _ = cv2.connectedComponentsWithStats(pale)
        keep = np.zeros_like(pale)
        for i in range(1, n):
            if stats[i, cv2.CC_STAT_AREA] > 500: keep[lab == i] = 255
        ys, xs = np.nonzero(keep)
        if len(xs) < 500: continue
        G = to_tag(ground(xs, ys, h))
        g = G.mean(0)
        along = (G - g) @ np.linalg.eigh(np.cov((G - g).T))[1][:, 1]
        print("%5d h=%.2f  camera (%+.2f,%+.2f)  washed stretch at (%+.2f,%+.2f), %.2f m long, S median %d, V median %d   [tag coordinates, m]" % (
            fi, h, C[0], C[1], g[0], g[1], np.percentile(along, 97) - np.percentile(along, 3), np.median(hsv[keep > 0][:, 1]), np.median(hsv[keep > 0][:, 2])))
        rows.append((h, C[0], C[1], g[0], g[1]))
        if out:
            t2 = cv2.resize(f, None, fx=0.4, fy=0.4)
            cv2.putText(t2, str(fi), (6, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            tiles.append(np.hstack([t2, cv2.resize(cv2.cvtColor(keep, cv2.COLOR_GRAY2BGR), None, fx=0.4, fy=0.4)]))
    if len(rows) >= 5:
        # The reflection of a lamp at L, hl above the floor, seen from a camera over C at height h lies at
        # G = C + (L - C) h / (h + hl). It only shows where it lies on the tape, so only its place ALONG the tape is known.
        r = np.array(rows)
        h, C, G = r[:, 0], r[:, 1:3], r[:, 3:5]
        t = np.zeros_like(G)
        for i in range(len(G)):
            d = G[min(len(G) - 1, i + 2)] - G[max(0, i - 2)]
            t[i] = d / max(1e-9, np.linalg.norm(d))
        print("lamp height (m)   lamp at (tag coordinates, m)   rms along the tape (cm)   the reflection moves, per metre flown")
        for hl in (0.4, 0.6, 0.8, 1.0, 1.3, 1.6, 2.0, 2.5, 3.0, 4.0):
            k = h / (h + hl)
            L = np.linalg.lstsq(t * k[:, None], np.sum((G - (1 - k)[:, None] * C) * t, 1), rcond=None)[0]
            res = np.sum(((1 - k)[:, None] * C + k[:, None] * L - G) * t, 1)
            print("      %3.1f            (%+.2f, %+.2f)                 %4.1f                   %.2f m" % (hl, L[0], L[1], np.sqrt(np.mean(res ** 2)) * 100, 1 - k.mean()))
    if out and tiles:
        while len(tiles) % 3: tiles.append(np.zeros_like(tiles[0]))
        cv2.imwrite(out, np.vstack([np.hstack(tiles[i:i + 3]) for i in range(0, len(tiles), 3)]), [cv2.IMWRITE_JPEG_QUALITY, 80])


if __name__ == "__main__":
    a = sys.argv[1:]
    if len(a) < 2: sys.exit(__doc__)
    what = a[0]
    if what == "tint": cmd_tint(a[1], int(a[2]) if len(a) > 2 else 1)
    elif what == "share": cmd_share(a[1], int(a[2]) if len(a) > 2 else 2, a[3] if len(a) > 3 else None)
    elif what == "layout": cmd_layout(a[1], int(a[2]), int(a[3]), a[4])
    elif what == "haze": cmd_haze(a[1], int(a[2]), a[3], [int(x) for x in a[4:]])
    elif what == "noise": cmd_noise(a[1], int(a[2]), int(a[3]), int(a[4]))
    elif what == "floor": cmd_floor(a[1], [int(x) for x in a[2:]])
    elif what == "glare": cmd_glare(a[1], int(a[2]), int(a[3]), int(a[4]), int(a[5]), float(a[6]), float(a[7]), a[8] if len(a) > 8 else None)
    else: sys.exit(__doc__)
