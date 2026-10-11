# /// script
# requires-python = ">=3.11"
# dependencies = ["opencv-contrib-python-headless", "numpy", "scipy", "scikit-image"]
# ///
"""Build a simulator course from ONE photo of a real course lying on the floor.

  uv run tools/course_from_photo.py photo.jpg courses/<name> [--tag-size 0.16] [--order 1,2,3,4] [--px-per-m 300] [--photo-tags 1]

The AprilTags (tag36h11) in the photo give the metric ground plane: every tag is a square of known size, so one
homography that makes all of them square maps the photo to a top-down view in metres. The tape colours are
segmented in the photo, the blue centre-lines are traced in the top-down view and chained in tag order
(segment k ends at tag k; the next one is the line whose free end is nearest to that tag).

Writes into the output folder:
  world.png    top-down view of the photo (the simulator floor), PX_PER_M pixels per metre
  course.txt   world size, start pose, segments (polylines), tags, per-tag turn/start offsets
  debug.png    the top-down view with the traced segments and tags drawn on it
"""
import sys, math, os
import numpy as np, cv2
from scipy.optimize import least_squares
from skimage.morphology import skeletonize

argv = sys.argv[1:]
opt, args, i = {}, [], 0
while i < len(argv):
    if argv[i].startswith("--"):
        opt[argv[i][2:]] = argv[i + 1]
        i += 2
    else:
        args.append(argv[i])
        i += 1
photo, outdir = args[0], args[1]
TAG = float(opt.get("tag-size", 0.16))
ORDER = [int(v) for v in opt.get("order", "1,2,3,4").split(",")]
PPM = int(opt.get("px-per-m", 300))
MARGIN = float(opt.get("margin", 1.0))          # free floor around the tarp in the simulator world
KEEP = float(opt.get("photo-margin", 0.40))     # how much real floor around the tarp is taken from the photo
START_IN = float(opt.get("start-in", 0.15))      # the drone starts this far along the first tape (m)
KEEP_TAGS = opt.get("photo-tags", "0") == "1"    # 1: simulate with the tags as photographed instead of exact ones
os.makedirs(outdir, exist_ok=True)
img = cv2.imread(photo)
Hh, Ww = img.shape[:2]

# ---------------------------------------------------------------- tags
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), cv2.aruco.DetectorParameters())
tags = {}
for sc in (1.0, 2.0, 0.5):
    g = gray if sc == 1 else cv2.resize(gray, None, fx=sc, fy=sc, interpolation=cv2.INTER_CUBIC)
    cs, ids, _ = det.detectMarkers(g)
    if ids is not None:
        for c, k in zip(cs, ids.ravel()):
            tags.setdefault(int(k), c[0].astype(np.float64) / sc)
missing = [k for k in ORDER if k not in tags]
if missing:
    sys.exit(f"tags not found in the photo: {missing}")
print("tags found:", sorted(tags))

# ---------------------------------------------------------------- homography image -> ground (metres, y up)
s = TAG / 2
SQ = np.array([[-s, s], [s, s], [s, -s], [-s, -s]])           # TL, TR, BR, BL seen from above, y up


def apply(Hm, p):
    q = np.c_[p, np.ones(len(p))] @ Hm.T
    return q[:, :2] / q[:, 2:3]


ref = max(tags, key=lambda k: cv2.contourArea(tags[k].astype(np.float32)))     # biggest tag: best conditioned start
H0, _ = cv2.findHomography(tags[ref], SQ)


def tag_pose(Hm, c):
    g = apply(Hm, c)
    ctr = g.mean(0)
    th = np.mean([math.atan2(a[0] * b[1] - a[1] * b[0], float(np.dot(a, b))) for a, b in zip(SQ, g - ctr)])
    return ctr[0], ctr[1], th


others = [k for k in sorted(tags) if k != ref]
x0 = list((H0 / H0[2, 2]).ravel()[:8])
for k in others:
    x0 += list(tag_pose(H0, tags[k]))


def resid(x):
    Hm = np.append(x[:8], 1.0).reshape(3, 3)
    Hi = np.linalg.inv(Hm)
    r = [(apply(Hi, SQ) - tags[ref]).ravel()]
    for j, k in enumerate(others):
        cx, cy, th = x[8 + 3 * j: 11 + 3 * j]
        c, sn = math.cos(th), math.sin(th)
        g = SQ @ np.array([[c, sn], [-sn, c]]) + [cx, cy]
        r.append((apply(Hi, g) - tags[k]).ravel())
    return np.concatenate(r)


sol = least_squares(resid, x0, method="lm")
H = np.append(sol.x[:8], 1.0).reshape(3, 3)
print(f"homography fit: reprojection rms {math.sqrt(np.mean(sol.fun ** 2)):.2f} px over {len(tags)} tags")
for k in sorted(tags):
    g = apply(H, tags[k])
    sides = [np.linalg.norm(g[j] - g[(j + 1) % 4]) for j in range(4)]
    print(f"  tag {k}: sides {np.round(np.array(sides) * 100, 1)} cm (expected {TAG * 100:.0f})")

# ---------------------------------------------------------------- colour masks in the photo
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
blue = cv2.inRange(hsv, (100, 90, 40), (130, 255, 255))
yellow = cv2.inRange(hsv, (18, 110, 110), (36, 255, 255))
k5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
blue = cv2.morphologyEx(cv2.morphologyEx(blue, cv2.MORPH_OPEN, k5), cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15)))
yellow = cv2.morphologyEx(cv2.morphologyEx(yellow, cv2.MORPH_OPEN, k5), cv2.MORPH_CLOSE, k5)

# ---------------------------------------------------------------- the tarp: the big clean white sheet the tags lie on
# The terrazzo floor is bright too, but speckled: a strict white test + a wide opening removes it; the tarp is the
# component around the tags. Its outline is a rectangle in the ground frame.
tag_fill = np.zeros((Hh, Ww), np.uint8)
for k in tags:
    c = tags[k].mean(0)
    cv2.fillConvexPoly(tag_fill, np.round(c + (tags[k] - c) * 1.6).astype(np.int32), 255)       # the plate, not only the code
white = cv2.inRange(hsv, (0, 0, 170), (180, 40, 255)) | blue | yellow | tag_fill
white = cv2.morphologyEx(white, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (31, 31)))
n, lab, st, _ = cv2.connectedComponentsWithStats(white)
votes = np.bincount(lab[tag_fill > 0], minlength=n)
votes[0] = 0
tarp = (lab == int(np.argmax(votes))).astype(np.uint8) * 255
tarp = cv2.morphologyEx(tarp, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (41, 41)))
tarp = cv2.morphologyEx(tarp, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (121, 121)))
cnt = max(cv2.findContours(tarp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0], key=cv2.contourArea)
tarp_g = apply(H, cnt[:, 0, :].astype(np.float64))

# ---------------------------------------------------------------- world frame: +y along the first segment, centred on the tarp
tag_g = {k: apply(H, tags[k]).mean(0) for k in tags}
ys, xs = np.nonzero(blue)
bottom = np.argsort(ys)[-400:]                                  # the blue pixels nearest the photo's bottom edge
start_img = np.array([[np.median(xs[bottom]), np.median(ys[bottom])]])
start_g = apply(H, start_img)[0]
d = tag_g[ORDER[0]] - start_g
ang = math.atan2(d[1], d[0]) - math.pi / 2
c, sn = math.cos(-ang), math.sin(-ang)
R = np.array([[c, -sn], [sn, c]])
tarp_r = (tarp_g @ R.T).astype(np.float32)
(rcx, rcy), (rw, rh), rang = cv2.minAreaRect(tarp_r)             # tarp rectangle in the rotated ground frame
rect_r = cv2.boxPoints(((rcx, rcy), (rw, rh), rang)).astype(np.float64)
ctr = np.array([rcx, rcy], float)
dims = sorted([rw, rh])
print(f"tarp: {dims[0]:.2f} m x {dims[1]:.2f} m")
size = math.ceil((dims[1] + 2 * MARGIN) * 2) / 2                  # square world, 0.5 m steps
A = np.eye(3)
A[:2, :2] = R
A[:2, 2] = -ctr
G = A @ H                                                        # image -> world metres
Wpx = int(round(size * PPM))
P = np.array([[PPM, 0, Wpx / 2], [0, -PPM, Wpx / 2], [0, 0, 1.0]])   # world metres -> world pixels (y up -> rows down)
M = P @ G
Pinv = np.linalg.inv(P)
rect_w = rect_r - ctr                                            # tarp corners, world metres


def to_world(p):
    return apply(G, np.atleast_2d(np.asarray(p, float)))


def world_px(p):
    return apply(P, np.atleast_2d(np.asarray(p, float)))


# ---------------------------------------------------------------- top-down floor image
top = cv2.warpPerspective(img, M, (Wpx, Wpx), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))
valid = cv2.warpPerspective(np.full((Hh, Ww), 255, np.uint8), M, (Wpx, Wpx), flags=cv2.INTER_NEAREST)
valid = cv2.erode(valid, np.ones((5, 5), np.uint8))
rect_px = np.round(world_px(rect_w)).astype(np.int32)
tarp_mask = np.zeros((Wpx, Wpx), np.uint8)
cv2.fillConvexPoly(tarp_mask, rect_px, 255)
near_tarp = cv2.dilate(tarp_mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * int(KEEP * PPM) + 1,) * 2))
# things standing on the floor (chair legs, wheels) are stretched by the projection: very dark blobs outside the tarp
dark = cv2.inRange(cv2.cvtColor(top, cv2.COLOR_BGR2HSV), (0, 0, 0), (180, 255, 70))
dark = cv2.dilate(cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8)), np.ones((15, 15), np.uint8)) & ~tarp_mask

rng = np.random.default_rng(7)
floor_px = img[(tarp == 0) & (hsv[:, :, 1] < 60) & (hsv[:, :, 2] > 70) & (hsv[:, :, 2] < 200)]
fc = np.median(floor_px, axis=0) if len(floor_px) > 1000 else np.array([120, 125, 128])
floor = np.clip(fc + rng.normal(0, 9, (Wpx, Wpx, 1)) + rng.normal(0, 3, (Wpx, Wpx, 3)), 0, 255)
speck = rng.random((Wpx, Wpx)) < 0.02
floor[speck] = np.clip(floor[speck] + rng.normal(0, 45, (int(speck.sum()), 1)), 0, 255)
world = cv2.GaussianBlur(floor.astype(np.uint8), (3, 3), 0)
# the part of the tarp the photo does not show: plain tarp colour
tarp_px = img[(tarp > 0) & (hsv[:, :, 1] < 30) & (hsv[:, :, 2] > 170)]
tc = np.median(tarp_px, axis=0) if len(tarp_px) > 1000 else np.array([215, 215, 215])
plain = np.clip(tc + rng.normal(0, 3, (Wpx, Wpx, 1)), 0, 255).astype(np.uint8)
world[tarp_mask > 0] = plain[tarp_mask > 0]
use = (valid > 0) & (near_tarp > 0) & (dark == 0)
world[use] = top[use]
cv2.imwrite(os.path.join(outdir, "world.png"), world)

# ---------------------------------------------------------------- centre-lines in the top-down view
blue_top = cv2.warpPerspective(blue, M, (Wpx, Wpx), flags=cv2.INTER_NEAREST) & tarp_mask
yellow_top = cv2.warpPerspective(yellow, M, (Wpx, Wpx), flags=cv2.INTER_NEAREST) & near_tarp
tag_w = {k: to_world(tags[k]).mean(0) for k in tags}
tag_yaw = {}
for k in tags:
    cw = to_world(tags[k])
    up = (cw[0] + cw[1]) / 2 - (cw[3] + cw[2]) / 2               # tag "up" direction
    tag_yaw[k] = math.degrees(math.atan2(up[1], up[0]))
for k in tags:                                                   # tags (and their plates) are not line
    cv2.circle(blue_top, tuple(int(v) for v in np.round(world_px(tag_w[k])[0])), int(0.12 * PPM), 0, -1)
blue_top = cv2.morphologyEx(blue_top, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)))


def trace(mask):
    """ordered centre-line (pixels) of every connected blob: the longest geodesic path of its skeleton"""
    out = []
    nn, lb, stt, _ = cv2.connectedComponentsWithStats(mask)
    for k in range(1, nn):
        if stt[k, cv2.CC_STAT_AREA] < (0.10 * PPM) * (0.03 * PPM):
            continue
        pts = np.argwhere(skeletonize(lb == k))
        if len(pts) < 5:
            continue
        idx = {tuple(p): j for j, p in enumerate(pts)}
        nb = [[idx[(y + dy, x + dx)] for dy in (-1, 0, 1) for dx in (-1, 0, 1) if (dy or dx) and (y + dy, x + dx) in idx] for y, x in pts]

        def bfs(src):
            dist = np.full(len(pts), -1)
            par = np.full(len(pts), -1)
            dist[src] = 0
            q = [src]
            for u in q:
                for v in nb[u]:
                    if dist[v] < 0:
                        dist[v] = dist[u] + 1
                        par[v] = u
                        q.append(v)
            return dist, par

        a = int(np.argmax(bfs(0)[0]))
        dist, par = bfs(a)
        path = [int(np.argmax(dist))]
        while path[-1] != a:
            path.append(int(par[path[-1]]))
        out.append(pts[path][:, ::-1].astype(float))             # (x, y) pixels
    return out


def smooth(poly, step=0.02):
    w = apply(Pinv, poly)
    dd = np.r_[0, np.cumsum(np.linalg.norm(np.diff(w, axis=0), axis=1))]
    if dd[-1] < step:
        return w
    t = np.arange(0, dd[-1] + 1e-9, step)
    r = np.c_[np.interp(t, dd, w[:, 0]), np.interp(t, dd, w[:, 1])]
    k = 5                                                        # light moving average (corners stay)
    if len(r) > 2 * k:
        pad = np.r_[np.repeat(r[:1], k // 2, 0), r, np.repeat(r[-1:], k // 2, 0)]
        r = np.c_[np.convolve(pad[:, 0], np.ones(k) / k, "valid"), np.convolve(pad[:, 1], np.ones(k) / k, "valid")]
    return r


def length(p):
    return float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))


def near_tag(p):
    return min(np.linalg.norm(p - t) for t in tag_w.values()) < 0.30


def join_pieces(lines, label):
    """join the pieces of one line that another tape crosses over (free ends close together, away from the tags)"""
    lines = [ln for ln in lines if length(ln) > 0.10]
    merged = True
    while merged:
        merged = False
        for a_i in range(len(lines)):
            for b_i in range(a_i + 1, len(lines)):
                for a in (lines[a_i], lines[a_i][::-1]):
                    for b in (lines[b_i], lines[b_i][::-1]):
                        if np.linalg.norm(a[-1] - b[0]) < 0.25 and not near_tag(a[-1]) and not near_tag(b[0]):
                            da = a[-1] - a[-min(8, len(a))]
                            db = b[min(7, len(b) - 1)] - b[0]
                            if np.dot(da, db) / (np.linalg.norm(da) * np.linalg.norm(db) + 1e-9) > 0.5:
                                lines[a_i] = np.r_[a, b]
                                lines.pop(b_i)
                                merged = True
                                break
                    if merged:
                        break
                if merged:
                    break
            if merged:
                break
    print(f"{label}: {len(lines)} lines, lengths {[round(length(ln), 2) for ln in lines]} m")
    return lines


def trim(w, cut=0.03):
    """drop the last centimetres at both ends: the skeleton of a tape end bends toward a corner of the tape"""
    dd = np.r_[0, np.cumsum(np.linalg.norm(np.diff(w, axis=0), axis=1))]
    if dd[-1] < 4 * cut:
        return w
    return w[(dd >= cut) & (dd <= dd[-1] - cut)]


def extend(w, mask, reach=0.09, step=0.005):
    """put back what the skeleton and trim() took off: follow each end straight on for as long as it is on the tape"""
    if len(w) < 8:
        return w
    for end in (0, -1):
        tip, ref = (w[0], w[8]) if end == 0 else (w[-1], w[-9])
        near = w[1] if end == 0 else w[-2]                           # direction over 14 cm: the last point may sit off the axis
        d = (near - ref) / (np.linalg.norm(near - ref) + 1e-9)
        last = None
        for k in range(1, int(reach / step) + 1):
            q = tip + d * step * k
            px = np.round(world_px(q)[0]).astype(int)
            if not (0 <= px[0] < mask.shape[1] and 0 <= px[1] < mask.shape[0]) or mask[px[1], px[0]] == 0:
                break
            last = q
        if last is not None:
            w = np.r_[[last], w] if end == 0 else np.r_[w, [last]]
    return w


blue_lines = join_pieces([extend(trim(smooth(p)), blue_top) for p in trace(blue_top)], "blue")
yellow_lines = join_pieces([extend(trim(smooth(p)), yellow_top) for p in trace(yellow_top)], "yellow")

# chain the blue lines in tag order; a short piece between a crossing tape and the tag belongs to the same segment
segs, pool = [], list(blue_lines)
start_w = to_world(start_img)[0]
for j, tid in enumerate(ORDER):
    anchor = start_w if j == 0 else tag_w[ORDER[j - 1]]
    best = min(((np.linalg.norm(ln[e] - anchor), n_i, e) for n_i, ln in enumerate(pool) for e in (0, -1)), key=lambda v: v[0])
    line = pool.pop(best[1])
    line = line if best[2] == 0 else line[::-1]
    grew = True
    while grew and np.linalg.norm(line[-1] - tag_w[tid]) > 0.22:                 # more tape of this segment before the tag?
        grew = False
        for n_i, ln in enumerate(pool):
            cand = ln if np.linalg.norm(ln[0] - line[-1]) <= np.linalg.norm(ln[-1] - line[-1]) else ln[::-1]
            if np.linalg.norm(cand[0] - line[-1]) < 0.30 and np.linalg.norm(cand[-1] - tag_w[tid]) < np.linalg.norm(line[-1] - tag_w[tid]):
                line = np.r_[line, cand]
                pool.pop(n_i)
                grew = True
                break
    tape_end = np.linalg.norm(line[-1] - tag_w[tid])
    line = np.r_[line, [tag_w[tid]]]                                              # the segment is flown up to the tag centre
    print(f"segment {j + 1}: starts {best[0]:.2f} m from {'the start' if j == 0 else 'tag ' + str(ORDER[j - 1])}, "
          f"tape {length(line) - tape_end:.2f} m, tape ends {tape_end:.2f} m from the centre of tag {tid}")
    segs.append(line)
if pool:
    print(f"unused blue pieces: {[round(length(ln), 2) for ln in pool]} m")


# ---------------------------------------------------------------- per-tag transition: turn and where the next line starts
def heading(p, at_end, span=0.18):
    q = p[::-1] if at_end else p
    dd = np.r_[0, np.cumsum(np.linalg.norm(np.diff(q, axis=0), axis=1))]
    j = min(len(q) - 1, int(np.searchsorted(dd, span)))
    v = (q[0] - q[j]) if at_end else (q[j] - q[0])
    return math.degrees(math.atan2(v[1], v[0]))


def wrap(a):
    return (a + 180) % 360 - 180


trans = []
for j in range(len(ORDER) - 1):
    arr = heading(segs[j][:-1], True)
    dep = heading(segs[j + 1], False)
    off = segs[j + 1][0] - tag_w[ORDER[j]]
    ca, sa = math.cos(math.radians(dep)), math.sin(math.radians(dep))
    fwd, left = off[0] * ca + off[1] * sa, -off[0] * sa + off[1] * ca          # start offset in the departure frame
    turn_ccw = wrap(dep - arr)
    trans.append((ORDER[j], -turn_ccw, fwd, left, float(np.linalg.norm(off))))
    print(f"at tag {ORDER[j]}: arrive heading {arr:.0f}, next line leaves heading {dep:.0f} -> turn {-turn_ccw:+.0f} deg (cw+), "
          f"next line starts {np.linalg.norm(off):.2f} m from the tag ({fwd:+.2f} ahead, {left:+.2f} left after the turn)")
route_len = sum(length(sg) for sg in segs) + sum(np.linalg.norm(segs[j + 1][0] - segs[j][-1]) for j in range(len(segs) - 1))
print(f"route length (tape + links): {route_len:.2f} m")

# ---------------------------------------------------------------- course file
with open(os.path.join(outdir, "course.txt"), "w") as f:
    f.write(f"# generated by tools/course_from_photo.py from {os.path.basename(photo)}\n")
    f.write(f"world_size {size}\npx_per_m {PPM}\nimage world.png\ntag_size {TAG}\n")
    # The simulator draws exact tags over the photographed ones: after rectifying one photo the tags come out
    # 1-2 % different in size, which the printed ones are not (--keep-photo-tags leaves the photo as it is).
    f.write("tags_in_image 1\n" if KEEP_TAGS else f"tags_in_image 0\ntag_paper {TAG / 0.8 + 0.01:.3f} {TAG / 0.8 + 0.01:.3f}\n")
    # the drone is put down on the first tape a hand-span from its beginning, nose along it: the camera (which looks
    # down and a little backwards through the mirror) then has tape in sight from the first moment
    dd0 = np.r_[0, np.cumsum(np.linalg.norm(np.diff(segs[0], axis=0), axis=1))]
    s0 = np.array([np.interp(START_IN, dd0, segs[0][:, 0]), np.interp(START_IN, dd0, segs[0][:, 1])])
    f.write(f"start {s0[0]:.3f} {s0[1]:.3f} {heading(segs[0], False):.1f}\n")
    for sgm in segs:
        f.write("segment " + " ".join(f"{x:.3f},{y:.3f}" for x, y in sgm) + "\n")
    for tid in ORDER:
        f.write(f"tag {tid} {tag_w[tid][0]:.3f} {tag_w[tid][1]:.3f} {tag_yaw[tid]:.1f}\n")
    for line in yellow_lines:
        f.write("distractor " + " ".join(f"{x:.3f},{y:.3f}" for x, y in line) + "\n")
    for tid, turn, fwd, left, dist in trans:
        f.write(f"transition {tid} {turn:.1f} {fwd:.3f} {left:.3f}\n")
    f.write("tarp " + " ".join(f"{x:.3f},{y:.3f}" for x, y in rect_w) + "\n")

dbg = world.copy()
cv2.polylines(dbg, [rect_px], True, (0, 255, 0), 1, cv2.LINE_AA)
for line in yellow_lines:
    cv2.polylines(dbg, [np.round(world_px(line)).astype(np.int32)], False, (0, 140, 255), 2, cv2.LINE_AA)
cols = [(0, 0, 255), (0, 200, 0), (255, 0, 255), (0, 255, 255)]
for j, sgm in enumerate(segs):
    pp = np.round(world_px(sgm)).astype(np.int32)
    cv2.polylines(dbg, [pp], False, cols[j % 4], 2, cv2.LINE_AA)
    cv2.circle(dbg, tuple(int(v) for v in pp[0]), 7, cols[j % 4], -1)
    cv2.putText(dbg, f"S{j + 1}", tuple(int(v) for v in pp[0] + [8, -8]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, cols[j % 4], 2)
for k in tags:
    cc = np.round(world_px(tag_w[k])[0]).astype(int)
    cv2.circle(dbg, tuple(int(v) for v in cc), int(0.13 * PPM), (255, 255, 0), 2)
    cv2.putText(dbg, f"ID{k}", tuple(int(v) for v in cc + [-20, -45]), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
for mm in np.arange(-size / 2, size / 2 + 0.01, 0.5):
    q = int(round(Wpx / 2 + mm * PPM))
    cv2.line(dbg, (q, 0), (q, Wpx), (90, 90, 90), 1)
    cv2.line(dbg, (0, q), (Wpx, q), (90, 90, 90), 1)
cv2.imwrite(os.path.join(outdir, "debug.png"), dbg)
print(f"wrote {outdir}/world.png ({Wpx}x{Wpx}, {size} m), course.txt, debug.png")
