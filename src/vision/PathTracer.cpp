#include "vision/PathTracer.h"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

#include "util/MathUtil.h"

void TraceParams::load(const Config& c) {
    cellM = c.getDouble("path.cell_m", cellM);
    maxRangeM = c.getDouble("path.max_range_m", maxRangeM);
    minLengthM = c.getDouble("path.min_length_m", minLengthM);
    minWidthCm = c.getDouble("path.min_width_cm", minWidthCm);
    maxWidthCm = c.getDouble("path.max_width_cm", maxWidthCm);
    stepM = c.getDouble("path.step_m", stepM);
    joinGapM = c.getDouble("path.join_gap_m", joinGapM);
    joinCos = c.getDouble("path.join_cos", joinCos);
    tagGuardM = c.getDouble("path.tag_guard_m", tagGuardM);
    edgeMarginPx = c.getInt("path.edge_margin_px", edgeMarginPx);
}

namespace {
cv::Point2d unit(const cv::Point2d& v) {
    double n = std::hypot(v.x, v.y);
    return n > 1e-9 ? cv::Point2d(v.x / n, v.y / n) : cv::Point2d(0, 0);
}

double polyLength(const std::vector<cv::Point2d>& p) {
    double s = 0;
    for (size_t i = 1; i < p.size(); ++i) s += std::hypot(p[i].x - p[i - 1].x, p[i].y - p[i - 1].y);
    return s;
}

// Zhang-Suen thinning of a 0/1 image whose one-pixel border is clear.
void thin(cv::Mat& im) {
    const int rows = im.rows, cols = im.cols;
    std::vector<uchar*> del;
    del.reserve(4096);
    bool changed = true;
    while (changed) {
        changed = false;
        for (int iter = 0; iter < 2; ++iter) {
            del.clear();
            for (int y = 1; y < rows - 1; ++y) {
                uchar* pu = im.ptr<uchar>(y - 1);
                uchar* pc = im.ptr<uchar>(y);
                uchar* pd = im.ptr<uchar>(y + 1);
                for (int x = 1; x < cols - 1; ++x) {
                    if (!pc[x]) continue;
                    int p2 = pu[x], p3 = pu[x + 1], p4 = pc[x + 1], p5 = pd[x + 1], p6 = pd[x], p7 = pd[x - 1], p8 = pc[x - 1], p9 = pu[x - 1];
                    int B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
                    if (B < 2 || B > 6) continue;
                    int A = (!p2 && p3) + (!p3 && p4) + (!p4 && p5) + (!p5 && p6) + (!p6 && p7) + (!p7 && p8) + (!p8 && p9) + (!p9 && p2);
                    if (A != 1) continue;
                    if (iter == 0) { if ((p2 && p4 && p6) || (p4 && p6 && p8)) continue; }
                    else { if ((p2 && p4 && p8) || (p2 && p6 && p8)) continue; }
                    del.push_back(pc + x);
                }
            }
            for (uchar* q : del) *q = 0;
            if (!del.empty()) changed = true;
        }
    }
}

// Even spacing along the polyline, then a 3-point moving average (the end points stay).
std::vector<cv::Point2d> resample(const std::vector<cv::Point2d>& in, double step) {
    std::vector<cv::Point2d> out;
    if (in.size() < 2) return in;
    std::vector<double> cum(in.size(), 0.0);
    for (size_t i = 1; i < in.size(); ++i) cum[i] = cum[i - 1] + std::hypot(in[i].x - in[i - 1].x, in[i].y - in[i - 1].y);
    double total = cum.back();
    if (total < 1e-6) return in;
    int n = std::max(1, (int)std::lround(total / step));
    size_t k = 1;
    for (int i = 0; i <= n; ++i) {
        double s = total * i / n;
        while (k < in.size() - 1 && cum[k] < s) ++k;
        double span = cum[k] - cum[k - 1];
        double f = span > 1e-9 ? (s - cum[k - 1]) / span : 0.0;
        out.emplace_back(in[k - 1].x + f * (in[k].x - in[k - 1].x), in[k - 1].y + f * (in[k].y - in[k - 1].y));
    }
    if (out.size() >= 5) {
        std::vector<cv::Point2d> sm = out;
        for (size_t i = 1; i + 1 < out.size(); ++i) sm[i] = (out[i - 1] + out[i] + out[i + 1]) * (1.0 / 3.0);
        out.swap(sm);
    }
    return out;
}

double distPointSeg(const cv::Point2d& p, const cv::Point2d& a, const cv::Point2d& b) {
    double dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
    double t = l2 > 1e-12 ? mu::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / l2, 0.0, 1.0) : 0.0;
    return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}
}  // namespace

cv::Point2d PathLine::tangentA() const {
    if (pts.size() < 2) return cv::Point2d(0, 0);
    size_t k = std::min(pts.size() - 1, (size_t)4);
    return unit(pts.front() - pts[k]);
}

cv::Point2d PathLine::tangentB() const {
    if (pts.size() < 2) return cv::Point2d(0, 0);
    size_t k = pts.size() - 1 - std::min(pts.size() - 1, (size_t)4);
    return unit(pts.back() - pts[k]);
}

bool PathTracer::supported(const GroundProjector& proj) const {
    const Intrinsics& K = proj.K();
    const double us[4] = {0.0, K.width - 1.0, K.width - 1.0, 0.0}, vs[4] = {0.0, 0.0, K.height - 1.0, K.height - 1.0};
    for (int k = 0; k < 4; ++k) {
        double X, Y;
        if (!proj.pixelToGroundAny(us[k], vs[k], X, Y) || std::hypot(X, Y) > 3.0 * p_.maxRangeM) return false;
    }
    return true;
}

std::vector<PathLine> PathTracer::trace(const cv::Mat& mask, double scale, const GroundProjector& proj,
                                        const std::vector<cv::Point2d>& tags) {
    std::vector<PathLine> out;
    grid_.release();
    if (mask.empty() || !supported(proj)) return out;
    const Intrinsics& K = proj.K();

    // 1. the piece of floor the camera sees
    double xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9;
    const int N = 6;
    for (int i = 0; i <= N; ++i) {
        double f = (double)i / N;
        const double us[4] = {f * (K.width - 1), f * (K.width - 1), 0.0, K.width - 1.0};
        const double vs[4] = {0.0, K.height - 1.0, f * (K.height - 1), f * (K.height - 1)};
        for (int k = 0; k < 4; ++k) {
            double X, Y;
            if (!proj.pixelToGroundAny(us[k], vs[k], X, Y)) return out;
            xmin = std::min(xmin, X); xmax = std::max(xmax, X); ymin = std::min(ymin, Y); ymax = std::max(ymax, Y);
        }
    }
    xmin = std::max(xmin, -p_.maxRangeM); xmax = std::min(xmax, p_.maxRangeM);
    ymin = std::max(ymin, -p_.maxRangeM); ymax = std::min(ymax, p_.maxRangeM);
    if (xmax - xmin < 0.1 || ymax - ymin < 0.1) return out;

    // 2. top-down grid: row r <-> X = xmax - (r + 0.5) cell (forward is up), column c <-> Y = ymax - (c + 0.5) cell (left is left)
    double cell = p_.cellM;
    int rows = (int)std::ceil((xmax - xmin) / cell), cols = (int)std::ceil((ymax - ymin) / cell);
    while (rows > 520 || cols > 520) {
        cell *= 1.25;
        rows = (int)std::ceil((xmax - xmin) / cell);
        cols = (int)std::ceil((ymax - ymin) / cell);
    }
    if (rows < 8 || cols < 8) return out;
    auto toGround = [&](double r, double c) { return cv::Point2d(xmax - (r + 0.5) * cell, ymax - (c + 0.5) * cell); };

    // 3. the floor is a plane: four points fix the mapping grid -> image
    cv::Point2f src[4], dst[4];
    const double inset = 0.12;
    const double us[4] = {K.width * inset, K.width * (1 - inset), K.width * (1 - inset), K.width * inset};
    const double vs[4] = {K.height * inset, K.height * inset, K.height * (1 - inset), K.height * (1 - inset)};
    for (int k = 0; k < 4; ++k) {
        double X, Y;
        if (!proj.pixelToGroundAny(us[k], vs[k], X, Y)) return out;
        src[k] = cv::Point2f((float)((ymax - Y) / cell - 0.5), (float)((xmax - X) / cell - 0.5));
        dst[k] = cv::Point2f((float)(us[k] * scale), (float)(vs[k] * scale));
    }
    cv::Mat Hm = cv::getPerspectiveTransform(src, dst);
    cv::warpPerspective(mask, grid_, Hm, cv::Size(cols, rows), cv::INTER_LINEAR | cv::WARP_INVERSE_MAP, cv::BORDER_CONSTANT, cv::Scalar(0));
    cv::threshold(grid_, grid_, 100, 255, cv::THRESH_BINARY);
    cv::morphologyEx(grid_, grid_, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
    grid_.row(0).setTo(0); grid_.row(rows - 1).setTo(0); grid_.col(0).setTo(0); grid_.col(cols - 1).setTo(0);
    if (cv::countNonZero(grid_) < 6) return out;

    // 4. tape width from the distance to the nearest non-tape cell, then thin to a skeleton
    cv::distanceTransform(grid_, dist_, cv::DIST_L2, 3);
    cv::threshold(grid_, skel_, 0, 1, cv::THRESH_BINARY);
    thin(skel_);

    // 5. walk every skeleton from one end to the other (longest path: short spurs at corners are ignored)
    index_.create(rows, cols);
    index_.setTo(-1);
    std::vector<cv::Point> px;
    for (int r = 1; r < rows - 1; ++r) {
        const uchar* q = skel_.ptr<uchar>(r);
        for (int c = 1; c < cols - 1; ++c)
            if (q[c]) { index_(r, c) = (int)px.size(); px.emplace_back(c, r); }
    }
    const int n = (int)px.size();
    std::vector<int> comp(n, -1), dst2(n, -1), par(n, -1), queue;
    queue.reserve(n);
    auto neighbours = [&](int i, int nb[8]) {
        int m = 0;
        for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx) {
                if (!dx && !dy) continue;
                int j = index_(px[i].y + dy, px[i].x + dx);
                if (j >= 0) nb[m++] = j;
            }
        return m;
    };
    const int minCells = std::max(3, (int)(p_.minLengthM / cell * 0.6));
    std::vector<PathLine> lines;
    for (int s0 = 0; s0 < n; ++s0) {
        if (comp[s0] >= 0) continue;
        // label the component, the last cell reached is one end
        queue.clear();
        queue.push_back(s0);
        comp[s0] = s0;
        for (size_t h = 0; h < queue.size(); ++h) {
            int nb[8], m = neighbours(queue[h], nb);
            for (int k = 0; k < m; ++k)
                if (comp[nb[k]] < 0) { comp[nb[k]] = s0; queue.push_back(nb[k]); }
        }
        if ((int)queue.size() < minCells) continue;
        int a = queue.back();
        std::vector<int> members = queue;
        for (int i : members) dst2[i] = -1;
        queue.clear();
        queue.push_back(a);
        dst2[a] = 0;
        par[a] = -1;
        for (size_t h = 0; h < queue.size(); ++h) {
            int nb[8], m = neighbours(queue[h], nb);
            for (int k = 0; k < m; ++k)
                if (dst2[nb[k]] < 0) { dst2[nb[k]] = dst2[queue[h]] + 1; par[nb[k]] = queue[h]; queue.push_back(nb[k]); }
        }
        int b = queue.back();
        std::vector<cv::Point2d> raw;
        std::vector<double> widths;
        for (int i = b; i >= 0; i = par[i]) {
            raw.push_back(toGround(px[i].y, px[i].x));
            widths.push_back(2.0 * dist_.at<float>(px[i].y, px[i].x) * cell * 100.0);
        }
        PathLine L;
        L.pts = resample(raw, p_.stepM);
        L.length = polyLength(L.pts);
        L.widthCm = mu::median(widths);
        if (L.length < p_.minLengthM || L.pts.size() < 2) continue;
        if (L.widthCm < p_.minWidthCm || L.widthCm > p_.maxWidthCm) continue;
        lines.push_back(L);
    }

    // 6. does a line end because the tape ends, or because the camera cannot see further?
    const double m = p_.edgeMarginPx;
    auto cut = [&](const cv::Point2d& end, const cv::Point2d& outward) {
        cv::Point2d q = end + outward * 0.06;
        if (q.x < xmin + 2 * cell || q.x > xmax - 2 * cell || q.y < ymin + 2 * cell || q.y > ymax - 2 * cell) return true;
        double u, v;
        if (!proj.groundToPixel(q.x, q.y, u, v)) return true;
        return u < m || u > K.width - 1 - m || v < m || v > K.height - 1 - m;
    };
    for (auto& L : lines) { L.cutA = cut(L.pts.front(), L.tangentA()); L.cutB = cut(L.pts.back(), L.tangentB()); }

    // 7. one tape cut in two by a tape lying across it (or by a glare patch) is one line
    bool merged = true;
    while (merged) {
        merged = false;
        for (size_t i = 0; i < lines.size() && !merged; ++i)
            for (size_t j = i + 1; j < lines.size() && !merged; ++j)
                for (int ei = 0; ei < 2 && !merged; ++ei)
                    for (int ej = 0; ej < 2 && !merged; ++ej) {
                        PathLine& A = lines[i];
                        PathLine& B = lines[j];
                        if ((ei ? A.cutB : A.cutA) || (ej ? B.cutB : B.cutA)) continue;
                        cv::Point2d pa = ei ? A.pts.back() : A.pts.front(), ta = ei ? A.tangentB() : A.tangentA();
                        cv::Point2d pb = ej ? B.pts.back() : B.pts.front(), tb = ej ? B.tangentB() : B.tangentA();
                        cv::Point2d g = pb - pa;
                        double gl = std::hypot(g.x, g.y);
                        if (gl > p_.joinGapM) continue;
                        if (ta.dot(tb) > -p_.joinCos) continue;                       // the two ends must face each other
                        if (gl > 0.03 && ta.dot(g * (1.0 / gl)) < p_.joinCos) continue;   // and lie on one straight line
                        bool tagBetween = false;
                        for (const auto& t : tags) if (distPointSeg(t, pa, pb) < p_.tagGuardM) tagBetween = true;
                        if (tagBetween) continue;
                        PathLine J;
                        std::vector<cv::Point2d> first = A.pts, second = B.pts;
                        if (!ei) std::reverse(first.begin(), first.end());       // A's joined end last
                        if (ej) std::reverse(second.begin(), second.end());      // B's joined end first
                        J.pts = first;
                        J.pts.insert(J.pts.end(), second.begin(), second.end());
                        J.cutA = ei ? A.cutA : A.cutB;
                        J.cutB = ej ? B.cutA : B.cutB;
                        J.length = polyLength(J.pts);
                        J.widthCm = 0.5 * (A.widthCm + B.widthCm);
                        lines[i] = J;
                        lines.erase(lines.begin() + (long)j);
                        merged = true;
                    }
    }
    // 8. thinning eats about half a tape width off every real end: put it back, straight on
    for (auto& L : lines) {
        const double add = std::min(0.04, 0.5 * L.widthCm / 100.0);
        if (add < 0.005 || L.pts.size() < 3) continue;
        if (!L.cutA) L.pts.insert(L.pts.begin(), L.pts.front() + L.tangentA() * add);
        if (!L.cutB) L.pts.push_back(L.pts.back() + L.tangentB() * add);
        L.length = polyLength(L.pts);
    }
    return lines;
}
