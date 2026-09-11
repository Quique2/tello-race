#include "sim/Track.h"

#include <random>

#include <opencv2/imgproc.hpp>

#include "util/MathUtil.h"

namespace {
void appendArc(std::vector<cv::Point2d>& pts, double& x, double& y, double& psiDeg, double length, double curvature) {
    const double step = 0.02;
    int n = std::max(1, (int)(length / step));
    double psi = mu::deg2rad(psiDeg);
    for (int i = 0; i < n; ++i) {
        x += step * std::cos(psi);
        y += step * std::sin(psi);
        psi += curvature * step;
        pts.emplace_back(x, y);
    }
    psiDeg = mu::rad2deg(psi);
}
cv::Point2d dirVec(double psiDeg) { return cv::Point2d(std::cos(mu::deg2rad(psiDeg)), std::sin(mu::deg2rad(psiDeg))); }
double minDist(const std::vector<cv::Point2d>& a, const std::vector<cv::Point2d>& b, size_t skipFirstOfA) {
    double m = 1e9;
    for (size_t i = skipFirstOfA; i < a.size(); i += 3)
        for (size_t j = 0; j < b.size(); j += 3) m = std::min(m, cv::norm(a[i] - b[j]));
    return m;
}
}

Track TrackBuilder::example() {
    Track t;
    t.worldSize = 8.0;
    t.start = cv::Point2d(0, -3.0);
    t.startYawDeg = 90;
    t.description = "example layout from the competition slides (RIGHT, FORWARD, LEFT, FINISH)";
    double x = 0, y = -3.0, psi = 90;
    TrackSegment s1; s1.pts.emplace_back(x, y);
    appendArc(s1.pts, x, y, psi, 1.5, 0);
    t.segments.push_back(s1); t.tags.push_back({3, x, y, psi});
    cv::Point2d s = cv::Point2d(x, y) + 0.8 * dirVec(psi - 90);
    x = s.x; y = s.y; psi = 0;
    TrackSegment s2; s2.pts.emplace_back(x, y);
    appendArc(s2.pts, x, y, psi, 0.4, 0);
    appendArc(s2.pts, x, y, psi, 1.4, 1.6);
    t.segments.push_back(s2); t.tags.push_back({1, x, y, psi});
    s = cv::Point2d(x, y) + 0.8 * dirVec(psi);
    x = s.x; y = s.y; psi += 10;
    TrackSegment s3; s3.pts.emplace_back(x, y);
    appendArc(s3.pts, x, y, psi, 1.6, 0);
    t.segments.push_back(s3); t.tags.push_back({2, x, y, psi});
    s = cv::Point2d(x, y) + 0.8 * dirVec(psi + 90);
    x = s.x; y = s.y; psi = psi + 90 - 20;
    TrackSegment s4; s4.pts.emplace_back(x, y);
    appendArc(s4.pts, x, y, psi, 0.5, 1.5);
    appendArc(s4.pts, x, y, psi, 0.7, -1.5);
    appendArc(s4.pts, x, y, psi, 0.5, 1.0);
    appendArc(s4.pts, x, y, psi, 0.4, 0);
    t.segments.push_back(s4); t.tags.push_back({4, x, y, psi});
    TrackSegment d1; double dx = 2.2, dy = -3.0, dpsi = 80;
    d1.pts.emplace_back(dx, dy); appendArc(d1.pts, dx, dy, dpsi, 1.8, 0.4);
    t.distractors.push_back(d1); t.distractorColors.push_back(cv::Scalar(40, 40, 220)); t.distractorOnStrip.push_back(false);
    TrackSegment d2; dx = -2.0; dy = -2.5; dpsi = 60;
    d2.pts.emplace_back(dx, dy); appendArc(d2.pts, dx, dy, dpsi, 1.5, -0.6);
    t.distractors.push_back(d2); t.distractorColors.push_back(cv::Scalar(220, 90, 30)); t.distractorOnStrip.push_back(true);
    return t;
}

Track TrackBuilder::random(uint64_t seed, double worldSize) {
    std::mt19937_64 rng(seed);
    auto uni = [&](double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); };
    Track t;
    t.worldSize = worldSize;
    t.start = cv::Point2d(uni(-1.0, 1.0), -worldSize / 2 + 1.2);
    t.startYawDeg = 90 + uni(-20, 20);
    t.description = "random track seed " + std::to_string(seed);
    int order[3] = {1, 2, 3};
    std::shuffle(order, order + 3, rng);
    double half = worldSize / 2 - 0.7;
    double x = t.start.x, y = t.start.y, psi = t.startYawDeg;
    for (int k = 0; k < 4; ++k) {
        TrackSegment best;
        double bx = x, by = y, bpsi = psi;
        bool ok = false;
        for (int attempt = 0; attempt < 40 && !ok; ++attempt) {
            TrackSegment seg;
            double sx = x, sy = y, spsi = psi;
            seg.pts.emplace_back(sx, sy);
            double total = uni(1.3, 2.6), turned = 0;
            bool inBounds = true;
            while (total > 0) {
                double len = std::min(total, uni(0.3, 0.8));
                double curv = uni(-1.3, 1.3);
                if (std::fabs(turned + curv * len) > mu::deg2rad(140)) curv = -curv * 0.5;
                appendArc(seg.pts, sx, sy, spsi, len, curv);
                turned += curv * len;
                total -= len;
                if (std::fabs(sx) > half || std::fabs(sy) > half) { inBounds = false; break; }
            }
            if (!inBounds) continue;
            bool clear = true;
            for (size_t i = 0; i + 1 < t.segments.size() + 1 && i < t.segments.size(); ++i)
                if (minDist(seg.pts, t.segments[i].pts, 15) < 0.5) { clear = false; break; }
            for (auto& tg : t.tags)
                if (minDist(seg.pts, {cv::Point2d(tg.x, tg.y)}, 15) < 0.5) { clear = false; break; }
            if (!clear && attempt < 30) continue;
            best = seg; bx = sx; by = sy; bpsi = spsi; ok = true;
        }
        if (!ok) { best.pts.clear(); best.pts.emplace_back(x, y); bx = x; by = y; bpsi = psi; appendArc(best.pts, bx, by, bpsi, 1.0, 0); }
        t.segments.push_back(best);
        int id = k < 3 ? order[k] : 4;
        t.tags.push_back({id, bx, by, bpsi + uni(-15, 15)});
        double dirDeg = id == 1 ? bpsi : (id == 2 ? bpsi + 90 : (id == 3 ? bpsi - 90 : bpsi));
        cv::Point2d s = cv::Point2d(bx, by) + 0.8 * dirVec(dirDeg);
        x = s.x; y = s.y;
        double dev = uni(0, 1) < 0.25 ? (uni(0, 1) < 0.5 ? 1 : -1) * uni(65, 85) : uni(-50, 50);
        psi = dirDeg + dev;
    }
    for (int d = 0; d < 2; ++d) {
        TrackSegment seg;
        double dx = uni(-half, half), dy = uni(-half, half), dpsi = uni(0, 360);
        seg.pts.emplace_back(dx, dy);
        appendArc(seg.pts, dx, dy, dpsi, uni(0.8, 2.0), uni(-0.8, 0.8));
        t.distractors.push_back(seg);
        t.distractorColors.push_back(d == 0 ? cv::Scalar(40, 40, 220) : cv::Scalar(220, 90, 30));
        t.distractorOnStrip.push_back(d == 1);
    }
    return t;
}

cv::Point2d TrackBuilder::worldToPx(const Track& t, double pxPerM, const cv::Point2d& p) {
    double W = t.worldSize * pxPerM;
    return cv::Point2d(W / 2 + p.x * pxPerM, W / 2 - p.y * pxPerM);
}

static std::vector<cv::Point> toPx(const Track& t, double ppm, const std::vector<cv::Point2d>& pts) {
    std::vector<cv::Point> out;
    out.reserve(pts.size());
    for (auto& p : pts) {
        cv::Point2d q = TrackBuilder::worldToPx(t, ppm, p);
        out.emplace_back((int)std::lround(q.x), (int)std::lround(q.y));
    }
    return out;
}

cv::Mat TrackBuilder::render(const Track& t, const TrackStyle& s, const TagDetector& tags) {
    int W = (int)std::lround(t.worldSize * s.pxPerM);
    cv::Mat img(W, W, CV_8UC3, s.carpet);
    std::mt19937_64 rng(s.textureSeed);
    std::uniform_real_distribution<double> uni(0, 1);
    // carpet texture: soft patches + fine noise
    cv::Mat patches = img.clone();
    for (int i = 0; i < 60; ++i) {
        cv::Point c((int)(uni(rng) * W), (int)(uni(rng) * W));
        cv::Size ax((int)(20 + uni(rng) * 120), (int)(20 + uni(rng) * 120));
        double d = uni(rng) < 0.5 ? -18 : 14;
        cv::ellipse(patches, c, ax, uni(rng) * 180, 0, 360, s.carpet + cv::Scalar(d, d, d), -1, cv::LINE_AA);
    }
    cv::addWeighted(img, 0.5, patches, 0.5, 0, img);
    cv::Mat noise(W, W, CV_32FC3);
    cv::randn(noise, 0, 7);
    cv::Mat imgF;
    img.convertTo(imgF, CV_32FC3);
    imgF += noise;
    imgF.convertTo(img, CV_8UC3);
    // net boundary: a plain dark band on the floor (the real net is vertical mesh above the horizon)
    int nb = (int)(0.12 * s.pxPerM);
    cv::rectangle(img, cv::Rect(0, 0, W, W), cv::Scalar(55, 52, 50), nb);
    // start marker
    cv::Point2d sp = worldToPx(t, s.pxPerM, t.start);
    cv::circle(img, cv::Point((int)sp.x, (int)sp.y), (int)(0.18 * s.pxPerM), cv::Scalar(160, 160, 160), -1, cv::LINE_AA);
    cv::circle(img, cv::Point((int)sp.x, (int)sp.y), (int)(0.18 * s.pxPerM), cv::Scalar(230, 230, 230), 3, cv::LINE_AA);
    // distractor lines (other colors); some on a white strip, some directly on the carpet
    int stripPx = std::max(2, (int)std::lround(s.stripWidthCm / 100.0 * s.pxPerM));
    int linePx = std::max(1, (int)std::lround(s.lineWidthCm / 100.0 * s.pxPerM));
    for (size_t i = 0; i < t.distractors.size(); ++i) {
        std::vector<cv::Point> pts = toPx(t, s.pxPerM, t.distractors[i].pts);
        if (i < t.distractorOnStrip.size() && t.distractorOnStrip[i])
            cv::polylines(img, pts, false, cv::Scalar(245, 245, 245), stripPx, cv::LINE_AA);
        cv::polylines(img, pts, false, t.distractorColors[i], linePx, cv::LINE_AA);
    }
    // track: white strips, then the dark line
    for (auto& seg : t.segments) cv::polylines(img, toPx(t, s.pxPerM, seg.pts), false, cv::Scalar(245, 245, 245), stripPx, cv::LINE_AA);
    for (auto& seg : t.segments) cv::polylines(img, toPx(t, s.pxPerM, seg.pts), false, s.lineColor, linePx, cv::LINE_AA);
    // tags on letter-size paper
    double frac = tags.blackSquareFraction();
    int tagPx = std::max(8, (int)std::lround(s.tagSizeM / frac * s.pxPerM));
    int paperW = (int)std::lround(0.216 * s.pxPerM), paperH = (int)std::lround(0.279 * s.pxPerM);
    for (auto& tg : t.tags) {
        cv::Mat bits = tags.renderTag(tg.id);
        if (bits.empty()) continue;
        cv::Mat big;
        cv::resize(bits, big, cv::Size(tagPx, tagPx), 0, 0, cv::INTER_NEAREST);
        cv::Mat paper(paperH, paperW, CV_8UC3, cv::Scalar(250, 250, 250));
        cv::Mat bigC;
        cv::cvtColor(big, bigC, cv::COLOR_GRAY2BGR);
        cv::Rect roi((paperW - tagPx) / 2, (paperH - tagPx) / 2, tagPx, tagPx);
        if (roi.x < 0 || roi.y < 0) { roi = cv::Rect(0, 0, std::min(tagPx, paperW), std::min(tagPx, paperH)); }
        bigC(cv::Rect(0, 0, roi.width, roi.height)).copyTo(paper(roi));
        cv::Point2d c = worldToPx(t, s.pxPerM, cv::Point2d(tg.x, tg.y));
        double angle = tg.yawDeg - 90;   // tag "up" points along the approach heading
        int D = (int)std::ceil(std::hypot(paperW, paperH)) + 4;
        cv::Mat canvas(D, D, CV_8UC3, cv::Scalar(0, 0, 0)), mask(D, D, CV_8UC1, cv::Scalar(0));
        cv::Mat pm(paperH, paperW, CV_8UC1, cv::Scalar(255));
        cv::Rect pr((D - paperW) / 2, (D - paperH) / 2, paperW, paperH);
        paper.copyTo(canvas(pr));
        pm.copyTo(mask(pr));
        cv::Mat M = cv::getRotationMatrix2D(cv::Point2f(D / 2.0f, D / 2.0f), angle, 1.0);
        cv::warpAffine(canvas, canvas, M, canvas.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        cv::warpAffine(mask, mask, M, mask.size(), cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
        cv::Rect dst((int)std::lround(c.x) - D / 2, (int)std::lround(c.y) - D / 2, D, D);
        cv::Rect clipped = dst & cv::Rect(0, 0, W, W);
        if (clipped.width <= 0 || clipped.height <= 0) continue;
        cv::Rect src(clipped.x - dst.x, clipped.y - dst.y, clipped.width, clipped.height);
        canvas(src).copyTo(img(clipped), mask(src));
    }
    return img;
}
