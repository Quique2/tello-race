#include "vision/LineDetector.h"

#include <opencv2/imgproc.hpp>

#include "util/Log.h"
#include "util/MathUtil.h"

static cv::Scalar scalarFromList(const Config& cfg, const std::string& key, cv::Scalar def) {
    std::vector<double> v = cfg.getDoubleList(key, {def[0], def[1], def[2]});
    if (v.size() < 3) return def;
    return cv::Scalar(v[0], v[1], v[2]);
}

void LineParams::load(const Config& cfg) {
    procWidth = cfg.getInt("line.proc_width", procWidth);
    colorMode = toLower(cfg.getString("line.color_mode", colorMode));
    blueLo = scalarFromList(cfg, "line.blue_lo", blueLo);
    blueHi = scalarFromList(cfg, "line.blue_hi", blueHi);
    yellowLo = scalarFromList(cfg, "line.yellow_lo", yellowLo);
    yellowHi = scalarFromList(cfg, "line.yellow_hi", yellowHi);
    whiteDilateCm = cfg.getDouble("line.white_dilate_cm", whiteDilateCm);
    darkVMax = cfg.getInt("line.dark_v_max", darkVMax);
    darkSMax = cfg.getInt("line.dark_s_max", darkSMax);
    hsvLo1 = scalarFromList(cfg, "line.hsv_lo", hsvLo1);
    hsvHi1 = scalarFromList(cfg, "line.hsv_hi", hsvHi1);
    hsvRange2 = cfg.has("line.hsv_lo2") && cfg.has("line.hsv_hi2");
    if (hsvRange2) { hsvLo2 = scalarFromList(cfg, "line.hsv_lo2", hsvLo2); hsvHi2 = scalarFromList(cfg, "line.hsv_hi2", hsvHi2); }
    whiteCheck = cfg.getBool("line.white_check", whiteCheck);
    whiteSMax = cfg.getInt("line.white_s_max", whiteSMax);
    whiteVMin = cfg.getInt("line.white_v_min", whiteVMin);
    whiteAuto = cfg.getBool("line.white_auto", whiteAuto);
    whiteDilatePx = cfg.getInt("line.white_dilate_px", whiteDilatePx);
    minWidthCm = cfg.getDouble("line.min_width_cm", minWidthCm);
    maxWidthCm = cfg.getDouble("line.max_width_cm", maxWidthCm);
    morphOpen = cfg.getInt("line.morph_open", morphOpen);
    morphClose = cfg.getInt("line.morph_close", morphClose);
    minComponentArea = cfg.getInt("line.min_component_area", minComponentArea);
    bands = cfg.getInt("line.bands", bands);
    bandStartM = cfg.getDouble("line.band_start_m", bandStartM);
    bandStepM = cfg.getDouble("line.band_step_m", bandStepM);
    bandMaxM = cfg.getDouble("line.band_max_m", bandMaxM);
    maxJumpCm = cfg.getDouble("line.max_jump_cm", maxJumpCm);
    horizonMarginPx = cfg.getInt("line.horizon_margin_px", horizonMarginPx);
    tagMaskDilatePx = cfg.getInt("line.tag_mask_dilate_px", tagMaskDilatePx);
}

bool LineDetector::setColor(const std::string& name) {
    std::string n = toLower(name);
    if (n != "dark" && n != "hsv" && n != "blue" && n != "yellow") return false;
    p_.colorMode = n;
    return true;
}

bool LineObservation::lateralAt(double X, double& Y) const {
    if (pts.size() < 2) { if (pts.size() == 1) { Y = pts[0].Y; return true; } return false; }
    if (X <= pts.front().X) {
        // extrapolate from the first two points
        const LinePoint &a = pts[0], &b = pts[1];
        double dx = b.X - a.X;
        Y = dx > 1e-6 ? a.Y + (b.Y - a.Y) * (X - a.X) / dx : a.Y;
        return true;
    }
    for (size_t i = 1; i < pts.size(); ++i) {
        if (X <= pts[i].X) {
            const LinePoint &a = pts[i - 1], &b = pts[i];
            double dx = b.X - a.X;
            Y = dx > 1e-6 ? a.Y + (b.Y - a.Y) * (X - a.X) / dx : a.Y;
            return true;
        }
    }
    Y = pts.back().Y;
    return true;
}

void LineDetector::buildMask(const cv::Mat& bgr, const GroundProjector& proj,
                             const std::vector<std::vector<cv::Point2f>>& polys) {
    cv::Mat proc;
    scale_ = (double)p_.procWidth / bgr.cols;
    if (std::fabs(scale_ - 1.0) < 1e-3) proc = bgr;
    else cv::resize(bgr, proc, cv::Size(), scale_, scale_, cv::INTER_AREA);
    cv::cvtColor(proc, hsv_, cv::COLOR_BGR2HSV);

    // The brightest unsaturated surface in sight is the white one. When the camera exposes it darker than expected,
    // the brightness limits follow it down (never up).
    int vMinWhite = p_.whiteVMin;
    double expo = 1.0;
    whiteLevel_ = 0;
    if (p_.whiteCheck && p_.whiteAuto) {
        int hist[256] = {0}, total = 0;
        for (int y = 0; y < hsv_.rows; y += 4) {
            const cv::Vec3b* row = hsv_.ptr<cv::Vec3b>(y);
            for (int x = 0; x < hsv_.cols; x += 4)
                if (row[x][1] <= p_.whiteSMax) { ++hist[row[x][2]]; ++total; }
        }
        if (total > 200) {
            int acc = 0, v95 = 255;
            for (int v = 255; v >= 0; --v) { acc += hist[v]; if (acc >= total / 20) { v95 = v; break; } }
            vMinWhite = std::min(vMinWhite, std::max(60, (int)std::lround(0.72 * v95)));   // 72 %: a grey floor stays out
            whiteLevel_ = v95;
            expo = std::min(1.0, v95 / 210.0);
        }
    }
    auto darker = [&](cv::Scalar lo) { lo[2] = std::max(12.0, lo[2] * expo); return lo; };

    if (p_.colorMode == "blue") {
        cv::inRange(hsv_, darker(p_.blueLo), p_.blueHi, mask_);
    } else if (p_.colorMode == "yellow") {
        cv::inRange(hsv_, darker(p_.yellowLo), p_.yellowHi, mask_);
    } else if (p_.colorMode == "hsv") {
        cv::inRange(hsv_, p_.hsvLo1, p_.hsvHi1, mask_);
        if (p_.hsvRange2) {
            cv::Mat m2;
            cv::inRange(hsv_, p_.hsvLo2, p_.hsvHi2, m2);
            mask_ |= m2;
        }
    } else {
        cv::inRange(hsv_, cv::Scalar(0, 0, 0), cv::Scalar(180, p_.darkSMax, p_.darkVMax), mask_);
    }
    if (p_.whiteCheck) {
        whiteVUsed_ = vMinWhite;
        cv::inRange(hsv_, cv::Scalar(0, 0, vMinWhite), cv::Scalar(180, p_.whiteSMax, 255), white_);
        int reach = p_.whiteDilatePx;
        if (p_.whiteDilateCm > 0) {   // a wide tape seen from close by: its middle is further from the white than a few pixels
            cv::Vec3d ray = proj.pixelRayBody(proj.K().cx, proj.K().cy);
            if (ray[2] < -0.05) {
                double mPerPx = (proj.height() / -ray[2]) / (proj.K().fx * scale_);
                reach = std::max(reach, std::min(120, (int)std::lround(p_.whiteDilateCm / 100.0 / mPerPx)));
            }
        }
        int k = 2 * reach + 1;
        // a box is dilated in constant time per pixel whatever its size; the round one only pays off when small
        cv::dilate(white_, white_, cv::getStructuringElement(k > 31 ? cv::MORPH_RECT : cv::MORPH_ELLIPSE, cv::Size(k, k)));
        mask_ &= white_;
    }
    if (p_.morphOpen > 1)
        cv::morphologyEx(mask_, mask_, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(p_.morphOpen, p_.morphOpen)));
    if (p_.morphClose > 1)
        cv::morphologyEx(mask_, mask_, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(p_.morphClose, p_.morphClose)));

    // Everything above the horizon (walls, net, people) is not ground: clear it.
    double hz = proj.horizonRow(proj.K().cx) * scale_ + p_.horizonMarginPx;
    int hzRow = (int)mu::clamp(hz, 0.0, (double)mask_.rows);
    if (hzRow > 0) mask_(cv::Rect(0, 0, mask_.cols, hzRow)).setTo(0);

    // Exclude AprilTags (their black border looks like a line).
    for (const auto& poly : polys) {
        std::vector<cv::Point> pts;
        cv::Point2f c(0, 0);
        for (auto& q : poly) c += q;
        c *= 1.0f / std::max<size_t>(1, poly.size());
        for (auto& q : poly) {
            cv::Point2f d = q - c;
            float n = std::sqrt(d.x * d.x + d.y * d.y);
            cv::Point2f e = n > 1e-3f ? q + d * (p_.tagMaskDilatePx / scale_ / n) : q;
            pts.emplace_back((int)std::lround(e.x * scale_), (int)std::lround(e.y * scale_));
        }
        if (pts.size() >= 3) cv::fillConvexPoly(mask_, pts, cv::Scalar(0));
    }
}

// Scan horizontal bands placed at fixed ground distances ahead of the drone; in each band pick the
// mask run whose projected width is plausible for the line and whose lateral position continues the chain.
bool LineDetector::scanBands(const GroundProjector& proj, const LineObservation* prev, const cv::Point2d* hint, LineObservation& obs) {
    bandDbg_.clear();
    double blind = proj.blindZoneX();
    if (blind > 20) return false;
    const int rows = mask_.rows, cols = mask_.cols;
    double hzRow = proj.horizonRow(proj.K().cx) * scale_ + p_.horizonMarginPx;

    std::vector<LinePoint> chain;
    double expectY = 0, slope = 0;
    bool haveExpect = false;
    int consecutiveMiss = 0, missNothingNearby = 0;
    // With a hint (the expected start of the next segment, 80 cm past the tag) start scanning shortly before it.
    // From a steep camera at altitude the blind zone is a few centimetres, and the tail of the segment just
    // finished lies right under the drone: scanning from the blind zone would lock the chain onto that stub.
    double startX = std::max(0.0, blind) + p_.bandStartM;   // the camera may see behind the drone: scan from under it
    if (hint) startX = std::max(startX, hint->x - 0.35);
    for (int i = 0; i < p_.bands; ++i) {
        double X = startX + i * p_.bandStepM;
        if (X > p_.bandMaxM) break;
        double u, v;
        if (!proj.groundToPixel(X, 0.0, u, v)) break;
        int row = (int)std::lround(v * scale_);
        if (row < hzRow || row >= rows) break;
        int r0 = std::max(0, row - 1), r1 = std::min(rows - 1, row + 1);

        std::vector<std::pair<int, int>> runs;
        int start = -1;
        for (int c = 0; c <= cols; ++c) {
            bool on = false;
            if (c < cols)
                for (int r = r0; r <= r1 && !on; ++r) on = mask_.at<uint8_t>(r, c) != 0;
            if (on && start < 0) start = c;
            if (!on && start >= 0) { runs.emplace_back(start, c - 1); start = -1; }
        }
        BandDebug dbg{row, runs, -1};

        if (!haveExpect) {
            // a hint (expected segment start) wins; a previous chain is only trusted near where it was seen
            if (hint) { expectY = hint->y; haveExpect = true; }
            else if (prev && prev->valid && prev->pts.front().X < X + 0.6 && prev->lateralAt(X, expectY)) haveExpect = true;
            else expectY = 0;
        }
        int best = -1, validRuns = 0;
        bool nearbyAny = false;
        double bestErr = 1e9;
        LinePoint bestPt;
        for (size_t k = 0; k < runs.size(); ++k) {
            double xl, yl, xr, yr;
            double uL = runs[k].first / scale_, uR = (runs[k].second + 1) / scale_, vv = row / scale_;
            if (!proj.pixelToGround(uL, vv, xl, yl) || !proj.pixelToGround(uR, vv, xr, yr)) continue;
            double Y = 0.5 * (yl + yr);
            if (std::fabs(Y - expectY) < 0.5) nearbyAny = true;
            // Width measured independently of the line orientation: combine the horizontal extent of the run
            // with the vertical extent of the mask through its centre (1/w^2 = 1/wh^2 + 1/wv^2 for a straight strip).
            double wh = std::fabs(yl - yr);
            int cc = (runs[k].first + runs[k].second) / 2;
            int up = row, dn = row;
            while (up > 0 && mask_.at<uint8_t>(up - 1, cc) && row - up < 80) --up;
            while (dn < rows - 1 && mask_.at<uint8_t>(dn + 1, cc) && dn - row < 80) ++dn;
            double xu, yu, xd, yd, wv = 1e9;
            if (proj.pixelToGround(cc / scale_, up / scale_, xu, yu) && proj.pixelToGround(cc / scale_, (dn + 1) / scale_, xd, yd))
                wv = std::hypot(xu - xd, yu - yd);
            double w = 100.0 / std::sqrt(1.0 / (wh * wh + 1e-9) + 1.0 / (wv * wv + 1e-9));
            if (w < p_.minWidthCm || w > p_.maxWidthCm) continue;
            ++validRuns;
            double err = std::fabs(Y - expectY);
            if (err < bestErr) {
                bestErr = err; best = (int)k;
                bestPt.u = 0.5 * (uL + uR); bestPt.v = vv; bestPt.X = 0.5 * (xl + xr); bestPt.Y = Y; bestPt.widthCm = w;
            }
        }
        double allow = p_.maxJumpCm / 100.0 * (chain.empty() ? (haveExpect ? 2.0 : 8.0) : 1.0 + consecutiveMiss)
                       + std::fabs(slope) * p_.bandStepM;
        if (best >= 0 && bestErr <= allow) {
            chain.push_back(bestPt);
            dbg.chosen = best;
            if (chain.size() >= 2) {
                const LinePoint& a = chain[chain.size() - 2];
                const LinePoint& b = chain.back();
                double dx = b.X - a.X;
                slope = dx > 1e-6 ? (b.Y - a.Y) / dx : 0;
            }
            expectY = bestPt.Y + slope * p_.bandStepM;
            haveExpect = true;
            consecutiveMiss = 0;
            missNothingNearby = 0;
        } else if (!chain.empty()) {
            ++consecutiveMiss;
            if (!nearbyAny) ++missNothingNearby;
            expectY += slope * p_.bandStepM;
            if (consecutiveMiss >= 2) {
                bandDbg_.push_back(dbg);
                if (missNothingNearby >= 2 && !lineContinuesBeyond(proj, chain)) {
                    obs.endVisible = true; obs.endX = chain.back().X; obs.endY = chain.back().Y;
                }
                break;
            }
        }
        bandDbg_.push_back(dbg);
        (void)validRuns;
    }
    LOG_DEBUG("bands: hint=%d (%.2f,%.2f) startX=%.2f blind=%.2f hzRow=%.0f scanned=%zu chain=%zu first=(%.2f,%.2f)", hint ? 1 : 0, hint ? hint->x : 0.0, hint ? hint->y : 0.0, startX, blind, hzRow,
              bandDbg_.size(), chain.size(), chain.empty() ? 0.0 : chain.front().X, chain.empty() ? 0.0 : chain.front().Y);
    if (chain.empty()) return false;
    obs.valid = true;
    obs.mode = 1;
    obs.pts = chain;
    obs.confidence = std::min(1.0, 0.3 + 0.15 * chain.size());
    return chain.size() >= 2;
}

static double distPointSegment(double px, double py, const LinePoint& a, const LinePoint& b) {
    double dx = b.X - a.X, dy = b.Y - a.Y, l2 = dx * dx + dy * dy;
    double t = l2 > 1e-9 ? mu::clamp(((px - a.X) * dx + (py - a.Y) * dy) / l2, 0.0, 1.0) : 0.0;
    return std::hypot(px - (a.X + t * dx), py - (a.Y + t * dy));
}

// True if mask pixels exist within ~0.55 m of the chain end that are not part of the chain itself,
// i.e. the line bends away / continues even though the band scan could not follow it.
bool LineDetector::lineContinuesBeyond(const GroundProjector& proj, const std::vector<LinePoint>& chain) const {
    const LinePoint& e = chain.back();
    double mpp = proj.metersPerPixel(e.u, e.v);
    if (mpp <= 0 || mpp > 1) return false;
    int halfW = (int)std::min(400.0, 0.6 * scale_ / mpp);
    double ua, va, ub, vb;
    int rTop = 0, rBot = mask_.rows - 1;
    if (proj.groundToPixel(e.X + 0.6, e.Y, ua, va)) rTop = (int)std::lround(va * scale_);
    if (proj.groundToPixel(std::max(0.05, e.X - 0.25), e.Y, ub, vb)) rBot = (int)std::lround(vb * scale_);
    int c = (int)std::lround(e.u * scale_);
    int r0 = mu::clamp(rTop, 0, mask_.rows - 1), r1 = mu::clamp(rBot, 0, mask_.rows - 1);
    int c0 = mu::clamp(c - halfW, 0, mask_.cols - 1), c1 = mu::clamp(c + halfW, 0, mask_.cols - 1);
    int count = 0;
    for (int y = r0; y <= r1; y += 2) {
        const uint8_t* row = mask_.ptr<uint8_t>(y);
        for (int x = c0; x <= c1; x += 2) {
            if (!row[x]) continue;
            double X, Y;
            if (!proj.pixelToGround(x / scale_, y / scale_, X, Y)) continue;
            double dEnd = std::hypot(X - e.X, Y - e.Y);
            if (dEnd > 0.55 || dEnd < 0.06) continue;
            double dmin = dEnd;
            for (size_t i = 1; i < chain.size(); ++i) dmin = std::min(dmin, distPointSegment(X, Y, chain[i - 1], chain[i]));
            if (dmin > 0.08 && ++count > 12) return true;
        }
    }
    return false;
}

// Fallback for lines at any orientation (e.g. the start of a segment seen sideways after a turn):
// PCA axis of the best connected component, sampled into a polyline ordered from the drone outwards.
bool LineDetector::scanComponents(const GroundProjector& proj, LineObservation& obs, const cv::Point2d& expect, bool haveExpect) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    double bestScore = -1e9;
    LineObservation best;
    for (auto& ct : contours) {
        double area = cv::contourArea(ct);
        if (area < p_.minComponentArea) continue;
        cv::Rect bb = cv::boundingRect(ct);
        cv::Mat cm = cv::Mat::zeros(bb.size(), CV_8UC1);
        std::vector<std::vector<cv::Point>> one{ct};
        cv::drawContours(cm, one, 0, cv::Scalar(255), cv::FILLED, cv::LINE_8, cv::noArray(), 0x7fffffff, -bb.tl());
        cv::Mat roi = mask_(bb);
        int step = area > 6000 ? 2 : 1;
        std::vector<cv::Point2f> pix;
        for (int y = 0; y < cm.rows; y += step)
            for (int x = 0; x < cm.cols; x += step)
                if (cm.at<uint8_t>(y, x) && roi.at<uint8_t>(y, x)) pix.emplace_back((float)(x + bb.x), (float)(y + bb.y));
        if (pix.size() < 12) continue;
        cv::Mat data((int)pix.size(), 2, CV_32F, pix.data());
        cv::PCA pca(data, cv::Mat(), cv::PCA::DATA_AS_ROW);
        cv::Point2f mean(pca.mean.at<float>(0, 0), pca.mean.at<float>(0, 1));
        cv::Point2f axis(pca.eigenvectors.at<float>(0, 0), pca.eigenvectors.at<float>(0, 1));
        double varMinor = pca.eigenvalues.rows > 1 ? pca.eigenvalues.at<float>(1) : 0.0;
        double tmin = 1e9, tmax = -1e9;
        for (auto& q : pix) {
            double t = (q.x - mean.x) * axis.x + (q.y - mean.y) * axis.y;
            tmin = std::min(tmin, t); tmax = std::max(tmax, t);
        }
        int nb = std::max(3, std::min(16, (int)((tmax - tmin) / 12.0)));
        std::vector<cv::Point2f> binSum(nb, cv::Point2f(0, 0));
        std::vector<int> binN(nb, 0);
        for (auto& q : pix) {
            double t = (q.x - mean.x) * axis.x + (q.y - mean.y) * axis.y;
            int b = std::min(nb - 1, (int)((t - tmin) / (tmax - tmin + 1e-6) * nb));
            binSum[b] += q; binN[b]++;
        }
        std::vector<LinePoint> pts;
        for (int b = 0; b < nb; ++b) {
            if (!binN[b]) continue;
            cv::Point2f c = binSum[b] * (1.0f / binN[b]);
            LinePoint lp;
            lp.u = c.x / scale_; lp.v = c.y / scale_;
            if (!proj.pixelToGround(lp.u, lp.v, lp.X, lp.Y)) continue;
            lp.widthCm = 3.46 * std::sqrt(std::max(0.0, varMinor)) / scale_ * proj.metersPerPixel(lp.u, lp.v) * 100.0;
            pts.push_back(lp);
        }
        if (pts.size() < 2) continue;
        auto dist0 = [](const LinePoint& q) { return std::sqrt(q.X * q.X + q.Y * q.Y); };
        if (dist0(pts.front()) > dist0(pts.back())) std::reverse(pts.begin(), pts.end());
        double len = std::hypot(pts.back().X - pts.front().X, pts.back().Y - pts.front().Y);
        double w = pts[pts.size() / 2].widthCm;
        if (len < 0.12 || w < 0.5 * p_.minWidthCm || w > 1.5 * p_.maxWidthCm) continue;
        if (dist0(pts.front()) > 2.2) continue;   // a segment start is never that far away
        // With an expected start (after a tag turn) the tail of the segment just finished can lie right under the
        // drone when the camera sees the floor beneath it: a short blob that starts under the drone is that tail,
        // never the next segment (which begins 80 cm from the tag the drone is hovering over).
        if (haveExpect && dist0(pts.front()) < 0.3 && len < 0.7) continue;
        double dExp = std::min(std::hypot(pts.front().X - expect.x, pts.front().Y - expect.y),
                               std::hypot(pts.back().X - expect.x, pts.back().Y - expect.y));
        double score = -dExp - (len < 0.3 ? 0.5 : 0.0) + 0.1 * std::min(len, 1.0);
        if (score > bestScore) {
            bestScore = score;
            best = LineObservation();
            best.valid = true; best.mode = 2; best.pts = pts;
            best.AX = pts.front().X; best.AY = pts.front().Y; best.BX = pts.back().X; best.BY = pts.back().Y;
            best.confidence = 0.5;
        }
    }
    if (bestScore <= -1e8) return false;
    obs = best;
    return true;
}

void LineDetector::finalize(LineObservation& obs) const {
    if (obs.pts.empty()) { obs.valid = false; return; }
    const LinePoint& n = obs.pts.front();
    const LinePoint& f = obs.pts.back();
    obs.nearX = n.X; obs.nearY = n.Y; obs.farX = f.X; obs.farY = f.Y;
    obs.lengthM = std::hypot(f.X - n.X, f.Y - n.Y);
    // heading from the near point to the first point at least 0.3 m further along the chain
    size_t k = obs.pts.size() - 1;
    for (size_t i = 1; i < obs.pts.size(); ++i) {
        if (std::hypot(obs.pts[i].X - n.X, obs.pts[i].Y - n.Y) >= 0.3) { k = i; break; }
    }
    if (k > 0) obs.headingDeg = mu::rad2deg(std::atan2(obs.pts[k].Y - n.Y, obs.pts[k].X - n.X));
    else obs.headingDeg = 0;
    // direction of the line at its visible end (used to know where the next segment will be)
    obs.endTangentValid = false;
    if (obs.endVisible && obs.pts.size() >= 4) {
        // principal direction of the last 3 chain points (short span: little lag on bends), curvature-corrected below
        size_t n0 = obs.pts.size() - 3;
        double mx = 0, my = 0; int m = 0;
        for (size_t i = n0; i < obs.pts.size(); ++i) { mx += obs.pts[i].X; my += obs.pts[i].Y; ++m; }
        mx /= m; my /= m;
        double sxx = 0, sxy = 0, syy = 0;
        for (size_t i = n0; i < obs.pts.size(); ++i) {
            double dx = obs.pts[i].X - mx, dy = obs.pts[i].Y - my;
            sxx += dx * dx; sxy += dx * dy; syy += dy * dy;
        }
        double ang = 0.5 * std::atan2(2 * sxy, sxx - syy);          // major axis of the point cloud
        double ex = std::cos(ang), ey = std::sin(ang);
        // orient along the chain (from the first of the points to the end)
        double dirX = obs.pts.back().X - obs.pts[n0].X, dirY = obs.pts.back().Y - obs.pts[n0].Y;
        if (ex * dirX + ey * dirY < 0) { ex = -ex; ey = -ey; }
        if (std::hypot(dirX, dirY) > 0.25) { obs.endTangentDeg = mu::rad2deg(std::atan2(ey, ex)); obs.endTangentValid = true; }
    }
    // curvature: least-squares Y = a X^2 + b X + c (band mode only, X monotonic)
    obs.curvature = 0;
    if (obs.mode == 1 && obs.pts.size() >= 4) {
        double sx = 0, sx2 = 0, sx3 = 0, sx4 = 0, sy = 0, sxy = 0, sx2y = 0;
        double x0 = n.X;
        int m = (int)obs.pts.size();
        for (auto& p : obs.pts) {
            double x = p.X - x0, y = p.Y;
            sx += x; sx2 += x * x; sx3 += x * x * x; sx4 += x * x * x * x;
            sy += y; sxy += x * y; sx2y += x * x * y;
        }
        cv::Matx33d A(sx4, sx3, sx2, sx3, sx2, sx, sx2, sx, (double)m);
        cv::Vec3d b(sx2y, sxy, sy);
        cv::Vec3d sol;
        if (cv::solve(A, b, sol, cv::DECOMP_SVD)) {
            double a = sol[0], bb = sol[1];
            obs.curvature = 2.0 * a / std::pow(1.0 + bb * bb, 1.5);
        }
    }
    if (obs.endTangentValid && obs.mode == 1 && obs.pts.size() >= 4) {
        // the 3-point direction is the chord of the last ~0.3 m: on a bend the tangent at the end is further along
        const LinePoint& a = obs.pts[obs.pts.size() - 3];
        const LinePoint& b = obs.pts.back();
        double span = std::hypot(b.X - a.X, b.Y - a.Y);
        obs.endTangentDeg = mu::wrapDeg(obs.endTangentDeg + mu::rad2deg(mu::clamp(obs.curvature, -3.0, 3.0) * 0.5 * span));
    }
}

LineObservation LineDetector::detect(const cv::Mat& bgr, const GroundProjector& proj,
                                     const std::vector<std::vector<cv::Point2f>>& maskPolys, const LineObservation* prev,
                                     const cv::Point2d* hint) {
    LineObservation obs;
    buildMask(bgr, proj, maskPolys);
    obs.maskPixels = cv::countNonZero(mask_);
    if (obs.maskPixels < p_.minComponentArea) return obs;
    bool bandsOk = scanBands(proj, prev, hint, obs);
    if (!bandsOk) {
        LineObservation comp;
        cv::Point2d expect = hint ? *hint : (prev && prev->valid) ? cv::Point2d(prev->nearX, prev->nearY) : cv::Point2d(0.8, 0.0);
        if (scanComponents(proj, comp, expect, hint != nullptr)) obs = comp;
    }
    if (obs.valid) finalize(obs);
    return obs;
}

void LineDetector::drawDebug(cv::Mat& frame, const LineObservation& obs, const GroundProjector& proj) const {
    double s = (double)frame.cols / proj.K().width;   // frame may be a scaled copy
    // horizon + band rows
    double hz = proj.horizonRow(proj.K().cx) * s;
    if (hz > 0 && hz < frame.rows) cv::line(frame, cv::Point(0, (int)hz), cv::Point(frame.cols, (int)hz), cv::Scalar(120, 120, 120), 1);
    for (auto& b : bandDbg_) {
        int row = (int)(b.row / scale_ * s);
        cv::line(frame, cv::Point(0, row), cv::Point(frame.cols, row), cv::Scalar(70, 70, 70), 1);
        for (size_t k = 0; k < b.runs.size(); ++k) {
            cv::Scalar col = ((int)k == b.chosen) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 200);
            cv::line(frame, cv::Point((int)(b.runs[k].first / scale_ * s), row), cv::Point((int)((b.runs[k].second + 1) / scale_ * s), row), col, 2);
        }
    }
    if (!obs.valid) return;
    cv::Scalar col = obs.mode == 1 ? cv::Scalar(0, 255, 255) : cv::Scalar(255, 0, 255);
    for (size_t i = 0; i < obs.pts.size(); ++i) {
        cv::Point p((int)(obs.pts[i].u * s), (int)(obs.pts[i].v * s));
        cv::circle(frame, p, 4, col, -1);
        if (i > 0) cv::line(frame, cv::Point((int)(obs.pts[i - 1].u * s), (int)(obs.pts[i - 1].v * s)), p, col, 2);
    }
    if (obs.endVisible) {
        double u, v;
        if (proj.groundToPixel(obs.endX, obs.endY, u, v)) cv::drawMarker(frame, cv::Point((int)(u * s), (int)(v * s)), cv::Scalar(0, 0, 255), cv::MARKER_TILTED_CROSS, 18, 2);
    }
    char buf[128];
    snprintf(buf, sizeof(buf), "line %s n=%zu y=%.2fm hdg=%.0f curv=%.2f len=%.1fm%s", obs.mode == 1 ? "bands" : "comp",
             obs.pts.size(), obs.nearY, obs.headingDeg, obs.curvature, obs.lengthM, obs.endVisible ? " END" : "");
    cv::putText(frame, buf, cv::Point(8, frame.rows - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, col, 1);
}
