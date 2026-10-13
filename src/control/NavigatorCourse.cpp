// Course mode of the navigator: the camera looks steeply down (mirror in front of the lens), so the tape is seen
// from above as centre-lines on the floor (Observation::paths). The drone follows the tape with a pure-pursuit
// law on those lines, flies over the tag at the end of each one and takes the nearest other tape end as the start
// of the next segment. It never turns: heading is held and the drone moves forwards, sideways or backwards
// (course.heading = fixed; with path or lead the nose turns along the route, the path following is the same).
#include "control/Navigator.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "util/Log.h"

void CourseParams::load(const Config& c) {
    std::vector<int> o;
    for (const auto& t : splitString(c.getString("course.tag_order", "1,2,3,4"), ',')) {
        int v = atoi(trim(t).c_str());
        if (v > 0) o.push_back(v);
    }
    if (!o.empty()) order = o;
    finishId = c.getInt("course.finish_id", order.back());
    vCruise = c.getDouble("course.v_cruise_ms", vCruise);
    vMin = c.getDouble("course.v_min_ms", vMin);
    vWaypoint = c.getDouble("course.v_waypoint_ms", vWaypoint);
    vClimb = c.getDouble("course.v_climb_ms", vClimb);
    vGap = c.getDouble("course.v_gap_ms", vGap);
    lookaheadM = c.getDouble("course.lookahead_m", lookaheadM);
    lookaheadPerMs = c.getDouble("course.lookahead_per_ms", lookaheadPerMs);
    accel = c.getDouble("course.accel_ms2", accel);
    latAccel = c.getDouble("course.lat_accel_ms2", latAccel);
    kCross = c.getDouble("course.k_cross", kCross);
    tagLookPastM = c.getDouble("course.tag_look_past_m", tagLookPastM);
    kBias = c.getDouble("course.k_bias", kBias);
    biasZoneM = c.getDouble("course.bias_zone_m", biasZoneM);
    calibAtFirstTag = c.getBool("course.calib_at_first_tag", calibAtFirstTag);
    mountPitchMinDeg = c.getDouble("course.mount_pitch_min_deg", mountPitchMinDeg);
    mountPitchMaxDeg = c.getDouble("course.mount_pitch_max_deg", mountPitchMaxDeg);
    traceHz = c.getDouble("course.trace_hz", traceHz);
    const std::string nose = toLower(c.getString("course.heading", headingLead ? "lead" : headingFollow ? "path" : "fixed"));
    headingLead = nose == "lead";
    headingFollow = headingLead || nose == "path";
    headingLookM = c.getDouble("course.heading_look_m", headingLookM);
    maxYawRc = c.getDouble("course.max_yaw_rc", maxYawRc);
    leadKYaw = c.getDouble("course.lead_k_yaw", leadKYaw);
    leadMaxYawRc = c.getDouble("course.lead_max_yaw_rc", leadMaxYawRc);
    leadLookM = c.getDouble("course.lead_look_m", leadLookM);
    leadLookS = c.getDouble("course.lead_look_s", leadLookS);
    leadSlowDeg = c.getDouble("course.lead_slow_deg", leadSlowDeg);
    leadSlowMin = c.getDouble("course.lead_slow_min", leadSlowMin);
    camAheadM = c.getDouble("course.camera_ahead_m", camAheadM);
    maxRangeM = c.getDouble("course.max_range_m", maxRangeM);
    biasMax = c.getDouble("course.bias_max_ms", biasMax);
    cornerBaseM = c.getDouble("course.corner_base_m", cornerBaseM);
    rcGain = c.getDouble("course.rc_gain_ms", rcGain);
    adaptGain = c.getBool("course.adapt_gain", adaptGain);
    respTau = c.getDouble("course.response_tau_s", respTau);
    latencyS = c.getDouble("course.latency_s", latencyS);
    cmdDelayS = c.getDouble("course.cmd_delay_s", cmdDelayS);
    shapeTau = c.getDouble("course.shape_tau_s", shapeTau);
    shapeMaxMs = c.getDouble("course.shape_max_ms", shapeMaxMs);
    tagCaptureM = c.getDouble("course.tag_capture_m", tagCaptureM);
    startCaptureM = c.getDouble("course.start_capture_m", startCaptureM);
    tagAssocM = c.getDouble("course.tag_assoc_m", tagAssocM);
    nextMinStartM = c.getDouble("course.next_min_start_m", nextMinStartM);
    arrivalSideCos = c.getDouble("course.arrival_side_cos", arrivalSideCos);
    nextSearchM = c.getDouble("course.next_search_m", nextSearchM);
    firstSearchM = c.getDouble("course.first_search_m", firstSearchM);
    lostCoastS = c.getDouble("course.lost_coast_s", lostCoastS);
    lostLandS = c.getDouble("course.lost_land_s", lostLandS);
    endWaitS = c.getDouble("course.end_wait_s", endWaitS);
    finishCenterM = c.getDouble("course.finish_center_m", finishCenterM);
    finishCenterMaxS = c.getDouble("course.finish_center_max_s", finishCenterMaxS);
    finishAltCm = c.getDouble("course.finish_altitude_cm", finishAltCm);
    finishDescentM = c.getDouble("course.finish_descent_m", finishDescentM);
    maxUd = c.getDouble("course.max_ud", maxUd);
    maxRc = c.getDouble("course.max_rc", maxRc);
    kYawHold = c.getDouble("course.k_yaw_hold", kYawHold);
}

namespace {
double norm(const cv::Point2d& v) { return std::hypot(v.x, v.y); }
cv::Point2d unit(const cv::Point2d& v) {
    double n = norm(v);
    return n > 1e-9 ? cv::Point2d(v.x / n, v.y / n) : cv::Point2d(0, 0);
}

// Telemetry heading at time t (the newest sample not later than t).
double yawAt(const std::deque<std::pair<double, int>>& h, double t, double fallback) {
    if (h.empty()) return fallback;
    for (auto it = h.rbegin(); it != h.rend(); ++it)
        if (it->first <= t) return it->second;
    return h.front().second;
}

// Is a floor point (body frame) inside the picture? Unknown view: assume it is.
bool inView(const Observation& obs, const cv::Point2d& q) {
    if (obs.view.size() != 4) return true;
    int pos = 0, neg = 0;
    for (int k = 0; k < 4; ++k) {
        const cv::Point2d a = obs.view[k], b = obs.view[(k + 1) % 4];
        const double cr = (b - a).cross(q - a);
        if (cr > 0) ++pos; else if (cr < 0) ++neg;
    }
    return pos == 0 || neg == 0;
}

// Polyline with arc lengths: "s" is the distance along it from its first point.
struct Poly {
    std::vector<cv::Point2d> p;
    std::vector<double> cum;
    Poly() = default;
    explicit Poly(const std::vector<cv::Point2d>& pts) : p(pts) { build(); }
    void build() {
        cum.assign(p.size(), 0.0);
        for (size_t i = 1; i < p.size(); ++i) cum[i] = cum[i - 1] + norm(p[i] - p[i - 1]);
    }
    double length() const { return cum.empty() ? 0.0 : cum.back(); }
    void reverse() { std::reverse(p.begin(), p.end()); build(); }
    cv::Point2d at(double s) const {
        if (p.empty()) return cv::Point2d(0, 0);
        if (s <= 0 || p.size() == 1) return p.front();
        if (s >= cum.back()) return p.back();
        size_t i = (size_t)(std::upper_bound(cum.begin(), cum.end(), s) - cum.begin());
        double span = cum[i] - cum[i - 1], f = span > 1e-9 ? (s - cum[i - 1]) / span : 0.0;
        return p[i - 1] + (p[i] - p[i - 1]) * f;
    }
    cv::Point2d tangent(double s) const {   // unit direction of increasing s, over a short span around s
        if (p.size() < 2) return cv::Point2d(1, 0);
        double a = mu::clamp(s - 0.04, 0.0, length()), b = mu::clamp(s + 0.04, 0.0, length());
        if (b - a < 1e-6) { a = std::max(0.0, length() - 0.08); b = length(); }
        return unit(at(b) - at(a));
    }
    // arc length of the point of the polyline nearest to q, searched between sLo and sHi
    double project(const cv::Point2d& q, double* dist = nullptr, double sLo = -1e9, double sHi = 1e9) const {
        double best = 1e18, bs = 0;
        if (p.size() == 1) { best = norm(q - p[0]); }
        for (size_t i = 1; i < p.size(); ++i) {
            if (cum[i] < sLo || cum[i - 1] > sHi) continue;
            cv::Point2d d = p[i] - p[i - 1];
            double l2 = d.dot(d), span = cum[i] - cum[i - 1];
            double t = l2 > 1e-12 ? (q - p[i - 1]).dot(d) / l2 : 0.0;
            double tLo = span > 1e-9 ? (sLo - cum[i - 1]) / span : 0.0, tHi = span > 1e-9 ? (sHi - cum[i - 1]) / span : 1.0;
            t = mu::clamp(t, std::max(0.0, tLo), std::min(1.0, tHi));
            double dd = norm(q - (p[i - 1] + d * t));
            if (dd < best) { best = dd; bs = cum[i - 1] + t * span; }
        }
        if (best > 1e17 && !p.empty()) {   // the window missed the polyline: clamp to its nearer end
            bs = sLo > length() ? length() : 0.0;
            best = norm(q - at(bs));
        }
        if (dist) *dist = best;
        return bs;
    }
};
}  // namespace

int Navigator::nextTagId() const {
    return cs_.k < (int)cp_.order.size() ? cp_.order[cs_.k] : 0;
}

double Navigator::speedMs() const { return flying() ? norm(cs_.vModel) : 0.0; }

bool Navigator::setCourseNoseMode(const std::string& name) {
    if (name != "fixed" && name != "path" && name != "lead") return false;
    cp_.headingLead = name == "lead";
    cp_.headingFollow = cp_.headingLead || name == "path";
    return true;
}

void Navigator::stepCourse(const Observation& obs, const TelloState& st) {
    if (phase_ == Phase::Idle || finished()) { landRequested_ = false; return; }
    if (landRequested_) { courseLand("land requested", Phase::Aborted); return; }
    if (p_.maxFlightS > 0 && missionTime() > p_.maxFlightS) {
        LOG_WARN("NAV max flight time reached -> landing");
        courseLand("max flight time", Phase::Aborted);
        return;
    }
    if (phase_ == Phase::Takeoff) courseTakeoff(st);
    else courseFly(obs, st);
}

void Navigator::courseLand(const char* why, Phase endPhase) {
    hover(true);
    courseRoute.clear();
    courseWaypoint = -1;
    if (endPhase == Phase::Landed) LOG_INFO("NAV landing, mission time %.1fs", missionTime());
    drone_.land();
    now_ = clock_.now();
    landRequested_ = false;
    if (endPhase == Phase::Landed) LOG_INFO("NAV landed. Total time %.1fs, tags %d", missionTime(), segmentsDone_);
    setPhase(endPhase, why);
}

void Navigator::courseTakeoff(const TelloState& st) {
    if (st.valid && st.bat >= 0 && st.bat < p_.minBattery) {
        LOG_ERROR("NAV battery %d%% below minimum %d%% -> not taking off", st.bat, p_.minBattery);
        setPhase(Phase::Aborted, "low battery");
        return;
    }
    const bool wasFlying = drone_.isFlying();
    if (!wasFlying && !drone_.takeoff()) {
        setPhase(Phase::Aborted, "takeoff failed");
        return;
    }
    missionStart_ = clock_.now();
    now_ = missionStart_;
    cs_ = CourseState();
    cs_.gain = cp_.rcGain;
    cs_.lastT = cs_.stageStart = cs_.lastRouteT = now_;
    // the first frames after the blocking take-off still show the view from before it
    cs_.blindUntil = now_ + (wasFlying ? 0.0 : p_.postCommandBlindS);
    TelloState s = drone_.state();
    if (s.valid) { cs_.yaw0 = cs_.lastYaw = s.yaw; cs_.yawRef = s.yaw; cs_.yaw0Valid = cs_.yawValid = true; }
    visited_.clear();
    segmentsDone_ = 0;
    hover(true);
    setPhase(Phase::Acquire, wasFlying ? "from hover: looking for the line" : "airborne: looking for the line");
}

// velocity in the body frame (m/s: x forward, y left) -> sticks, plus height and heading hold
void Navigator::courseCommand(const cv::Point2d& vel, const TelloState& st) {
    CourseState& c = cs_;
    const double g = std::max(1e-4, c.gain);
    cv::Point2d v = vel;
    if (cp_.shapeTau > 0 && cp_.respTau > cp_.shapeTau) {
        // The drone takes respTau to follow a command. Asking for more than is wanted, by what is still missing,
        // makes it answer as if it took shapeTau.
        cv::Point2d extra = (vel - c.bias - c.vPred) * (cp_.respTau / cp_.shapeTau - 1.0);
        const double en = norm(extra);
        if (en > cp_.shapeMaxMs) extra *= cp_.shapeMaxMs / en;
        v += extra;
    }
    if (cp_.headingFollow && st.valid && !c.yawHist.empty()) {
        // the direction was worked out in the picture, and the nose has turned since the picture was taken
        // ... and will have turned further by the time this command acts
        const double d = mu::deg2rad(mu::wrapDeg(st.yaw + c.yawRate * cp_.cmdDelayS - yawAt(c.yawHist, now_ - cp_.latencyS, st.yaw)));
        v = cv::Point2d(v.x * std::cos(d) - v.y * std::sin(d), v.x * std::sin(d) + v.y * std::cos(d));
    }
    double fb = mu::clamp(v.x / g, -cp_.maxRc, cp_.maxRc), lr = mu::clamp(-v.y / g, -cp_.maxRc, cp_.maxRc);
    double ud = 0, h = st.heightCm();
    if (h > 0) {
        const double want = (c.finalDescent && cp_.finishAltCm > 0 ? std::min(cp_.finishAltCm, p_.altitudeCm) : p_.altitudeCm) + c.altBoostCm;
        double err = want - h;
        if (std::fabs(err) >= p_.altDeadCm) ud = mu::clamp(p_.kAlt * err, -cp_.maxUd, cp_.maxUd);
    }
    double yaw = 0;
    if (c.yaw0Valid && st.valid && !c.yawHoldOff) {
        const double err = mu::wrapDeg(c.yawRef - st.yaw);
        if (cp_.headingLead) {
            yaw = mu::clamp(cp_.leadKYaw * err, -cp_.leadMaxYawRc, cp_.leadMaxYawRc);
        } else if (cp_.headingFollow) {
            yaw = mu::clamp(cp_.kYawHold * err, -cp_.maxYawRc, cp_.maxYawRc);
        } else if (std::fabs(err) > 25.0) {
            // held, and still 25 degrees away: the hold is pushing the wrong way or cannot keep up. Without it the drone
            // simply keeps the heading it has (the navigation works at any heading).
            c.yawHoldOff = true;
            LOG_WARN("NAV heading hold switched off: %.0f deg from the take-off heading while being held", err);
        } else {
            yaw = mu::clamp(cp_.kYawHold * err, -30.0, 30.0);
        }
    }
    sendRc(lr, fb, ud, yaw);
    c.vCmd = cv::Point2d(rc_.fb * g, -rc_.lr * g);   // what was really sent (after the slew limit)
    c.cmdHist.emplace_back(now_, c.vCmd - c.bias);
    while (c.cmdHist.size() > 2 && c.cmdHist.front().first < now_ - cp_.cmdDelayS - 1.0) c.cmdHist.pop_front();
}

void Navigator::courseTagReached(int id, const cv::Point2d& pos, const Observation& obs, int activeLine) {
    CourseState& c = cs_;
    const int expected = nextTagId();
    visited_.insert(id);
    ++segmentsDone_;
    bool known = false;
    for (size_t i = 0; i < cp_.order.size(); ++i)
        if (cp_.order[i] == id) { c.k = (int)i + 1; known = true; }
    if (!known) ++c.k;
    if (id != expected) LOG_WARN("NAV flew over tag ID%d where ID%d was expected: continuing from it", id, expected);
    char buf[160];
    snprintf(buf, sizeof(buf), "t=%6.2f  tag ID%d reached (%d of %zu)", missionTime(), id, segmentsDone_, cp_.order.size());
    LOG_INFO("NAV %s", buf);
    events.push_back(buf);
    if (events.size() > 12) events.erase(events.begin());
    c.anchor = pos;
    c.anchorTag = id;
    c.goalTag = -1;
    c.arrivalDir = c.travelDir;
    // Which side of the tag the drone came from: towards the point of the tape 15 cm behind it. That does not depend
    // on how the drone was pointing or moving when the tag was reached, nor on how the end of the tape came out.
    c.arrivalFrom = c.havePrev && norm(c.prevBehind - pos) > 0.10 ? unit(c.prevBehind - pos) : c.travelDir * -1.0;
    c.endSince = -1;
    if (id == cp_.finishId || c.k >= (int)cp_.order.size()) {
        c.stage = CourseState::Finish;
        c.stageStart = now_;
        setPhase(Phase::Finish, "finish tag reached");
        return;
    }
    if (cp_.calibAtFirstTag && !courseCalibDone_ && obs.pathMode && !obs.mountTrusted) {
        // The mirror angle is still the configured guess. Before anything is aimed at across a gap, measure it: the
        // drone moves a hand-span around the tag so the tag is seen all over the picture (about 5 s, first flight only).
        c.stage = CourseState::Calib;
        c.stageStart = c.calibLegStart = now_;
        c.calibLeg = 0;
        LOG_INFO("NAV camera angle not measured yet: moving around tag ID%d to measure it", id);
        setPhase(Phase::TagManeuver, "measuring the camera angle over the tag");
        return;
    }
    if (courseChooseNext(obs, activeLine)) return;
    c.stage = CourseState::SeekNext;
    c.stageStart = now_;
    c.seekRises = 0; c.seekOverT = 0;
    setPhase(Phase::Search, "looking for the next line");
}

// The next segment starts at the tape end nearest to a tag (anchor), the tape the drone comes on excepted.
bool Navigator::courseFindNext(const Observation& obs, int excludeLine, const cv::Point2d& anchor, const cv::Point2d& arrivalDir, const cv::Point2d& arrivalFrom,
                               int& line, int& end, double& dist) const {
    const CourseState& c = cs_;
    int best = -1, bestEnd = 0;
    double bd = cp_.nextSearchM;
    for (size_t i = 0; i < obs.paths.size(); ++i) {
        if ((int)i == excludeLine) continue;
        const PathLine& L = obs.paths[i];
        if (L.pts.size() < 2) continue;
        for (int e = 0; e < 2; ++e) {
            if (e ? L.cutB : L.cutA) continue;                          // the camera does not see where this tape starts
            const cv::Point2d p = e ? L.pts.back() : L.pts.front();
            const double d = norm(p - anchor);
            if (d >= bd) continue;
            if (d < cp_.nextMinStartM) continue;                        // inside the tag: its dark cells traced as a piece of tape
            // the tape the drone came on, whole or in pieces, lies on the side it came from
            if (cp_.arrivalSideCos > 0 && d > 0.02 && unit(p - anchor).dot(arrivalFrom) > cp_.arrivalSideCos) continue;
            const cv::Point2d into = (e ? L.tangentB() : L.tangentA()) * -1.0;   // which way the tape runs from this end
            const bool behind = unit(p - anchor).dot(arrivalDir) < -0.85;
            if ((d < 0.35 || behind) && into.dot(arrivalDir) < -0.75) continue;   // a piece of the tape we arrive on
            // a tape that starts at a tag already visited, or whose other end sits on one, is a segment already flown
            const cv::Point2d other = e ? L.pts.front() : L.pts.back();
            // the next segment starts near the tag and runs away from it. An end from which the tape comes TOWARDS the
            // tag is the far end of a piece that ends here (the tape we arrived on, cut in two by a crossing tape or by glare)
            if (norm(other - anchor) < d + 0.05) continue;
            bool flown = false;
            for (const auto& t : c.tags) {
                if (!visited_.count(t.id) || norm(t.p - anchor) < 0.25) continue;
                if (norm(t.p - p) < cp_.tagAssocM) flown = true;
                if (!(e ? L.cutA : L.cutB) && norm(t.p - other) < cp_.tagAssocM) flown = true;
            }
            if (flown) continue;
            bd = d; best = (int)i; bestEnd = e;
        }
    }
    if (best < 0) return false;
    line = best; end = bestEnd; dist = bd;
    return true;
}

bool Navigator::courseChooseNext(const Observation& obs, int excludeLine) {
    CourseState& c = cs_;
    int best = -1, bestEnd = 0;
    double bd = 0;
    if (!courseFindNext(obs, excludeLine, c.anchor, c.arrivalDir, c.arrivalFrom, best, bestEnd, bd)) return false;
    const PathLine& L = obs.paths[best];
    c.startPos = bestEnd ? L.pts.back() : L.pts.front();
    c.haveStart = true;
    c.startCaptured = false;
    c.havePrev = false;
    c.stage = CourseState::Line;
    c.stageStart = now_;
    c.lastRouteT = now_;
    c.altBoostCm = 0;
    LOG_INFO("NAV next line starts %.2f m from the tag, at (%.2f, %.2f) m", bd, c.startPos.x, c.startPos.y);
    setPhase(Phase::GapCoast, "to the start of the next line");
    return true;
}

void Navigator::courseFly(const Observation& obs, const TelloState& st) {
    CourseState& c = cs_;
    const double dt = mu::clamp(now_ - c.lastT, 0.0, 0.5);
    c.lastT = now_;
    const bool newFrame = obs.pathMode;

    // ---- how the drone moved: the stick commands seen through its response; heading from the telemetry
    const double tauR = std::max(0.05, cp_.respTau);
    auto sentAt = [&](double t) {             // the command that was on its way at time t
        cv::Point2d u(0, 0);
        for (const auto& e : c.cmdHist) { if (e.first > t) break; u = e.second; }
        return u;
    };
    double dyaw = 0;
    if (st.valid) {
        if (c.yawValid) dyaw = mu::wrapDeg(st.yaw - c.lastYaw);
        c.lastYaw = st.yaw;
        c.yawValid = true;
        c.yawHist.emplace_back(now_, st.yaw);
        while (c.yawHist.size() > 2 && c.yawHist.front().first < now_ - 2.0) c.yawHist.pop_front();
    }
    // With the nose turning, a stick command acts along wherever the nose points when it acts, and what the drone
    // already moves at keeps its direction over the floor. So the model lives in a frame that does not turn (the
    // telemetry heading says how the nose stands in it); with the nose fixed the two frames are the same.
    const bool turning = cp_.headingFollow && st.valid;
    c.yawRate = turning && c.yawHist.size() >= 2 ? mu::wrapDeg(st.yaw - yawAt(c.yawHist, now_ - 0.3, st.yaw)) / 0.3 : 0.0;
    auto psi = [&](double t) { return !turning ? 0.0 : t <= now_ ? yawAt(c.yawHist, t, st.yaw) : st.yaw + c.yawRate * (t - now_); };
    auto turn = [](const cv::Point2d& p, double deg) {
        const double a = mu::deg2rad(deg);
        return cv::Point2d(p.x * std::cos(a) - p.y * std::sin(a), p.x * std::sin(a) + p.y * std::cos(a));
    };
    c.vW += (turn(cp_.cmdDelayS > 0 ? sentAt(now_ - cp_.cmdDelayS - 0.5 * dt) : c.vCmd - c.bias, -psi(now_ - 0.5 * dt)) - c.vW) * std::min(1.0, dt / tauR);
    c.vModel = turn(c.vW, psi(now_));
    c.vHist.emplace_back(now_, c.vW);
    while (c.vHist.size() > 2 && c.vHist.front().first < now_ - 1.5) c.vHist.pop_front();
    const double tImg = now_ - cp_.latencyS;                 // when the image being used was taken
    cv::Point2d vThen = c.vHist.front().second, pd(0, 0);    // velocity then; how far the drone went since
    {
        double tPrev = tImg;
        cv::Point2d vPrev = vThen;
        for (const auto& e : c.vHist) {
            if (e.first <= tImg) { vThen = vPrev = e.second; continue; }
            pd += vPrev * (e.first - tPrev);
            tPrev = e.first;
            vPrev = e.second;
        }
        pd += vPrev * std::max(0.0, now_ - tPrev);
    }
    // What is sent now acts cmdDelayS from now: the commands already on their way say where the drone will be then.
    c.vPred = c.vW;
    for (double s = 0; s < cp_.cmdDelayS - 1e-9; s += 0.02) {
        const double h = std::min(0.02, cp_.cmdDelayS - s);
        pd += c.vPred * h;
        c.vPred += (turn(sentAt(now_ + s - cp_.cmdDelayS), -psi(now_ + s)) - c.vPred) * std::min(1.0, h / tauR);
    }
    // Lead: the nose turns fast, so what is remembered from earlier pictures turns by as much as the nose turned
    // between the two pictures (not between the two newest telemetry packets: the picture is older, and the turn
    // rate changes quickly).
    const bool noseLead = turning && cp_.headingLead;
    if (noseLead) {
        const double pI = psi(tImg);
        dyaw = c.psiImgValid ? mu::wrapDeg(pI - c.lastPsiImg) : 0.0;
        c.lastPsiImg = pI;
        c.psiImgValid = true;
    }
    // ... all of it as seen in the picture being used
    pd = turn(pd, psi(tImg));
    vThen = turn(vThen, psi(tImg));
    c.vPred = turn(c.vPred, psi(tImg));
    // Everything seen is measured from the point straight under the camera (the mirror, at the nose). What has to be
    // on the tape and over the tags is the drone itself: its centre lies behind that point.
    pd.x -= cp_.camAheadM;
    const double ca = std::cos(mu::deg2rad(dyaw)), sa = std::sin(mu::deg2rad(dyaw));
    auto rot = [&](const cv::Point2d& p) { return cv::Point2d(p.x * ca - p.y * sa, p.x * sa + p.y * ca); };
    // floor point: previous image frame -> this one. Lead: the drone turns about its own centre, and everything seen
    // is measured from the point under the camera, which swings round that centre on an arm of camAheadM.
    const cv::Point2d arm(cp_.camAheadM, 0.0);
    auto move = [&](const cv::Point2d& p) { return noseLead ? rot(p + arm - vThen * dt) - arm : rot(p - vThen * dt); };
    for (auto& t : c.tags) { t.p = move(t.p); t.fresh = false; }
    if (c.havePrev) { c.prevTarget = move(c.prevTarget); c.prevBehind = move(c.prevBehind); }
    if (c.haveStart) c.startPos = move(c.startPos);
    c.anchor = move(c.anchor);
    c.travelDir = rot(c.travelDir);
    c.arrivalDir = rot(c.arrivalDir);
    c.arrivalFrom = rot(c.arrivalFrom);
    bool recalibrated = false;
    if (newFrame && obs.mountChanged) {      // the camera angle was corrected: everything remembered is seen elsewhere now
        const cv::Point2d s = obs.mountShift;
        for (auto& t : c.tags) t.p += s;
        if (c.havePrev) { c.prevTarget += s; c.prevBehind += s; }
        if (c.haveStart) c.startPos += s;
        c.anchor += s;
        recalibrated = norm(s) > 0.02;
        if (norm(s) > 0.05) LOG_INFO("NAV camera angle corrected: the floor under the drone moved by (%.2f, %.2f) m", s.x, s.y);
    }
    c.fromStart = rot(c.fromStart - c.vModel * dt);          // the take-off point as seen from the drone
    if (cp_.maxRangeM > 0 && norm(c.fromStart) > cp_.maxRangeM) {
        LOG_WARN("NAV %.1f m from the take-off point: beyond course.max_range_m -> landing", norm(c.fromStart));
        courseLand("too far from the start", Phase::Aborted);
        return;
    }

    if (now_ < c.blindUntil) { courseCommand(cv::Point2d(0, 0), st); return; }
    if (!newFrame) {   // no new image in this cycle: keep doing the same thing, briefly
        double lost = now_ - c.lastRouteT;
        courseCommand(lost < cp_.lostCoastS ? c.vCmd : cv::Point2d(0, 0), st);
        if (lost > cp_.lostLandS) courseLand("no video", Phase::Aborted);
        return;
    }

    // ---- tags: positions on the floor, remembered while out of sight; they also measure the true speed
    if (obs.tagsFresh) {
        double rSum = 0;
        int rN = 0;
        for (const auto& t : obs.tags) {
            const cv::Point2d p(t.X, t.Y);
            CourseState::TagMem* m = nullptr;
            for (auto& q : c.tags) if (q.id == t.id) m = &q;
            if (!m) { c.tags.push_back({t.id, p, now_, true, p, now_}); continue; }
            const double age = now_ - m->seen;
            const double vm2 = vThen.dot(vThen);
            if (!recalibrated && age > 0.02 && age < 0.25 && vm2 > 0.15 * 0.15 && norm(c.vCmd - vThen) < 0.10) {
                // m->p is where the model put the tag; the difference to where it is seen is the speed error
                const cv::Point2d vTrue = vThen - (p - m->p) * (1.0 / age);
                rSum += vTrue.dot(vThen) / vm2;
                ++rN;
            }
            m->p = p; m->seen = now_; m->fresh = true;
        }
        if (rN > 0) {
            const double r = rSum / rN;                      // true speed / modelled speed
            if (r < -0.3) ++c.reversedCount; else c.reversedCount = std::max(0, c.reversedCount - 1);
            if (c.reversedCount >= 8) {
                LOG_ERROR("NAV the drone moves AGAINST the commands (ratio %.2f): the camera image is turned around. "
                          "Check camera.flip: with the mirror the floor AHEAD of the drone must be at the top of the picture.", r);
                courseLand("image reversed: check camera.flip", Phase::Aborted);
                return;
            }
            if (cp_.adaptGain && r > 0.2 && r < 5.0) {
                c.gain = mu::clamp(c.gain * mu::clamp(1.0 + 0.05 * (r - 1.0), 0.93, 1.07), 0.4 * cp_.rcGain, 2.5 * cp_.rcGain);
                if (++c.gainSamples % 40 == 0) LOG_INFO("NAV measured stick gain: %.4f m/s per unit (configured %.4f)", c.gain, cp_.rcGain);
            }
        }
    }
    for (auto it = c.tags.begin(); it != c.tags.end();)
        if (now_ - it->seen > 4.0) it = c.tags.erase(it); else ++it;

    std::vector<Poly> polys;
    polys.reserve(obs.paths.size());
    for (const auto& L : obs.paths) polys.emplace_back(L.pts);
    courseRoute.clear();
    courseWaypoint = -1;
    haveTarget = false;

    // ---- FIRST: the nearest tape, followed the way the drone faces
    if (c.stage == CourseState::First) {
        int best = -1;
        double bd = cp_.firstSearchM, bs = 0;
        for (size_t i = 0; i < polys.size(); ++i) {
            double d, s = polys[i].project(pd, &d);
            if (d < bd) { bd = d; best = (int)i; bs = s; }
        }
        if (best < 0) {
            // The drone is put down nose along the line: with a mirror that looks backwards more than expected the
            // tape may begin just ahead of what the camera sees. Creep forward for a while before giving up.
            const double waited = now_ - c.stageStart;
            const bool creep = !c.havePrev && waited > 1.0 && waited < 5.0;
            courseCommand(creep ? cv::Point2d(0.10, 0) : cv::Point2d(0, 0), st);
            if (now_ - c.stageStart > cp_.lostLandS + 3.0) {
                LOG_WARN("NAV no tape of the chosen colour in sight after take-off -> landing");
                courseLand("no line found", Phase::Aborted);
            }
            return;
        }
        const Poly& P = polys[best];
        const PathLine& L = obs.paths[best];
        const cv::Point2d t = P.tangent(bs);
        bool fwd = c.havePrev ? t.dot(c.travelDir) >= 0 : t.x >= 0;   // re-acquiring: keep going the same way
        if (!c.havePrev) {   // standing on the very beginning of the tape: away from that end
            const double dA = bs, dB = P.length() - bs;
            if (!L.cutA && dA < 0.35 && dB > dA + 0.2) fwd = true;
            else if (!L.cutB && dB < 0.35 && dA > dB + 0.2) fwd = false;
        }
        const double sg = fwd ? 1.0 : -1.0;
        c.prevTarget = P.at(bs + sg * 0.25);
        c.prevBehind = P.at(bs - sg * 0.12);
        c.travelDir = t * sg;
        c.havePrev = true;
        c.haveStart = false;
        c.startCaptured = true;
        c.stage = CourseState::Line;
        c.stageStart = c.lastRouteT = now_;
        setPhase(Phase::Follow, "line found");
    }

    // ---- CALIB: move around the tag just reached so that it is seen all over the picture
    if (c.stage == CourseState::Calib) {
        // where the drone goes, relative to the tag (m, body frame): ahead of it, just behind it, to its left and right
        static const cv::Point2d legs[] = {{0.22, 0.0}, {-0.06, 0.0}, {0.08, 0.22}, {0.08, -0.22}, {0.0, 0.0}};
        const int nLegs = 5;
        for (const auto& t : c.tags) if (t.id == c.anchorTag && now_ - t.seen < 0.5) c.anchor = t.p;
        if (obs.mountTrusted && c.calibLeg < nLegs - 1) { c.calibLeg = nLegs - 1; c.calibLegStart = now_; }   // measured: back over the tag
        const cv::Point2d e = c.anchor + legs[std::min(c.calibLeg, nLegs - 1)] - pd;
        const double d = norm(e);
        // the way points only have to be passed near; the last one (back over the tag) is flown exactly
        const bool lastLeg = c.calibLeg >= nLegs - 1;
        if (d < (lastLeg ? 0.05 : 0.09) || now_ - c.calibLegStart > (lastLeg ? 2.5 : 1.8)) { ++c.calibLeg; c.calibLegStart = now_; }
        if (c.calibLeg >= nLegs || now_ - c.stageStart > 12.0) {
            courseCalibDone_ = true;
            if (obs.mountTrusted && (obs.mountPitchDeg > cp_.mountPitchMaxDeg || obs.mountPitchDeg < cp_.mountPitchMinDeg)) {
                // with the mirror this far off the camera sees next to nothing on one side of the drone: flying on would
                // mean flying blind. Better to land here, on the tag, and say why.
                LOG_ERROR("NAV the mirror makes the camera look %.0f deg down (90 = straight down): it sees almost nothing %s the drone. "
                          "Re-seat the clip (96-99 deg expected) and start again.", obs.mountPitchDeg,
                          obs.mountPitchDeg > cp_.mountPitchMaxDeg ? "ahead of" : "behind");
                courseLand("mirror angle out of range: re-seat the clip", Phase::Aborted);
                return;
            }
            LOG_INFO("NAV camera angle %s: going on", obs.mountTrusted ? "measured" : "not settled yet (it keeps being refined in flight)");
            if (!courseChooseNext(obs, -1)) {
                c.stage = CourseState::SeekNext;
                c.stageStart = now_;
                c.seekRises = 0; c.seekOverT = 0;
                setPhase(Phase::Search, "looking for the next line");
            }
            return;
        }
        targetX = c.anchor.x; targetY = c.anchor.y; haveTarget = true;
        courseCommand((d > 0.02 ? e * (std::min(0.35, (lastLeg ? 1.5 : 3.0) * d) / d) : cv::Point2d(0, 0)) + c.bias, st);
        return;
    }

    // ---- SEEK NEXT: hold over the tag until the start of the next tape is in sight
    if (c.stage == CourseState::SeekNext) {
        // the tag itself says where it is while it is in sight: without that the drone would hold a remembered point
        // and be carried off by any drift
        for (const auto& t : c.tags) if (t.id == c.anchorTag && now_ - t.seen < 0.5) c.anchor = t.p;
        if (!courseChooseNext(obs, -1)) {
            // nose following the path: look around. The camera sees further behind the drone than ahead of it, so a
            // slow turn on the spot brings every direction into the long side of the picture.
            if (cp_.headingFollow && now_ - c.stageStart > 0.6) c.yawRef = mu::wrapDeg(c.yawRef + 30.0 * dt);
            const cv::Point2d e = c.anchor - pd;
            const double d = norm(e);
            courseCommand((d > 0.03 ? e * (std::min(0.25, 1.5 * d) / d) : cv::Point2d(0, 0)) + c.bias, st);
            const double waited = now_ - c.stageStart;
            // The next tape starts next to the tag: first get back over the tag (the drone may have run past it),
            // and only then go up for a wider view.
            if (d < 0.15) c.seekOverT += dt;
            if (c.seekRises < 2 && (c.seekOverT > 1.0 * (c.seekRises + 1) || waited > 3.5 + 1.2 * c.seekRises)) {
                ++c.seekRises;
                c.altBoostCm += 30;
                LOG_INFO("NAV no further tape in sight: rising 30 cm for a wider view");
            }
            if (waited > cp_.lostLandS) {
                if (c.anchorTag < 0) {
                    LOG_INFO("NAV end of the path: the tape ends without a tag and no further tape is in sight");
                    courseLand("end of the path", Phase::Landed);
                } else {
                    LOG_WARN("NAV no next line found near the tag -> landing");
                    courseLand("no next line", Phase::Aborted);
                }
            }
            return;
        }
    }

    // ---- FINISH: centre over the last tag, then land on it
    if (c.stage == CourseState::Finish) {
        c.finalDescent = cp_.finishAltCm > 0;
        cv::Point2d target = c.anchor;
        for (const auto& t : c.tags) if (t.id == c.anchorTag && now_ - t.seen < 1.5) target = t.p;
        const cv::Point2d e = target - pd;
        const double d = norm(e);
        if ((d < cp_.finishCenterM && norm(c.vModel) < 0.08) || now_ - c.stageStart > cp_.finishCenterMaxS) {
            courseLand("landed", Phase::Landed);
            return;
        }
        targetX = target.x; targetY = target.y; haveTarget = true;
        if (d < 0.15) {
            c.bias += e * (cp_.kBias * dt);
            const double bn = norm(c.bias);
            if (bn > cp_.biasMax) c.bias *= cp_.biasMax / bn;
        }
        courseCommand((d > 0.01 ? e * (std::min(0.18, 1.5 * d) / d) : cv::Point2d(0, 0)) + c.bias, st);
        return;
    }

    // ---- LINE 1: the tape being followed, oriented the way we travel
    bool leadIn = c.haveStart && !c.startCaptured;
    if (leadIn && now_ - c.stageStart > 6.0) {      // never got there: look again from where we are
        LOG_WARN("NAV the start of the next line was not reached in 6 s: looking for it again");
        c.anchor = pd;
        c.stage = CourseState::SeekNext;
        c.stageStart = now_;
        c.haveStart = false;
        courseCommand(cv::Point2d(0, 0), st);
        return;
    }
    int li = -1;
    Poly pl;
    bool cutEnd = false;
    double sPrev = 0;                         // arc length on pl of the previous look-ahead point
    if (leadIn) {
        // the tape whose end is where the start was seen (it is followed from frame to frame, so it has not moved far);
        // an end of the tape we came on does not qualify, however close it lies
        auto ours = [&](const cv::Point2d& endPt, const cv::Point2d& into) {
            const cv::Point2d r = endPt - c.anchor;
            if (cp_.arrivalSideCos > 0 && norm(r) > 0.02 && unit(r).dot(c.arrivalFrom) > cp_.arrivalSideCos) return true;
            return (norm(r) < 0.35 || unit(r).dot(c.arrivalDir) < -0.85) && into.dot(c.arrivalDir) < -0.75;
        };
        double bd = 0.20;
        bool rev = false;
        for (size_t i = 0; i < polys.size(); ++i) {
            if (polys[i].p.size() < 2) continue;
            const PathLine& L = obs.paths[i];
            const double dA = norm(polys[i].p.front() - c.startPos), dB = norm(polys[i].p.back() - c.startPos);
            const double rA = norm(L.pts.front() - c.anchor), rB = norm(L.pts.back() - c.anchor);
            if (dA < bd && rB > rA + 0.05 && !ours(L.pts.front(), L.tangentA() * -1.0)) { bd = dA; li = (int)i; rev = false; }
            if (dB < bd && rA > rB + 0.05 && !ours(L.pts.back(), L.tangentB() * -1.0)) { bd = dB; li = (int)i; rev = true; }
        }
        if (li >= 0) {
            pl = polys[li];
            if (rev) pl.reverse();
            c.startPos = pl.p.front();
            cutEnd = rev ? obs.paths[li].cutA : obs.paths[li].cutB;
            double dOn = 1e9;
            const double sOnTape = pl.project(pd, &dOn);
            LOG_DEBUG("LEADIN t=%.2f li=%d rev=%d start=(%.2f,%.2f) pd=(%.2f,%.2f) dStart=%.2f sOn=%.2f dOn=%.2f len=%.2f anchor=(%.2f,%.2f) arr=(%.2f,%.2f) other=(%.2f,%.2f)",
                      missionTime(), li, rev ? 1 : 0, c.startPos.x, c.startPos.y, pd.x, pd.y, norm(c.startPos - pd), sOnTape, dOn, pl.length(), c.anchor.x, c.anchor.y,
                      c.arrivalDir.x, c.arrivalDir.y, pl.p.back().x, pl.p.back().y);
            if (norm(c.startPos - pd) < cp_.startCaptureM || (sOnTape > 0.03 && dOn < 0.05)) {   // on the new tape: follow it away from this end
                c.startCaptured = true;
                c.haveStart = false;
                sPrev = std::min(std::max(0.15, sOnTape + 0.10), pl.length());
                c.prevTarget = pl.at(sPrev);
                c.prevBehind = pl.p.front();
                c.travelDir = pl.tangent(0.05);
                c.havePrev = true;
                leadIn = false;
                setPhase(Phase::Follow, "on the next line");
            }
        }
    } else if (c.havePrev) {
        // The tape followed a moment ago: it passes close to where the preview point was AND close to the point behind
        // the drone. Both, because another tape may end a hand-span away (the one we arrived on, next to a tag).
        double bd = 0.25, bestCost = 1e9;
        for (size_t i = 0; i < polys.size(); ++i) {
            double d, dBehind, s = polys[i].project(c.prevTarget, &d);
            if (d >= 0.15) continue;
            polys[i].project(c.prevBehind, &dBehind);
            if (dBehind >= 0.30) continue;
            const double cost = d + 0.5 * dBehind;
            if (cost < bestCost) { bestCost = cost; bd = d; li = (int)i; sPrev = s; }
        }
        bool fwd = true;
        if (li >= 0) {
            const Poly& P = polys[li];
            double dB, sB = P.project(c.prevBehind, &dB);
            if (dB < 0.30 && std::fabs(sPrev - sB) > 0.04) fwd = sPrev > sB;
            else {
                const double sD = P.project(pd);
                if (std::fabs(sPrev - sD) > 0.04) fwd = sPrev > sD;
                else fwd = P.tangent(sPrev).dot(c.travelDir) >= 0;
            }
        } else if (now_ - c.lastRouteT > 0.3) {
            // The thread was lost for more than a moment (a tape missing from a single picture is simply waited
            // for, coasting): take the nearest tape that runs our way
            bd = 0.30;
            for (size_t i = 0; i < polys.size(); ++i) {
                double d, s = polys[i].project(pd, &d);
                if (d < bd && std::fabs(polys[i].tangent(s).dot(c.travelDir)) > 0.5) { bd = d; li = (int)i; sPrev = s; }
            }
            if (li >= 0) fwd = polys[li].tangent(sPrev).dot(c.travelDir) >= 0;
        }
        if (li >= 0) {
            // a tape is never followed back to a tag already flown over: if that is where this direction leads
            // (and the other does not), the direction got turned around at a tape end -> the other way
            const cv::Point2d endFar = fwd ? polys[li].p.back() : polys[li].p.front(), endNear = fwd ? polys[li].p.front() : polys[li].p.back();
            const bool farCut = fwd ? obs.paths[li].cutB : obs.paths[li].cutA;
            bool farVisited = false, nearVisited = false;
            for (const auto& t : c.tags) {
                if (!visited_.count(t.id) || now_ - t.seen > 2.5) continue;
                if (norm(t.p - endFar) < cp_.tagAssocM) farVisited = true;
                if (norm(t.p - endNear) < cp_.tagAssocM) nearVisited = true;
            }
            if (farVisited && !nearVisited && !farCut) {
                LOG_INFO("NAV this way leads back to a tag already flown over: following the tape the other way");
                for (const auto& t : c.tags)
                    LOG_DEBUG("GUARD tag %d visited=%d age=%.1f p=(%.2f,%.2f) far=(%.2f,%.2f) d=%.2f near=(%.2f,%.2f) d=%.2f len=%.2f sPrev=%.2f fwd=%d pd=(%.2f,%.2f)",
                              t.id, (int)visited_.count(t.id), now_ - t.seen, t.p.x, t.p.y, endFar.x, endFar.y, norm(t.p - endFar), endNear.x, endNear.y,
                              norm(t.p - endNear), polys[li].length(), sPrev, fwd ? 1 : 0, pd.x, pd.y);
                fwd = !fwd;
            }
            pl = polys[li];
            if (!fwd) { pl.reverse(); sPrev = pl.length() - sPrev; }
            cutEnd = fwd ? obs.paths[li].cutB : obs.paths[li].cutA;
        }
    }

    // ---- LINE 2: the route ahead = (the way to the tape) + the tape + what lies at its end
    enum WpType { WpNone, WpStart, WpTag, WpEnd };
    struct Wp { int idx; WpType type; int tagId; };
    std::vector<Wp> wps;
    Poly route;
    double visibleLen = 0, tapeLeft = 0;
    CourseState::TagMem* endTag = nullptr;
    bool throughTag = false;                     // the route goes on beyond the tag, onto the next tape
    bool peekEnd = false;                        // the tape ends at the edge of the picture: flying on to see what is there
    cv::Point2d tagInDir(0, 0);                  // the way the route runs into the tag
    auto lastTag = [&](int id) {
        if (id == cp_.finishId) return true;
        for (size_t i = 0; i < cp_.order.size(); ++i)
            if (cp_.order[i] == id) return i + 1 >= cp_.order.size();
        return c.k + 1 >= (int)cp_.order.size();
    };
    if (li >= 0) {
        Poly full;
        int startIdx = -1, endIdx = -1;
        if (leadIn) {
            // the way across the gap is a line as well: from the tag just flown over (or the end of the last tape)
            // to the start of the next tape
            for (const auto& t : c.tags)
                if (t.id == c.anchorTag && now_ - t.seen < 0.5) c.anchor = t.p;      // seen again: better than dead reckoning
            if (norm(c.anchor - pl.p.front()) > 0.04) { full.p.push_back(c.anchor); startIdx = 1; }
        }
        const size_t tapeFirst = full.p.size();
        full.p.insert(full.p.end(), pl.p.begin(), pl.p.end());
        const cv::Point2d E = pl.p.back();
        if (cutEnd) {
            full.p.push_back(E + pl.tangent(pl.length()) * 0.6);   // out of view: assume the tape goes straight on
        } else {
            double bd = cp_.tagAssocM;
            for (auto& t : c.tags) {
                if (visited_.count(t.id) || now_ - t.seen > 2.5) continue;
                const double d = norm(t.p - E);
                if (d < bd) { bd = d; endTag = &t; }
            }
            if (endTag) full.p.push_back(endTag->p);
            else if (!inView(obs, E + pl.tangent(pl.length()) * 0.22)) {
                // the floor just beyond the end of the tape is out of the picture and a tag may lie there: go and look
                full.p.push_back(E + pl.tangent(pl.length()) * 0.25);
                peekEnd = true;
            }
            endIdx = (int)full.p.size() - 1;
            if (endTag) {
                tagInDir = norm(endTag->p - E) > 0.03 ? unit(endTag->p - E) : pl.tangent(pl.length());
                // The start of the next tape is already in sight: the route goes on through the tag, which is then a
                // corner like any other and is flown over without stopping.
                int nl = -1, ne = 0;
                double nd = 0;
                const cv::Point2d back = c.prevBehind - endTag->p;       // the tape being followed lies on this side of the tag
                if (!lastTag(endTag->id) &&
                    courseFindNext(obs, li, endTag->p, tagInDir, c.havePrev && norm(back) > 0.10 ? unit(back) : tagInDir * -1.0, nl, ne, nd)) {
                    std::vector<cv::Point2d> nx = obs.paths[nl].pts;
                    if (ne) std::reverse(nx.begin(), nx.end());
                    full.p.insert(full.p.end(), nx.begin(), nx.end());
                    throughTag = true;
                }
            }
        }
        full.build();
        const double sTapeEnd = full.cum[tapeFirst + pl.p.size() - 1];
        // where the drone is along it: never ahead of the previous preview point, never far behind it
        double s0 = 0;
        if (!leadIn) s0 = full.project(pd, nullptr, sPrev - 0.9, sPrev + 0.05 + c.prevExtra);
        else s0 = full.project(pd, nullptr, 0.0, full.cum[tapeFirst] + 0.05);
        route.p.push_back(full.at(s0));
        for (size_t i = 0; i < full.p.size(); ++i) {
            if (full.cum[i] <= s0 + 1e-6) continue;
            if ((int)i == startIdx) wps.push_back({(int)route.p.size(), WpStart, 0});
            if ((int)i == endIdx) wps.push_back({(int)route.p.size(), endTag ? WpTag : WpEnd, endTag ? endTag->id : 0});
            route.p.push_back(full.p[i]);
        }
        if (endIdx >= 0 && wps.empty() && full.cum[endIdx] <= s0 + 1e-6)
            wps.push_back({0, endTag ? WpTag : WpEnd, endTag ? endTag->id : 0});   // standing on it, or just beyond
        route.build();
        tapeLeft = std::max(0.0, sTapeEnd - s0);
        visibleLen = tapeLeft;
        if (endTag) c.goalTag = endTag->id;
        // memory for the next frame
        if (!leadIn) {
            const double sOn = mu::clamp(s0, 0.0, pl.length());
            c.prevBehind = pl.at(std::max(0.0, sOn - 0.15));
            c.travelDir = pl.tangent(sOn + 0.02);
        } else if (norm(c.startPos - pd) > 0.05) {
            c.travelDir = unit(c.startPos - pd);
        }
    } else if (leadIn) {
        route.p = {pd, c.startPos};           // the start is remembered but not seen in this frame
        route.build();
        wps.push_back({1, WpStart, 0});
    } else if (c.goalTag >= 0) {
        // the tape has left the picture (it lies on the side the camera sees least of) but the tag it leads to has
        // not: go on to the tag
        for (auto& t : c.tags)
            if (t.id == c.goalTag && !visited_.count(t.id) && now_ - t.seen < 2.5) endTag = &t;
        if (endTag) {
            route.p = {pd, endTag->p};
            route.build();
            wps.push_back({1, WpTag, endTag->id});
            tagInDir = norm(endTag->p - pd) > 0.03 ? unit(endTag->p - pd) : c.travelDir;
        }
    }

    if (route.p.size() < 2 && wps.empty()) {
        // nothing to follow in this image
        const double lost = now_ - c.lastRouteT;
        courseCommand(lost < cp_.lostCoastS ? c.vCmd * 0.7 : c.bias, st);
        if (lost > 1.0 && !polys.empty() && c.stage == CourseState::Line && !leadIn) {
            LOG_INFO("NAV lost the tape for %.1f s: taking the nearest one again", lost);
            c.stage = CourseState::First;
            c.stageStart = now_;
        }
        if (lost > cp_.lostLandS) {
            LOG_WARN("NAV the path was lost for %.1f s -> landing", lost);
            courseLand("path lost", Phase::Aborted);
        }
        return;
    }
    c.lastRouteT = now_;

    // ---- LINE 3: waypoints reached?
    if (!wps.empty() && wps.front().type == WpTag && endTag && now_ - endTag->seen < 1.0) {
        // flown over the tag: within the capture radius, across the line through it square to the way in, or already
        // on the part of the route that leaves it (a corner taken a little short)
        const cv::Point2d rel = pd - endTag->p;
        const double dTag = norm(rel);
        if (dTag < cp_.tagCaptureM || (dTag < 0.15 && rel.dot(tagInDir) > -0.015) || (wps.front().idx == 0 && dTag < 0.20)) {
            const cv::Point2d keep = c.vCmd;
            courseTagReached(endTag->id, endTag->p, obs, li);
            if (!finished()) courseCommand(keep * 0.8, st);
            return;
        }
    }
    if (!wps.empty() && wps.front().type == WpEnd && norm(route.p[wps.front().idx] - pd) < 0.10) {
        if (c.endSince < 0) c.endSince = now_;
        if (now_ - c.endSince > cp_.endWaitS) {
            LOG_WARN("NAV the tape ends here and no tag was read: taking the nearest other tape");
            c.anchor = route.p[wps.front().idx];
            c.anchorTag = -1;
            c.arrivalDir = c.travelDir;
            c.arrivalFrom = c.travelDir * -1.0;
            c.endSince = -1;
            if (!courseChooseNext(obs, li)) {
                LOG_INFO("NAV no tag here and no further tape within %.1f m: looking around from higher up", cp_.nextSearchM);
                c.stage = CourseState::SeekNext;
                c.stageStart = now_;
                c.seekRises = 0; c.seekOverT = 0;
                c.haveStart = false;
                setPhase(Phase::Search, "the tape ends: looking for the next one");
            }
            return;
        }
    } else {
        c.endSince = -1;
    }

    // ---- LINE 4: speed. Cruise; slow down for corners and for the points where the drone has to stop; never
    // outrun what is visible. Distances are shortened by what the drone still travels before a new command shows.
    const double speedNow = norm(c.vPred);
    const double lead = speedNow * (cp_.shapeTau > 0 ? std::min(cp_.shapeTau, cp_.respTau) : cp_.respTau);
    double v = cp_.vCruise, vStop = 1e9;
    if (!wps.empty()) {
        const Wp& w = wps.front();
        const double sWp = route.cum[std::min((size_t)w.idx, route.cum.size() - 1)];
        double vW = -1;                           // speed allowed at the waypoint; < 0: nothing but the corner rule
        if (w.type == WpEnd) vW = cp_.vMin;
        else if (w.type == WpTag && w.tagId == cp_.finishId) vW = std::min(cp_.vWaypoint, 0.15);
        else if (w.type == WpTag && !throughTag) vW = cp_.vWaypoint;      // what comes after the tag is not known yet
        if (vW >= 0) v = std::min(v, std::sqrt(vW * vW + 2.0 * cp_.accel * std::max(0.0, sWp - lead)));
        if (w.type == WpEnd || (w.type == WpTag && w.tagId == cp_.finishId)) vStop = 2.5 * sWp + 0.03;   // no overshoot
    }
    const double base = std::max(0.03, cp_.cornerBaseM);
    for (double s = base; s + base <= std::min(route.length(), 1.5); s += 0.025) {
        const cv::Point2d d1 = unit(route.at(s) - route.at(s - base)), d2 = unit(route.at(s + base) - route.at(s));
        const double ang = std::acos(mu::clamp(d1.dot(d2), -1.0, 1.0));
        const double vC = std::sqrt(cp_.latAccel / std::max(ang / base, 1e-3));
        v = std::min(v, std::sqrt(vC * vC + 2.0 * cp_.accel * std::max(0.0, s - base - lead)));
    }
    if (cutEnd) v = std::min(v, std::sqrt(2.0 * cp_.accel * std::max(0.02, visibleLen - lead)));
    if (peekEnd) v = std::min(v, cp_.vWaypoint);
    if (leadIn) v = std::min(v, cp_.vGap);
    const double h = st.heightCm();
    c.finalDescent = cp_.finishAltCm > 0 && !wps.empty() && wps.front().type == WpTag && lastTag(wps.front().tagId) &&
                     route.cum[std::min((size_t)wps.front().idx, route.cum.size() - 1)] < cp_.finishDescentM;
    if (h > 0 && !c.finalDescent && std::fabs(p_.altitudeCm + c.altBoostCm - h) > 25) v = std::min(v, cp_.vClimb);
    if (cp_.headingLead && st.valid && cp_.leadSlowDeg > 0) {
        // the nose turns more slowly than the path does at a sharp corner: give it time to come round
        const double off = std::fabs(mu::wrapDeg(c.yawRef - st.yaw));
        if (off > cp_.leadSlowDeg) v *= std::max(cp_.leadSlowMin, 1.0 - (off - cp_.leadSlowDeg) / 60.0);
    }
    v = std::min(std::max(v, cp_.vMin), vStop);

    // ---- LINE 5: direction. Along the route towards a point a little ahead of the drone (far enough to turn in
    // time for a corner, given how fast the drone answers), plus a pull back onto the route, plus what it takes to
    // hold against a steady drift.
    double sLook = std::min(cp_.lookaheadM + cp_.lookaheadPerMs * speedNow, route.length());
    if (!wps.empty() && wps.front().type == WpTag)      // a tag is flown over, not cut short like a corner of the tape
        sLook = std::min(sLook, route.cum[std::min((size_t)wps.front().idx, route.cum.size() - 1)] + cp_.tagLookPastM);
    const cv::Point2d p0 = route.p.front(), target = route.at(sLook);
    const cv::Point2d dir = unit(target - p0);
    const cv::Point2d e = p0 - pd;                 // from the drone to the nearest point of the route
    if (li >= 0 && norm(e) < cp_.biasZoneM) {
        // only what pushes the drone off the tape sideways is a drift: being a little behind along it is not
        c.bias += (e - dir * e.dot(dir)) * (cp_.kBias * dt);
        const double bn = norm(c.bias);
        if (bn > cp_.biasMax) c.bias *= cp_.biasMax / bn;
    }
    cv::Point2d corr = e * cp_.kCross;
    const double cn = norm(corr);
    if (cn > 0.25) corr *= 0.25 / cn;
    if (cp_.headingFollow && st.valid && route.length() > 0.08) {
        // The nose aims along the route, a little further ahead than the preview point (a steadier direction). The
        // heading wanted is kept as a telemetry heading, so the turn itself is closed on the drone's own compass.
        // Lead: not AT a point ahead but the way the route runs there, so the nose turns before the corner and,
        // across a gap, already points along the next tape ...
        cv::Point2d hd(0, 0);
        if (cp_.headingLead) {
            double sH = std::min(sLook + cp_.leadLookM + cp_.leadLookS * speedNow, route.length());
            bool hold = false;
            if (leadIn) {
                // ... the next tape itself, not the way to it (the two directions differ by up to 110 degrees here and
                // the nose would swing towards one and then the other); nothing to go by while that tape is not seen
                if (li < 0) hold = true;
                else if (!wps.empty() && wps.front().type == WpStart)
                    sH = std::max(sH, std::min(route.cum[std::min((size_t)wps.front().idx, route.cum.size() - 1)] + 0.04, route.length()));
            }
            // ... and never beyond the end of a tape that leads to a tag not yet flown over: the nose stays along that
            // tape until the tag is reached (and is held over the last hand-span, where only the step from the tape end
            // to the tag is left to go by). Seen at about 45 degrees to the drone's axes the last straight piece of a
            // tape is lost from the picture's centre-lines, from its free end back to the nearest bend, and it is the
            // direction of that piece in the frame the tag is reached that tells the tape the drone came on from the
            // next one (simulator: with the nose already swinging towards the next tape, 1 flight in 30 went back).
            bool tagAhead = false;
            for (const Wp& w : wps) tagAhead = tagAhead || w.type == WpTag;
            if (tagAhead) { sH = std::min(sH, tapeLeft - 0.04); hold = hold || tapeLeft < 0.08; }
            if (!hold) hd = route.tangent(sH);
        } else {
            hd = unit(route.at(std::min(sLook + cp_.headingLookM, route.length())) - p0);
        }
        if (norm(hd) > 0.5) {
            const double want = mu::wrapDeg(yawAt(c.yawHist, tImg, st.yaw) - mu::rad2deg(std::atan2(hd.y, hd.x)));
            c.yawRef = mu::wrapDeg(c.yawRef + 0.35 * mu::wrapDeg(want - c.yawRef));
        }
    }
    courseCommand(dir * v + corr + c.bias, st);
    if (cp_.traceHz > 0 && now_ - c.lastTraceT >= 1.0 / cp_.traceHz) {     // what the navigator is doing, for the flight log
        c.lastTraceT = now_;
        LOG_DEBUG("CRS t=%.2f %s li=%d wp=%d/%d thr=%d v=%.2f look=%.2f e=(%.3f,%.3f) dir=(%.2f,%.2f) bias=(%.3f,%.3f) cmd=(%.2f,%.2f) vm=(%.2f,%.2f) tgt=(%.2f,%.2f) h=%.0f gain=%.4f len=%.2f",
                  missionTime(), leadIn ? "LEAD" : "LINE", li, wps.empty() ? -1 : (int)wps.front().type, wps.empty() ? -1 : wps.front().idx, throughTag ? 1 : 0, v,
                  sLook, e.x, e.y, dir.x, dir.y, c.bias.x, c.bias.y, c.vCmd.x, c.vCmd.y, c.vModel.x, c.vModel.y, target.x, target.y, st.heightCm(), c.gain,
                  route.length());
    }

    // memory for the next frame: the point of the TAPE under the preview point, and how far beyond its end that was
    if (li >= 0 && !leadIn) {
        c.prevTarget = pl.at(pl.project(target));
        c.havePrev = true;
        c.prevExtra = std::max(0.0, sLook - tapeLeft);
    } else {
        c.prevExtra = 0;
    }
    targetX = target.x; targetY = target.y; haveTarget = true;
    courseRoute = route.p;
    courseWaypoint = wps.empty() ? -1 : wps.front().idx;
}
