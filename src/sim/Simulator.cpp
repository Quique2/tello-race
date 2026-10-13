#include "sim/Simulator.h"

#include <cstdio>
#include <cstring>
#include <sstream>

#include <opencv2/imgproc.hpp>

#include "util/Log.h"
#include "util/MathUtil.h"

void SimParams::load(const Config& c) {
    fps = c.getDouble("sim.fps", fps);
    latencyFrames = c.getInt("sim.latency_frames", latencyFrames);
    camPitchTrueDeg = c.getDouble("sim.cam_pitch_true_deg", camPitchTrueDeg);
    camRollTrueDeg = c.getDouble("sim.cam_roll_true_deg", camRollTrueDeg);
    rcVel = c.getDouble("sim.rc_vel_ms", rcVel);
    rcVz = c.getDouble("sim.rc_vz_ms", rcVz);
    rcYawRate = c.getDouble("sim.rc_yaw_rate", rcYawRate);
    tau = c.getDouble("sim.tau", tau);
    yawTau = c.getDouble("sim.yaw_tau", yawTau);
    leanDegPerMs = c.getDouble("sim.lean_deg_per_ms", leanDegPerMs);
    leanDegPerMs2 = c.getDouble("sim.lean_deg_per_ms2", leanDegPerMs2);
    leanTau = c.getDouble("sim.lean_tau", leanTau);
    takeoffAltM = c.getDouble("sim.takeoff_alt_m", takeoffAltM);
    goSpeedErr = c.getDouble("sim.go_scale_err", goSpeedErr);
    goPosNoiseM = c.getDouble("sim.go_pos_noise_m", goPosNoiseM);
    turnErrDeg = c.getDouble("sim.turn_err_deg", turnErrDeg);
    driftSigma = c.getDouble("sim.drift_sigma", driftSigma);
    noiseSigma = c.getDouble("sim.noise_sigma", noiseSigma);
    blurPerMs = c.getDouble("sim.blur_per_ms", blurPerMs);
    batteryStart = c.getDouble("sim.battery_start", batteryStart);
    boundsMargin = c.getDouble("sim.bounds_margin_m", boundsMargin);
    reachRadiusM = c.getDouble("sim.reach_radius_m", reachRadiusM);
    finishRadiusM = c.getDouble("sim.finish_radius_m", finishRadiusM);
    idleLimitS = c.getDouble("sim.idle_limit_s", idleLimitS);
    maxTimeS = c.getDouble("sim.max_time_s", maxTimeS);
    tracePoseS = c.getDouble("sim.trace_pose_s", tracePoseS);
    exposureS = c.getDouble("sim.exposure_s", exposureS);
    exposureSamples = c.getInt("sim.exposure_samples", exposureSamples);
    startJitterM = c.getDouble("sim.start_jitter_m", startJitterM);
    startYawJitterDeg = c.getDouble("sim.start_yaw_jitter_deg", startYawJitterDeg);
    startYawOffsetDeg = c.getDouble("sim.start_yaw_offset_deg", startYawOffsetDeg);
    rcVelLat = c.getDouble("sim.rc_vel_lat_ms", rcVelLat);
    telemetryHz = c.getDouble("sim.telemetry_hz", telemetryHz);
    tofScale = c.getDouble("sim.tof_scale", tofScale);
    focalScale = c.getDouble("sim.focal_scale", focalScale);
    rcDeadband = c.getDouble("sim.rc_deadband", rcDeadband);
    imageGain = c.getDouble("sim.image_gain", imageGain);
    camAheadM = c.getDouble("sim.camera_ahead_m", camAheadM);
    rcDelayS = c.getDouble("sim.rc_delay_s", rcDelayS);
    inertial = c.getBool("sim.inertial", inertial);
    glareLamps = c.getDoubleList("sim.glare_lamps", glareLamps);
    glareSigmaM = c.getDouble("sim.glare_sigma_m", glareSigmaM);
    glareStrength = c.getDouble("sim.glare_strength", glareStrength);
    chromaBlurPx = c.getDouble("sim.chroma_blur_px", chromaBlurPx);
    blockNoiseSigma = c.getDouble("sim.block_noise_sigma", blockNoiseSigma);
    tintSigma = c.getDouble("sim.tint_sigma", tintSigma);
    tintPatchM = c.getDouble("sim.tint_patch_m", tintPatchM);
    tintWideSigma = c.getDouble("sim.tint_wide_sigma", tintWideSigma);
    tintDriftSigma = c.getDouble("sim.tint_drift_sigma", tintDriftSigma);
    mirrorEdgePx = c.getDouble("sim.mirror_edge_px", mirrorEdgePx);
    mirrorEdgeTiltPx = c.getDouble("sim.mirror_edge_tilt_px", mirrorEdgeTiltPx);
    std::vector<double> ec = c.getDoubleList("sim.mirror_edge_bgr", {});
    if (ec.size() >= 3) mirrorEdgeColor = cv::Scalar(ec[0], ec[1], ec[2]);
}

Simulator::Simulator(const Config& cfg, uint64_t seed, const std::string& trackKind, const TagDetector& tagRenderer)
    : rng_(seed), seed_(seed) {
    cv::theRNG().state = seed * 2654435761ULL + 12345;   // image noise (cv::randn) reproducible per episode, whatever ran before
    p_.load(cfg);
    style_.pxPerM = cfg.getDouble("sim.px_per_m", 100);
    style_.tagSizeM = cfg.getDouble("tag.size_m", 0.16);
    style_.lineWidthCm = cfg.getDouble("sim.line_width_cm", 3);
    style_.stripWidthCm = cfg.getDouble("sim.strip_width_cm", 10);
    std::vector<double> lc = cfg.getDoubleList("sim.line_color_bgr", {25, 25, 25});
    if (lc.size() >= 3) style_.lineColor = cv::Scalar(lc[0], lc[1], lc[2]);
    std::vector<double> cc = cfg.getDoubleList("sim.carpet_bgr", {118, 108, 100});
    if (cc.size() >= 3) style_.carpet = cv::Scalar(cc[0], cc[1], cc[2]);
    std::vector<double> tb = cfg.getDoubleList("sim.tag_black_bgr", {});
    if (tb.size() >= 3) style_.tagBlack = cv::Scalar(tb[0], tb[1], tb[2]);
    style_.floorGain = cfg.getDouble("sim.floor_gain", 1.0);
    style_.textureSeed = seed;
    if (trackKind == "example") track_ = TrackBuilder::example();
    else if (trackKind == "random" || trackKind.empty()) track_ = TrackBuilder::random(seed, cfg.getDouble("sim.world_size_m", 9.0));
    else {
        std::string err;
        if (!TrackBuilder::fromFile(trackKind, track_, &err)) {
            LOG_ERROR("%s -> using the example track", err.c_str());
            track_ = TrackBuilder::example();
        } else if (track_.pxPerM > 0) style_.pxPerM = track_.pxPerM;
    }
    // The sheets of the tags are loose: on the day of a flight they do not lie exactly where the photo has them.
    const std::vector<double> off = cfg.getDoubleList("sim.tag_offsets", {});
    for (size_t i = 0; i + 2 < off.size(); i += 3) TrackBuilder::moveTag(track_, (int)std::lround(off[i]), off[i + 1], off[i + 2]);
    const double paper = cfg.getDouble("sim.tag_paper_m", 0);
    if (paper > 0) track_.tagPaperW = track_.tagPaperH = paper;
    world_ = TrackBuilder::render(track_, style_, tagRenderer);
    if (p_.tintSigma > 0 || p_.tintWideSigma > 0) tintWorld();
    if (p_.glareLamps.size() >= 3) {
        // a reflection washes the tape towards the white of the floor it lies on: the brightest twentieth of the floor
        cv::Mat grey;
        cv::cvtColor(world_, grey, cv::COLOR_BGR2GRAY);
        int hist[256] = {0}, total = 0, acc = 0;
        for (int y = 0; y < grey.rows; y += 2)
            for (int x = 0; x < grey.cols; x += 2) { ++hist[grey.at<uchar>(y, x)]; ++total; }
        for (int v = 255; v >= 0; --v) { acc += hist[v]; if (acc >= total / 20) { floorWhite_ = v; break; } }
    }
    route_ = track_.route();
    res_.tagPass.assign(track_.tags.size(), 1e9);
    if (p_.tracePoseS > 0)
        for (size_t i = 0; i < track_.segments.size(); ++i) {
            const auto& sp = track_.segments[i].pts;
            if (sp.size() < 2) continue;
            double a0 = mu::rad2deg(std::atan2(sp[1].y - sp[0].y, sp[1].x - sp[0].x));
            double a1 = mu::rad2deg(std::atan2(sp.back().y - sp[sp.size() - 2].y, sp.back().x - sp[sp.size() - 2].x));
            LOG_INFO("SIM segment %zu: start (%.2f, %.2f) dir %.0f -> end (%.2f, %.2f) dir %.0f, tag ID%d at (%.2f, %.2f)", i + 1,
                     sp.front().x, sp.front().y, a0, sp.back().x, sp.back().y, a1,
                     i < track_.tags.size() ? track_.tags[i].id : 0, i < track_.tags.size() ? track_.tags[i].x : 0.0,
                     i < track_.tags.size() ? track_.tags[i].y : 0.0);
        }
    Intrinsics K;
    K.width = cfg.getInt("camera.width", 960);
    K.height = cfg.getInt("camera.height", 720);
    K.fx = cfg.getDouble("camera.fx", 920) * p_.focalScale;
    K.fy = cfg.getDouble("camera.fy", 920) * p_.focalScale;
    K.cx = cfg.getDouble("camera.cx", K.width / 2.0);
    K.cy = cfg.getDouble("camera.cy", K.height / 2.0);
    truth_.setIntrinsics(K);
    truth_.setCameraMount(p_.camPitchTrueDeg, p_.camRollTrueDeg);
    x_ = track_.start.x; y_ = track_.start.y; z_ = 0;
    psi_ = psi0_ = track_.startYawDeg + p_.startYawOffsetDeg;
    if (p_.startJitterM > 0 || p_.startYawJitterDeg > 0) {   // nobody places a drone to the millimetre
        std::normal_distribution<double> np(0.0, 1.0);
        x_ += p_.startJitterM * np(rng_); y_ += p_.startJitterM * np(rng_);
        psi_ = psi0_ = track_.startYawDeg + p_.startYawOffsetDeg + p_.startYawJitterDeg * np(rng_);
    }
    LOG_INFO("Simulator: %s, %zu segments, tags:%s", track_.description.c_str(), track_.segments.size(),
             [&] { std::string s; for (auto& t : track_.tags) s += " " + std::to_string(t.id); return s; }().c_str());
}

void Simulator::physicsStep(double dt) {
    if (!flying_) return;
    while (!rcQueue_.empty() && rcQueue_.front().t <= t_ + 1e-9) {
        const PendingRc& q = rcQueue_.front();
        cmdVx_ = q.vx; cmdVy_ = q.vy; cmdVz_ = q.vz; cmdYaw_ = q.yaw;
        rcQueue_.pop_front();
    }
    double a = dt / p_.tau;
    const double vx0 = vxB_, vy0 = vyB_;
    vxB_ += (cmdVx_ - vxB_) * a;
    vyB_ += (cmdVy_ - vyB_) * a;
    yawRate_ += (cmdYaw_ - yawRate_) * (dt / p_.yawTau);
    if (std::fabs(cmdVz_) > 1e-6) { vz_ += (cmdVz_ - vz_) * a; zTarget_ = z_; }
    else vz_ = (zTarget_ - z_) * 2.0;
    std::normal_distribution<double> nd(0.0, p_.driftSigma * std::sqrt(dt));
    driftX_ = mu::clamp(driftX_ + nd(rng_), -0.08, 0.08);
    driftY_ = mu::clamp(driftY_ + nd(rng_), -0.08, 0.08);
    double psi = mu::deg2rad(psi_);
    double vxW = vxB_ * std::cos(psi) - vyB_ * std::sin(psi) + driftX_;
    double vyW = vxB_ * std::sin(psi) + vyB_ * std::cos(psi) + driftY_;
    x_ += vxW * dt;
    y_ += vyW * dt;
    res_.pathLen += std::hypot(vxW, vyW) * dt;
    z_ = std::max(0.0, z_ + vz_ * dt);
    psi_ = mu::wrapDeg(psi_ + yawRate_ * dt);
    if (p_.inertial) {       // the velocity is kept over the floor, so in the body's axes it turns the other way
        const double d = mu::deg2rad(-yawRate_ * dt), vx = vxB_, vy = vyB_;
        vxB_ = vx * std::cos(d) - vy * std::sin(d);
        vyB_ = vx * std::sin(d) + vy * std::cos(d);
    }
    // tilt: what holds the speed against the drag, plus what it takes to accelerate (the airframe leans first, the speed follows)
    if (dt > 1e-9) {
        const double k = std::min(1.0, dt / std::max(1e-3, p_.leanTau));
        leanAx_ += (p_.leanDegPerMs2 * (vxB_ - vx0) / dt - leanAx_) * k;
        leanAy_ += (p_.leanDegPerMs2 * (vyB_ - vy0) / dt - leanAy_) * k;
    }
    bodyPitch_ = -(p_.leanDegPerMs * vxB_ + leanAx_);
    bodyRoll_ = -(p_.leanDegPerMs * vyB_ + leanAy_);
    if (trail_.empty() || cv::norm(trail_.back() - cv::Point2d(x_, y_)) > 0.05) trail_.emplace_back(x_, y_);
    checkProgress();
}

void Simulator::advance(double dt) {
    double h = 1.0 / p_.fps;
    while (dt > 1e-9) {
        double s = std::min(dt, h);
        physicsStep(s);
        t_ += s;
        dt -= s;
    }
}

// Straight relative move in the body frame with a smooth velocity profile (models "go"/"forward"/...)
void Simulator::moveBlocking(double dxB, double dyB, double dzB, double speedMs) {
    double dist = std::sqrt(dxB * dxB + dyB * dyB + dzB * dzB);
    if (dist < 1e-6) return;
    std::normal_distribution<double> nScale(0.0, p_.goSpeedErr), nPos(0.0, p_.goPosNoiseM);
    double scale = 1.0 + nScale(rng_);
    double psi = mu::deg2rad(psi_);
    double tx = x_ + (dxB * std::cos(psi) - dyB * std::sin(psi)) * scale + nPos(rng_);
    double ty = y_ + (dxB * std::sin(psi) + dyB * std::cos(psi)) * scale + nPos(rng_);
    double tz = std::max(0.0, z_ + dzB * scale);
    double x0 = x_, y0 = y_, z0 = z_;
    double duration = dist / std::max(0.1, speedMs) + 0.8;
    double h = 1.0 / p_.fps, el = 0;
    cmdVx_ = cmdVy_ = cmdVz_ = cmdYaw_ = 0; rcQueue_.clear();
    while (el < duration) {
        double s = std::min(h, duration - el);
        el += s;
        double f = 0.5 - 0.5 * std::cos(mu::PI * el / duration);     // ease in/out
        double fdot = 0.5 * mu::PI / duration * std::sin(mu::PI * el / duration);
        x_ = x0 + (tx - x0) * f; y_ = y0 + (ty - y0) * f; z_ = z0 + (tz - z0) * f;
        vxB_ = dxB * scale * fdot; vyB_ = dyB * scale * fdot; vz_ = dzB * fdot;
        bodyPitch_ = -p_.leanDegPerMs * vxB_;
        bodyRoll_ = -p_.leanDegPerMs * vyB_;
        t_ += s;
        checkProgress();
        if (blockingCb_) blockingCb_(s);
        if (!flying_) return;
    }
    vxB_ = vyB_ = vz_ = 0;
    zTarget_ = z_;
    advance(0.3);
}

void Simulator::turnBlocking(double deg) {
    std::normal_distribution<double> nErr(0.0, p_.turnErrDeg);
    double actual = deg + nErr(rng_);
    double duration = std::fabs(deg) / 90.0 + 0.4;
    double h = 1.0 / p_.fps, el = 0, psi0 = psi_;
    cmdVx_ = cmdVy_ = cmdVz_ = cmdYaw_ = 0; rcQueue_.clear();
    vxB_ = vyB_ = 0;
    while (el < duration) {
        double s = std::min(h, duration - el);
        el += s;
        double f = 0.5 - 0.5 * std::cos(mu::PI * el / duration);
        psi_ = mu::wrapDeg(psi0 + actual * f);
        t_ += s;
        physicsStep(0);
        if (blockingCb_) blockingCb_(s);
    }
    yawRate_ = 0;
    advance(0.3);
}

bool Simulator::command(const std::string& cmd, std::string* reply, double) {
    std::istringstream ss(cmd);
    std::string op;
    ss >> op;
    auto ok = [&](const std::string& r = "ok") { if (reply) *reply = r; lastReply = r; return r != "error"; };
    LOG_DEBUG("sim << %s", cmd.c_str());
    if (op == "command" || op == "streamon" || op == "streamoff" || op == "speed") return ok();
    if (op == "takeoff") {
        if (flying_) return ok("error");
        flying_ = true;
        res_.tookOff = true;
        takeoffT_ = t_;
        lastProgressT_ = t_;
        takingOff_ = true;                 // the climb is the drone's own: where it was put down is not a tracking error
        moveBlocking(0, 0, p_.takeoffAltM, 0.5);
        takingOff_ = false;
        zTarget_ = z_;
        return ok();
    }
    if (op == "land") {
        if (!flying_) return ok();
        landing_ = true;
        moveBlocking(0, 0, -z_, 0.4);
        flying_ = false;
        res_.landed = true;
        res_.time = t_ - takeoffT_;
        const TrackTag& fin = track_.tags.back();
        double d = std::hypot(x_ - fin.x, y_ - fin.y);
        res_.finished = (nextTag_ >= (int)track_.tags.size()) && d < p_.finishRadiusM;
        res_.reason = res_.finished ? "finished" : (nextTag_ >= (int)track_.tags.size() ? "landed too far from FINISH" : "landed early");
        return ok();
    }
    if (op == "emergency") {
        flying_ = false; z_ = 0; res_.landed = true; res_.time = t_ - takeoffT_; res_.reason = "emergency";
        return ok();
    }
    if (op == "go") {
        int x = 0, y = 0, z = 0, s = 50;
        ss >> x >> y >> z >> s;
        if (std::abs(x) < 20 && std::abs(y) < 20 && std::abs(z) < 20) return ok("error");
        if (!flying_) return ok("error");
        moveBlocking(x / 100.0, y / 100.0, z / 100.0, mu::clamp(s, 10, 100) / 100.0);
        return ok();
    }
    if (op == "forward" || op == "back" || op == "left" || op == "right" || op == "up" || op == "down") {
        int d = 0;
        ss >> d;
        if (d < 20 || d > 500 || !flying_) return ok("error");
        double m = d / 100.0;
        if (op == "forward") moveBlocking(m, 0, 0, 0.5);
        else if (op == "back") moveBlocking(-m, 0, 0, 0.5);
        else if (op == "left") moveBlocking(0, m, 0, 0.5);
        else if (op == "right") moveBlocking(0, -m, 0, 0.5);
        else if (op == "up") moveBlocking(0, 0, m, 0.4);
        else moveBlocking(0, 0, -std::min(m, std::max(0.0, z_ - 0.2)), 0.4);
        return ok();
    }
    if (op == "cw" || op == "ccw") {
        int d = 0;
        ss >> d;
        if (d < 1 || d > 360 || !flying_) return ok("error");
        turnBlocking(op == "cw" ? -d : d);
        return ok();
    }
    if (op == "battery?") return ok(std::to_string((int)std::lround(std::max(0.0, p_.batteryStart - t_ / 7.8))));
    if (op == "speed?") return ok("50.0");
    if (op == "sdk?") return ok("30");
    if (op == "time?") return ok(std::to_string((int)(flying_ ? t_ - takeoffT_ : 0)) + "s");
    if (op == "height?") return ok(std::to_string((int)(z_ * 10)) + "dm");
    if (op == "rc") return ok();
    return ok("error");
}

void Simulator::rc(int lr, int fb, int ud, int yaw) {
    if (std::abs(fb) < p_.rcDeadband) fb = 0;
    if (std::abs(lr) < p_.rcDeadband) lr = 0;
    const double vx = mu::clamp(fb, -100, 100) * p_.rcVel;
    const double vy = -mu::clamp(lr, -100, 100) * (p_.rcVelLat > 0 ? p_.rcVelLat : p_.rcVel);
    const double vz = mu::clamp(ud, -100, 100) * p_.rcVz;
    const double vyaw = -mu::clamp(yaw, -100, 100) * p_.rcYawRate;
    if (p_.rcDelayS > 0) { rcQueue_.push_back({t_ + p_.rcDelayS, vx, vy, vz, vyaw}); return; }
    cmdVx_ = vx; cmdVy_ = vy; cmdVz_ = vz; cmdYaw_ = vyaw;
}

// The real Tello pushes its state about 10 times a second: between two packets the navigator sees the old one.
TelloState Simulator::state() const {
    if (p_.telemetryHz <= 0) return trueState();
    if (t_ - heldStateT_ >= 1.0 / p_.telemetryHz) { heldState_ = trueState(); heldStateT_ = t_; }
    return heldState_;
}

TelloState Simulator::trueState() const {
    TelloState s;
    s.valid = true;
    s.stamp = t_;
    s.pitch = mu::roundInt(bodyPitch_);
    s.roll = mu::roundInt(bodyRoll_);
    s.yaw = mu::roundInt(mu::wrapDeg(-(psi_ - psi0_)));
    s.vgx = mu::roundInt(vxB_ * 100);
    s.vgy = mu::roundInt(-vyB_ * 100);
    s.vgz = mu::roundInt(vz_ * 100);
    s.tof = flying_ ? mu::roundInt(z_ * 100 * p_.tofScale) : 10;
    s.h = mu::roundInt(z_ * 100);
    s.bat = (int)std::lround(std::max(0.0, p_.batteryStart - t_ / 7.8));
    s.baro = z_;
    s.time = flying_ ? (int)(t_ - takeoffT_) : 0;
    s.templ = 60; s.temph = 63;
    return s;
}

void Simulator::checkProgress() {
    if (!flying_) return;
    if (t_ - takeoffT_ > 1.0 && !landing_ && !takingOff_ && !devPaused_) {   // path fidelity: distance to the route (the tape and the straight links between segments)
        double best = 1e9;
        for (size_t i = 1; i < route_.size(); ++i) {
            const cv::Point2d& a = route_[i - 1]; const cv::Point2d& b = route_[i];
            double dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
            double u = l2 > 1e-9 ? mu::clamp(((x_ - a.x) * dx + (y_ - a.y) * dy) / l2, 0.0, 1.0) : 0.0;
            best = std::min(best, std::hypot(x_ - (a.x + u * dx), y_ - (a.y + u * dy)));
        }
        res_.devSum += best; res_.devSq += best * best; res_.devMax = std::max(res_.devMax, best); ++res_.devN;
    }
    for (size_t i = 0; i < track_.tags.size(); ++i)
        res_.tagPass[i] = std::min(res_.tagPass[i], std::hypot(x_ - track_.tags[i].x, y_ - track_.tags[i].y));
    if (p_.tracePoseS > 0 && t_ - lastTraceT_ >= p_.tracePoseS) {
        lastTraceT_ = t_;
        const TrackTag* tg = nextTag_ < (int)track_.tags.size() ? &track_.tags[nextTag_] : nullptr;
        LOG_INFO("SIM pose t=%.1f x=%.2f y=%.2f z=%.2f psi=%.0f  next tag ID%d at (%.2f, %.2f) dist=%.2f",
                 t_ - takeoffT_, x_, y_, z_, psi_, tg ? tg->id : 0, tg ? tg->x : 0.0, tg ? tg->y : 0.0,
                 tg ? std::hypot(x_ - tg->x, y_ - tg->y) : 0.0);
    }
    double half = track_.worldSize / 2 - p_.boundsMargin;
    if (std::fabs(x_) > half || std::fabs(y_) > half) {
        res_.crashed = true;
        res_.reason = "hit the net (out of bounds)";
        res_.time = t_ - takeoffT_;
        flying_ = false;
        LOG_WARN("SIM crash: out of bounds at (%.2f, %.2f)", x_, y_);
        return;
    }
    if (nextTag_ < (int)track_.tags.size()) {
        const TrackTag& tg = track_.tags[nextTag_];
        if (std::hypot(x_ - tg.x, y_ - tg.y) < p_.reachRadiusM) {
            ++nextTag_;
            res_.segments = nextTag_;
            lastProgressT_ = t_;
            LOG_INFO("SIM checkpoint: reached tag ID%d (%d/%zu) at t=%.1fs", tg.id, nextTag_, track_.tags.size(), t_ - takeoffT_);
        }
    }
    if (t_ - lastProgressT_ > p_.idleLimitS && res_.reason.empty()) { res_.reason = "no progress for 60 s (idle rule)"; res_.time = t_ - takeoffT_; }
    if (t_ - takeoffT_ > p_.maxTimeS && res_.reason.empty()) { res_.reason = "max attempt time"; res_.time = t_ - takeoffT_; }
}

bool Simulator::done() const {
    return res_.crashed || (res_.tookOff && res_.landed) || (!res_.reason.empty() && !res_.finished && !flying_) ||
           (res_.tookOff && !res_.reason.empty() && (res_.reason.rfind("no progress", 0) == 0 || res_.reason.rfind("max attempt", 0) == 0));
}

// Render the camera view by mapping every pixel's ground intersection onto the rendered world image.
cv::Mat Simulator::render() {
    truth_.setHeight(std::max(0.02, z_));
    truth_.setBodyAttitude(bodyPitch_, bodyRoll_);
    const Intrinsics& K = truth_.K();
    const int W = K.width, H = K.height;
    if (mapX_.empty()) { mapX_.create(H, W, CV_32F); mapY_.create(H, W, CV_32F); }
    // How many views the exposure needs: one for every two or three pixels the picture's corner moves in that time.
    int views = 1;
    if (p_.exposureS > 0 && p_.exposureSamples > 1) {
        const double far = z_ * std::hypot(0.5 * W / K.fx, 0.5 * H / K.fy);            // floor distance of a corner of the picture
        const double moved = (std::hypot(vxB_, vyB_) + std::fabs(mu::deg2rad(yawRate_)) * far) * p_.exposureS * K.fx / std::max(0.05, z_);
        views = mu::clamp((int)std::ceil(moved / 2.5) + 1, 1, p_.exposureSamples);
        if (views > 1 && bodyX_.empty()) { bodyX_.create(H, W, CV_32F); bodyY_.create(H, W, CV_32F); }
    }
    // A glossy floor mirrors every lamp at the point G where the ray from the lamp to the camera's mirror image crosses
    // it: G = C + (L - C) * h / (h + hl), with C the point under the camera, h its height and hl the lamp's. The
    // reflection moves along with the drone, more slowly than the drone does.
    std::vector<cv::Point2d> glareAt;
    for (size_t i = 0; i + 2 < p_.glareLamps.size(); i += 3) {
        const cv::Point2d C(x_ + p_.camAheadM * std::cos(mu::deg2rad(psi_)), y_ + p_.camAheadM * std::sin(mu::deg2rad(psi_)));
        const double hl = std::max(0.1, p_.glareLamps[i + 2]);
        glareAt.push_back(C + (cv::Point2d(p_.glareLamps[i], p_.glareLamps[i + 1]) - C) * (z_ / (z_ + hl)));
    }
    const bool glare = !glareAt.empty() && p_.glareStrength > 0 && p_.glareSigmaM > 0;
    if (glare) { glare_.create(H, W, CV_32F); glare_.setTo(0); }
    const double glareK = -0.5 / std::max(1e-6, p_.glareSigmaM * p_.glareSigmaM), glareR2 = 9.0 * p_.glareSigmaM * p_.glareSigmaM;
    double psi = mu::deg2rad(psi_), c = std::cos(psi), s = std::sin(psi);
    double ppm = style_.pxPerM, half = track_.worldSize * ppm / 2;
    // body_from_cam rotation applied to the per-pixel camera rays
    cv::Vec3d ex = truth_.camToBody(cv::Vec3d(1, 0, 0)), ey = truth_.camToBody(cv::Vec3d(0, 1, 0)), ez = truth_.camToBody(cv::Vec3d(0, 0, 1));
    for (int v = 0; v < H; ++v) {
        float* mx = mapX_.ptr<float>(v);
        float* my = mapY_.ptr<float>(v);
        double b = (v - K.cy) / K.fy;
        for (int u = 0; u < W; ++u) {
            double a = (u - K.cx) / K.fx;
            double dbx = ex[0] * a + ey[0] * b + ez[0];
            double dby = ex[1] * a + ey[1] * b + ez[1];
            double dbz = ex[2] * a + ey[2] * b + ez[2];
            if (views > 1) { bodyX_.at<float>(v, u) = 1e6f; bodyY_.at<float>(v, u) = 1e6f; }
            if (dbz > -1e-4) { mx[u] = -1; my[u] = -1; continue; }
            double t = -z_ / dbz;
            if (t > 60.0) { mx[u] = -1; my[u] = -1; continue; }
            if (views > 1) { bodyX_.at<float>(v, u) = (float)(t * dbx); bodyY_.at<float>(v, u) = (float)(t * dby); }
            double gx = x_ + p_.camAheadM * c + t * (c * dbx - s * dby);
            double gy = y_ + p_.camAheadM * s + t * (s * dbx + c * dby);
            mx[u] = (float)(half + gx * ppm);
            my[u] = (float)(half - gy * ppm);
            if (glare) {
                double w = 0;
                for (const auto& g : glareAt) {
                    const double d2 = (gx - g.x) * (gx - g.x) + (gy - g.y) * (gy - g.y);
                    if (d2 < glareR2) w += std::exp(glareK * d2);
                }
                if (w > 0) glare_.at<float>(v, u) = (float)w;
            }
        }
    }
    cv::Mat img;
    if (views > 1) img = renderExposure(views);
    else cv::remap(world_, img, mapX_, mapY_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(75, 70, 62));
    if (glare) applyGlare(img);
    if (p_.exposureS > 0 && views == 1) {
        // The floor point at the image centre moves (relative to the drone) by -v * exposure, plus the yaw rate
        // turning it around the drone: smear the frame along that image direction by that many pixels.
        cv::Vec3d ray = truth_.pixelRayBody(K.cx, K.cy);
        if (ray[2] < -1e-3) {
            double tt = -z_ / ray[2];
            double X0 = tt * ray[0], Y0 = tt * ray[1], u0, v0, u1, v1;
            double w = mu::deg2rad(yawRate_);
            double X1 = X0 - (vxB_ - w * Y0) * p_.exposureS, Y1 = Y0 - (vyB_ + w * X0) * p_.exposureS;
            if (truth_.groundToPixel(X0, Y0, u0, v0) && truth_.groundToPixel(X1, Y1, u1, v1)) {
                double du = u1 - u0, dv = v1 - v0, len = std::hypot(du, dv);
                int n = std::min((int)std::lround(len), 61);
                if (n >= 2) {
                    int ks = n | 1;
                    cv::Mat kern = cv::Mat::zeros(ks, ks, CV_32F);
                    double hx = du / len * (n - 1) / 2.0, hy = dv / len * (n - 1) / 2.0, c0 = (ks - 1) / 2.0;
                    cv::line(kern, cv::Point((int)std::lround(c0 - hx), (int)std::lround(c0 - hy)),
                             cv::Point((int)std::lround(c0 + hx), (int)std::lround(c0 + hy)), cv::Scalar(1.0), 1, cv::LINE_8);
                    double sum = cv::sum(kern)[0];
                    if (sum > 0) { kern /= sum; cv::filter2D(img, img, -1, kern, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE); }
                }
            }
        }
    } else if (p_.exposureS <= 0) {
        double speed = std::hypot(vxB_, vyB_) + std::fabs(yawRate_) * 0.012;
        int k = 1 + (int)(speed * p_.blurPerMs);
        if (k > 1) cv::blur(img, img, cv::Size(std::min(k, 9), std::min(k, 9)));
    }
    if (p_.chromaBlurPx > 0) applyChromaBlur(img);
    if (p_.mirrorEdgePx > 0) applyMirrorEdge(img);
    if (std::fabs(p_.imageGain - 1.0) > 1e-3) img.convertTo(img, -1, p_.imageGain, 0);
    if (p_.noiseSigma > 0 || p_.blockNoiseSigma > 0 || p_.tintDriftSigma > 0) {
        cv::Mat f;
        img.convertTo(f, CV_32FC3);
        if (p_.noiseSigma > 0) {
            cv::Mat n(img.size(), CV_32FC3);
            cv::randn(n, 0, p_.noiseSigma);
            f += n;
        }
        if (p_.blockNoiseSigma > 0) addBlockNoise(f);
        if (p_.tintDriftSigma > 0) {
            // the white balance hunts: the whole picture a little bluer or greener, changing over about a second
            const double keep = std::exp(-1.0 / (0.7 * p_.fps));
            tintDrift_ = keep * tintDrift_ + std::sqrt(1.0 - keep * keep) * cv::theRNG().gaussian(p_.tintDriftSigma);
            f += cv::Scalar(tintDrift_, 0, 0);
        }
        f.convertTo(img, CV_8UC3);
    }
    return img;
}

// The picture gathers light for the whole exposure while the drone flies on and its nose turns: the mean of the views
// from the beginning of the exposure to its end. A point far from the drone is carried further by a turn than one
// under it, which one smear for the whole picture cannot show.
cv::Mat Simulator::renderExposure(int n) {
    const double ppm = style_.pxPerM, half = track_.worldSize * ppm / 2;
    const double psiNow = mu::deg2rad(psi_);
    const double vxW = vxB_ * std::cos(psiNow) - vyB_ * std::sin(psiNow), vyW = vxB_ * std::sin(psiNow) + vyB_ * std::cos(psiNow);
    cv::Mat sum, view, mx, my;
    for (int k = 0; k < n; ++k) {
        const double back = p_.exposureS * k / (n - 1);                                 // this long before the end of the exposure
        const double psi = mu::deg2rad(psi_ - yawRate_ * back), c = std::cos(psi), s = std::sin(psi);
        const double cx = x_ - vxW * back + p_.camAheadM * c, cy = y_ - vyW * back + p_.camAheadM * s;
        cv::addWeighted(bodyX_, c * ppm, bodyY_, -s * ppm, half + cx * ppm, mx);
        cv::addWeighted(bodyX_, -s * ppm, bodyY_, -c * ppm, half - cy * ppm, my);
        cv::remap(world_, view, mx, my, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(75, 70, 62));
        if (sum.empty()) sum = cv::Mat::zeros(view.size(), CV_32FC3);
        cv::accumulate(view, sum);
    }
    cv::Mat img;
    sum.convertTo(img, CV_8UC3, 1.0 / n);
    return img;
}

// The tape is glossy, the tarp and the paper of the tags are not: only what has a colour is washed out, towards the
// white of the floor. (On the recorded frames the tape under the lamp keeps its hue, loses two thirds of its
// saturation and comes up to four fifths of the tarp's brightness; the tarp next to it does not change.)
void Simulator::applyGlare(cv::Mat& img) const {
    for (int v = 0; v < img.rows; ++v) {
        const float* g = glare_.ptr<float>(v);
        cv::Vec3b* px = img.ptr<cv::Vec3b>(v);
        for (int u = 0; u < img.cols; ++u) {
            if (g[u] < 0.02f) continue;
            cv::Vec3b& q = px[u];
            const int hi = std::max(q[0], std::max(q[1], q[2])), lo = std::min(q[0], std::min(q[1], q[2]));
            const double a = std::min(1.0, p_.glareStrength * g[u]) * mu::clamp((hi - lo - 20) / 40.0, 0.0, 1.0);
            if (a <= 0) continue;
            for (int k = 0; k < 3; ++k) q[k] = cv::saturate_cast<uchar>(q[k] + a * (floorWhite_ - q[k]));
        }
    }
}

// The video carries colour at half the resolution of brightness, and smooths it more: next to a tape the white floor
// keeps a breath of the tape's colour for some pixels beyond where it stops being dark. Seen from above that haze
// closes the two centimetres of white paper between the end of a tape and the black of the tag it ends at.
void Simulator::applyChromaBlur(cv::Mat& img) const {
    cv::Mat f, ycc;
    img.convertTo(f, CV_32FC3);            // in whole levels the way there and back would tint every dark grey by a level
    cv::cvtColor(f, ycc, cv::COLOR_BGR2YCrCb);
    std::vector<cv::Mat> ch;
    cv::split(ycc, ch);
    cv::GaussianBlur(ch[1], ch[1], cv::Size(0, 0), p_.chromaBlurPx);
    cv::GaussianBlur(ch[2], ch[2], cv::Size(0, 0), p_.chromaBlurPx);
    cv::merge(ch, ycc);
    cv::cvtColor(ycc, f, cv::COLOR_YCrCb2BGR);
    f.convertTo(img, CV_8UC3);
}

// The camera looks into the mirror past the mirror's lower edge. Out of focus, that edge is a dark veil over the
// last rows of the picture: nothing at the upper end of the band, nearly opaque in the last row. A tape that runs out
// of the bottom of the picture fades away in it before it reaches the border.
void Simulator::applyMirrorEdge(cv::Mat& img) const {
    const int W = img.cols, H = img.rows;
    for (int u = 0; u < W; ++u) {
        const double band = p_.mirrorEdgePx + p_.mirrorEdgeTiltPx * ((double)u / (W - 1) - 0.5);
        if (band < 1) continue;
        for (int v = std::max(0, (int)std::ceil(H - band)); v < H; ++v) {
            const double a = 0.95 * std::pow((v - (H - band)) / band, 1.3);
            cv::Vec3b& q = img.at<cv::Vec3b>(v, u);
            for (int k = 0; k < 3; ++k) q[k] = cv::saturate_cast<uchar>(q[k] + a * (p_.mirrorEdgeColor[k] - q[k]));
        }
    }
}

// Video compression works in 8x8 pixel blocks and keeps the colour of each only roughly: on a plain surface every
// block comes out a little off as a whole. That is far less noise than the sensor's, but it does not average out
// over a few pixels: a patch of dark grey can turn bluish, all of it. Every picture is coded as a correction of the
// one before, so an error outlives its picture: most of it is still there in the next.
void Simulator::addBlockNoise(cv::Mat& f) {
    cv::Mat coarse((f.rows + 7) / 8, (f.cols + 7) / 8, CV_32FC3), fine;
    cv::randn(coarse, 0, p_.blockNoiseSigma);
    const double keep = 0.7;
    if (blockNoise_.size() == coarse.size()) cv::addWeighted(blockNoise_, keep, coarse, std::sqrt(1.0 - keep * keep), 0, coarse);
    blockNoise_ = coarse;
    cv::resize(coarse, fine, cv::Size(coarse.cols * 8, coarse.rows * 8), 0, 0, cv::INTER_NEAREST);
    f += fine(cv::Rect(0, 0, f.cols, f.rows));
}

// The colour of the floor as the camera gives it is uneven in a way that stays put on the floor: the light differs
// from one end of the course to the other, and what the video coding gets wrong travels along with the things it
// sits on. A level or two of blue against green, nothing to the eye: but it is what decides whether the dark grey of
// a tag is taken for cyan or for blue tape, and it makes the same patches of a tag pass for tape frame after frame.
void Simulator::tintWorld() {
    cv::RNG rng(seed_ * 0x9E3779B97F4A7C15ULL + 77);
    cv::Mat tint = cv::Mat::zeros(world_.size(), CV_32F);
    auto add = [&](double cellM, double sigma) {
        if (sigma <= 0) return;
        const int n = std::max(2, (int)std::ceil(track_.worldSize / std::max(1e-3, cellM)) + 1);
        cv::Mat grid(n, n, CV_32F), big;
        rng.fill(grid, cv::RNG::NORMAL, 0.0, sigma);
        cv::resize(grid, big, world_.size(), 0, 0, cv::INTER_CUBIC);
        tint += big;
    };
    add(p_.tintPatchM, p_.tintSigma);
    add(0.6, p_.tintWideSigma);
    std::vector<cv::Mat> ch;
    cv::split(world_, ch);
    cv::add(ch[0], tint, ch[0], cv::noArray(), CV_8U);
    cv::merge(ch, world_);
}

bool Simulator::latest(Frame& out, uint64_t, double) {
    advance(1.0 / p_.fps);
    latencyQ_.push_back(render());
    while ((int)latencyQ_.size() > p_.latencyFrames + 1) latencyQ_.pop_front();
    out.image = latencyQ_.front();
    out.seq = ++seq_;
    out.stamp = t_;
    return true;
}

cv::Mat Simulator::topView(int sizePx) const {
    cv::Mat top;
    cv::resize(world_, top, cv::Size(sizePx, sizePx), 0, 0, cv::INTER_AREA);
    double k = (double)sizePx / world_.cols;
    auto toPx = [&](double wx, double wy) {
        cv::Point2d p = TrackBuilder::worldToPx(track_, style_.pxPerM, cv::Point2d(wx, wy));
        return cv::Point((int)std::lround(p.x * k), (int)std::lround(p.y * k));
    };
    for (size_t i = 1; i < trail_.size(); ++i)
        cv::line(top, toPx(trail_[i - 1].x, trail_[i - 1].y), toPx(trail_[i].x, trail_[i].y), cv::Scalar(0, 200, 255), 1, cv::LINE_AA);
    for (size_t i = 0; i < track_.tags.size(); ++i) {
        const TrackTag& tg = track_.tags[i];
        cv::Scalar col = (int)i < nextTag_ ? cv::Scalar(0, 200, 0) : ((int)i == nextTag_ ? cv::Scalar(0, 255, 255) : cv::Scalar(200, 200, 200));
        cv::circle(top, toPx(tg.x, tg.y), 7, col, 2);
        cv::putText(top, std::to_string(tg.id), toPx(tg.x, tg.y) + cv::Point(9, 4), cv::FONT_HERSHEY_SIMPLEX, 0.5, col, 1);
    }
    // camera footprint on the ground (rays through the image bottom corners and a far row)
    GroundProjector g = truth_;
    g.setHeight(std::max(0.02, z_));
    const Intrinsics& K = g.K();
    double hz = g.horizonRow(K.cx);
    double farRow = std::max(hz + 40.0, K.height * 0.45);
    double pts[4][2] = {{0.0, (double)K.height - 1}, {(double)K.width - 1, (double)K.height - 1}, {(double)K.width - 1, farRow}, {0.0, farRow}};
    std::vector<cv::Point> poly;
    double psi = mu::deg2rad(psi_);
    for (auto& pt : pts) {
        double X, Y;
        if (!g.pixelToGround(pt[0], pt[1], X, Y)) continue;
        X = std::min(X, 5.0);
        double wx = x_ + X * std::cos(psi) - Y * std::sin(psi), wy = y_ + X * std::sin(psi) + Y * std::cos(psi);
        poly.push_back(toPx(wx, wy));
    }
    if (poly.size() == 4) cv::polylines(top, poly, true, cv::Scalar(255, 120, 0), 1, cv::LINE_AA);
    cv::Point d = toPx(x_, y_);
    cv::circle(top, d, 6, cv::Scalar(0, 0, 255), -1);
    cv::line(top, d, toPx(x_ + 0.4 * std::cos(psi), y_ + 0.4 * std::sin(psi)), cv::Scalar(0, 0, 255), 2);
    char buf[128];
    snprintf(buf, sizeof(buf), "sim t=%.1fs z=%.2f yaw=%.0f  %s", t_, z_, psi_, res_.reason.c_str());
    cv::putText(top, buf, cv::Point(8, 18), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    return top;
}
