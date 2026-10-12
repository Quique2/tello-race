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
    world_ = TrackBuilder::render(track_, style_, tagRenderer);
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
    cmdVx_ = cmdVy_ = cmdVz_ = cmdYaw_ = 0;
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
    cmdVx_ = cmdVy_ = cmdVz_ = cmdYaw_ = 0;
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
    cmdVx_ = mu::clamp(fb, -100, 100) * p_.rcVel;
    cmdVy_ = -mu::clamp(lr, -100, 100) * (p_.rcVelLat > 0 ? p_.rcVelLat : p_.rcVel);
    cmdVz_ = mu::clamp(ud, -100, 100) * p_.rcVz;
    cmdYaw_ = -mu::clamp(yaw, -100, 100) * p_.rcYawRate;
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
            if (dbz > -1e-4) { mx[u] = -1; my[u] = -1; continue; }
            double t = -z_ / dbz;
            if (t > 60.0) { mx[u] = -1; my[u] = -1; continue; }
            double gx = x_ + p_.camAheadM * c + t * (c * dbx - s * dby);
            double gy = y_ + p_.camAheadM * s + t * (s * dbx + c * dby);
            mx[u] = (float)(half + gx * ppm);
            my[u] = (float)(half - gy * ppm);
        }
    }
    cv::Mat img;
    cv::remap(world_, img, mapX_, mapY_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(75, 70, 62));
    if (p_.exposureS > 0) {
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
    } else {
        double speed = std::hypot(vxB_, vyB_) + std::fabs(yawRate_) * 0.012;
        int k = 1 + (int)(speed * p_.blurPerMs);
        if (k > 1) cv::blur(img, img, cv::Size(std::min(k, 9), std::min(k, 9)));
    }
    if (std::fabs(p_.imageGain - 1.0) > 1e-3) img.convertTo(img, -1, p_.imageGain, 0);
    if (p_.noiseSigma > 0) {
        cv::Mat f, n(img.size(), CV_32FC3);
        img.convertTo(f, CV_32FC3);
        cv::randn(n, 0, p_.noiseSigma);
        f += n;
        f.convertTo(img, CV_8UC3);
    }
    return img;
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
