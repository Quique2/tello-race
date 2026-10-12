#include "vision/Perception.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>

#include <opencv2/imgproc.hpp>

#include "util/Log.h"
#include "util/MathUtil.h"

static double msSince(const std::chrono::steady_clock::time_point& t0) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

bool Perception::courseMode(const Config& cfg) {
    std::string m = toLower(cfg.getString("nav.mode", "auto"));
    if (m == "course") return true;
    if (m == "line") return false;
    return cfg.getDouble("camera.pitch_deg", 10.0) >= cfg.getDouble("nav.steep_pitch_deg", 75.0);
}

bool Perception::init(const Config& cfg, std::string* err) {
    Intrinsics K;
    K.width = cfg.getInt("camera.width", 960);
    K.height = cfg.getInt("camera.height", 720);
    K.fx = cfg.getDouble("camera.fx", 920);
    K.fy = cfg.getDouble("camera.fy", 920);
    K.cx = cfg.getDouble("camera.cx", K.width / 2.0);
    K.cy = cfg.getDouble("camera.cy", K.height / 2.0);
    proj_.setIntrinsics(K);
    proj_.setCameraMount(cfg.getDouble("camera.pitch_deg", 10.0), cfg.getDouble("camera.roll_deg", 0.0));
    heightSource_ = toLower(cfg.getString("ground.height_source", "tof"));
    fixedHeightM_ = cfg.getDouble("mission.altitude_cm", 50) / 100.0;
    proj_.setHeight(fixedHeightM_);
    useImu_ = cfg.getBool("ground.use_imu_attitude", true);
    imuPitchSign_ = cfg.getDouble("ground.imu_pitch_sign", 1.0);
    imuRollSign_ = cfg.getDouble("ground.imu_roll_sign", 1.0);
    attDelayS_ = cfg.getDouble("ground.attitude_delay_s", 0.0);
    imuVerified_ = cfg.getBool("ground.imu_signs_verified", true);
    mountVerified_ = cfg.getBool("ground.mount_verified", true);
    if (useImu_ && !imuVerified_)
        LOG_INFO("Perception: the lean of the drone is left out of the floor model until the tags confirm the telemetry signs");
    autoCalib_ = cfg.getBool("ground.auto_calib", true);
    autoCalibMinSamples_ = cfg.getInt("ground.auto_calib_min_samples", 12);
    calibMaxDist_ = cfg.getDouble("ground.auto_calib_max_dist_m", 2.2);
    calibMinMargin_ = cfg.getDouble("ground.auto_calib_min_margin", 40);

    tagSize_ = cfg.getDouble("tag.size_m", 0.16);
    tagEveryN_ = std::max(1, cfg.getInt("tag.every_n_frames", 1));
    tagMinMargin_ = cfg.getDouble("tag.min_margin", 25);
    tagMaxDist_ = cfg.getDouble("tag.max_dist_m", 3.0);
    tagPosSource_ = toLower(cfg.getString("tag.position_source", "pose"));
    if (!tags_.init(cfg.getString("tag.family", "tag36h11"), cfg.getDouble("tag.decimate", 1.5),
                    cfg.getDouble("tag.blur", 0.0), cfg.getInt("tag.threads", 4), cfg.getBool("tag.refine_edges", true),
                    cfg.getDouble("tag.sharpening", 0.25), err))
        return false;
    line_.configure(cfg);
    tracer_.configure(cfg);
    pathMode_ = courseMode(cfg);
    tagWindowsOn_ = cfg.getBool("tag.windows", true);
    tagDarkV_ = cfg.getInt("tag.dark_v_max", tagDarkV_);
    tagFullEveryS_ = cfg.getDouble("tag.full_frame_every_s", tagFullEveryS_);
    heightFromTags_ = cfg.getBool("ground.height_from_tags", false);
    mountPitch0_ = proj_.cameraPitchDeg();
    mountRoll0_ = proj_.cameraRollDeg();
    LOG_INFO("Perception: camera %dx%d f=%.0f pitch=%.1f deg  hfov=%.0f vfov=%.0f  blind zone @%.2fm = %.2fm%s",
             K.width, K.height, K.fx, proj_.cameraPitchDeg(), K.hfovDeg(), K.vfovDeg(), fixedHeightM_, proj_.blindZoneX(),
             pathMode_ ? "  [course mode: top-down path tracing]" : "");
    return true;
}

void Perception::updateCalib(const TagDetection& d, const TelloState& st) {
    double pitch, roll, h;
    if (!GroundProjector::groundFromTagPose(d.R, d.t, pitch, roll, h)) return;
    double dist = std::sqrt(d.t.dot(d.t));
    if (dist > calibMaxDist_ || d.margin < calibMinMargin_ || h < 0.1 || h > 4.0) return;
    if (st.valid && (std::abs(st.pitch) > 6 || std::abs(st.roll) > 6)) return;   // only when the airframe is level
    double bodyPitch = (useImu_ && st.valid) ? imuPitchSign_ * st.pitch : 0.0;
    double bodyRoll = (useImu_ && st.valid) ? imuRollSign_ * st.roll : 0.0;
    pitchS_.push_back(pitch + bodyPitch);
    rollS_.push_back(roll - bodyRoll);
    heightS_.push_back(h);
    double hs = st.heightCm() > 0 ? st.heightCm() / 100.0 : -1;
    if (hs > 0) ratioS_.push_back(h / hs);
    const size_t maxN = 60;
    while (pitchS_.size() > maxN) { pitchS_.pop_front(); rollS_.pop_front(); heightS_.pop_front(); }
    while (ratioS_.size() > maxN) ratioS_.pop_front();
    calib_.samples = (int)pitchS_.size();
    calib_.pitchDeg = mu::median(std::vector<double>(pitchS_.begin(), pitchS_.end()));
    calib_.rollDeg = mu::median(std::vector<double>(rollS_.begin(), rollS_.end()));
    calib_.heightM = mu::median(std::vector<double>(heightS_.begin(), heightS_.end()));
    calib_.heightRatio = ratioS_.empty() ? 0 : mu::median(std::vector<double>(ratioS_.begin(), ratioS_.end()));
    if (autoCalib_ && calib_.samples >= autoCalibMinSamples_ && calib_.pitchDeg > -5 && calib_.pitchDeg < 135 &&   // a mirror mount can aim the camera straight down and beyond (looking backwards)
        std::fabs(calib_.rollDeg) < 15) {
        bool first = !calib_.applied;
        proj_.setCameraMount(calib_.pitchDeg, calib_.rollDeg);
        calib_.applied = true;
        // The sample window is capped, so "samples % 30 == 0" held on every frame once it was full (14 lines/s):
        // log on the first application and then at most every 2 s.
        static double lastCalibLog = 0;
        double nowS = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        if (first || nowS - lastCalibLog > 2.0) {
            lastCalibLog = nowS;
            LOG_INFO("Ground auto-calibration: camera pitch=%.1f roll=%.1f deg, height(tag)=%.2fm ratio(tag/tof)=%.2f (%d samples)",
                     calib_.pitchDeg, calib_.rollDeg, calib_.heightM, calib_.heightRatio, calib_.samples);
        }
    }
}

// Lean of the airframe at time t, between the telemetry packets around it (they come 10 times a second, in whole degrees).
void Perception::attitudeAt(double t, const TelloState& st, double& pitch, double& roll) const {
    pitch = st.pitch; roll = st.roll;
    if (attHist_.size() < 2) return;
    if (t >= attHist_.back().t) { pitch = attHist_.back().pitch; roll = attHist_.back().roll; return; }
    for (size_t i = attHist_.size() - 1; i > 0; --i) {
        const AttSample& a = attHist_[i - 1];
        const AttSample& b = attHist_[i];
        if (t < a.t) continue;
        const double f = b.t - a.t > 1e-6 ? (t - a.t) / (b.t - a.t) : 1.0;
        pitch = a.pitch + f * (b.pitch - a.pitch);
        roll = a.roll + f * (b.roll - a.roll);
        return;
    }
    pitch = attHist_.front().pitch; roll = attHist_.front().roll;
}

Observation Perception::process(const cv::Mat& frame, const TelloState& st, double stamp, uint64_t seq) {
    Observation obs;
    obs.stamp = stamp;
    obs.seq = seq;
    ++frameCount_;

    // ground model for this frame
    double H = fixedHeightM_;
    if (heightSource_ == "tof" && st.valid && st.heightCm() > 0) H = st.heightCm() / 100.0;
    else if (heightSource_ == "tag" && calib_.samples > 0) H = calib_.heightM;
    if (pathMode_ && heightFromTags_) H *= heightScale_;
    proj_.setHeight(H);
    double leanPitch = st.pitch, leanRoll = st.roll;
    if (st.valid) {
        if (attHist_.empty() || st.stamp != attHist_.back().t) attHist_.push_back({st.stamp, (double)st.pitch, (double)st.roll});
        while (attHist_.size() > 40) attHist_.pop_front();
        if (attDelayS_ > 0) attitudeAt(stamp - attDelayS_, st, leanPitch, leanRoll);
    }
    leanPitch_ = leanPitch; leanRoll_ = leanRoll;
    if (useImu_ && st.valid) proj_.setBodyAttitude(effPitchSign() * leanPitch, effRollSign() * leanRoll);
    else proj_.setBodyAttitude(0, 0);
    obs.heightM = H;
    obs.totalPitchDeg = proj_.totalPitchDeg();
    obs.blindZoneM = proj_.blindZoneX();

    if (pathMode_ && !downward_) return processCourse(frame, st, obs);

    // AprilTags on the full-resolution gray image
    std::vector<std::vector<cv::Point2f>> polys;
    if (downward_) {   // belly camera: 320x240 grey, unknown intrinsics -> detect IDs only, no floor model
        auto t0 = std::chrono::steady_clock::now();
        cv::Mat gray;
        if (frame.channels() == 3) cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY); else gray = frame;
        lastRaw_ = tags_.detect(gray, nullptr, 0);
        obs.tagMs = msSince(t0);
        for (const auto& d : lastRaw_) {
            if (d.margin < tagMinMargin_ || d.hamming > 0) continue;
            TagObservation t;
            t.id = d.id; t.margin = d.margin; t.center = d.center;
            for (int k = 0; k < 4; ++k) t.corners[k] = d.corners[k];
            obs.tags.push_back(t);
        }
        return obs;
    }
    const bool tagFrame = frameCount_ % tagEveryN_ == 0;
    obs.tagsFresh = tagFrame;
    if (tagFrame) {
        auto t0 = std::chrono::steady_clock::now();
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        lastRaw_ = tags_.detect(gray, &proj_.K(), tagSize_);
        obs.tagMs = msSince(t0);
    }
    // Remember tags briefly after they stop being decoded (partially visible at the image border):
    // their black border would otherwise be taken for a line. Extrapolate with the last image motion.
    for (auto& m : tagMem_) m.seenNow = false;
    for (const auto& d : lastRaw_) {
        if (d.margin < tagMinMargin_ || d.hamming > 0) continue;
        cv::Point2f c(0, 0);
        for (int k = 0; k < 4; ++k) c += cv::Point2f((float)d.corners[k].x, (float)d.corners[k].y);
        c *= 0.25f;
        TagMemory* mem = nullptr;
        for (auto& m : tagMem_) if (m.id == d.id) mem = &m;
        if (!mem) { tagMem_.push_back(TagMemory()); mem = &tagMem_.back(); mem->id = d.id; mem->vel = cv::Point2f(0, 0); mem->center = c; }
        else mem->vel = 0.5f * mem->vel + 0.5f * (c - mem->center);
        mem->center = c;
        for (int k = 0; k < 4; ++k) mem->corners[k] = cv::Point2f((float)d.corners[k].x, (float)d.corners[k].y);
        mem->lastSeen = stamp;
        mem->seenNow = true;
    }
    for (auto it = tagMem_.begin(); it != tagMem_.end();) {
        if (stamp - it->lastSeen > 0.7) { it = tagMem_.erase(it); continue; }
        if (!it->seenNow) {
            cv::Point2f shift = it->vel;                         // one frame of motion per frame not seen
            it->center += shift;
            std::vector<cv::Point2f> poly;
            for (int k = 0; k < 4; ++k) {
                it->corners[k] += shift;
                cv::Point2f d = it->corners[k] - it->center;
                poly.push_back(it->center + d * 1.35f);          // grow: the tag gets bigger as it approaches
            }
            polys.push_back(poly);
        }
        ++it;
    }
    for (const auto& d : lastRaw_) {
        std::vector<cv::Point2f> poly;
        for (int k = 0; k < 4; ++k) poly.emplace_back((float)d.corners[k].x, (float)d.corners[k].y);
        polys.push_back(poly);
        if (d.margin < tagMinMargin_ || d.hamming > 0) continue;
        if (pathMode_ && !tagFrame) continue;        // stale corners: good enough to mask the tag, not to steer on
        TagObservation t;
        t.id = d.id;
        t.margin = d.margin;
        t.center = d.center;
        for (int k = 0; k < 4; ++k) t.corners[k] = d.corners[k];
        t.poseOk = d.hasPose;
        bool havePos = false;
        if (pathMode_) {
            // the tag lies on the floor like the tape: locate it with the same floor model, in any direction
            havePos = proj_.pixelToGroundAny(d.center.x, d.center.y, t.X, t.Y);
            if (d.hasPose) {
                cv::Vec3d xt = proj_.camToBody(cv::Vec3d(d.R(0, 0), d.R(1, 0), d.R(2, 0)));
                t.yawDeg = mu::rad2deg(std::atan2(xt[1], xt[0]));
                double p, r, h;
                if (GroundProjector::groundFromTagPose(d.R, d.t, p, r, h)) t.heightFromPose = h;
            }
        } else if (d.hasPose && tagPosSource_ == "pose") {
            cv::Vec3d pb = proj_.camToBody(d.t);
            t.X = pb[0]; t.Y = pb[1];
            havePos = t.X > 0.05;
            cv::Vec3d xt = proj_.camToBody(cv::Vec3d(d.R(0, 0), d.R(1, 0), d.R(2, 0)));
            t.yawDeg = mu::rad2deg(std::atan2(xt[1], xt[0]));
            double p, r, h;
            if (GroundProjector::groundFromTagPose(d.R, d.t, p, r, h)) t.heightFromPose = h;
        }
        if (!havePos && !pathMode_) havePos = proj_.pixelToGround(d.center.x, d.center.y, t.X, t.Y);
        if (!havePos) continue;
        t.dist = std::sqrt(t.X * t.X + t.Y * t.Y);
        if (t.dist > tagMaxDist_) continue;
        obs.tags.push_back(t);
        if (d.hasPose) updateCalib(d, st);
    }

    // Line
    auto t1 = std::chrono::steady_clock::now();
    if (pathMode_) {
        obs.pathMode = true;
        line_.maskOnly(frame, proj_, polys);
        std::vector<cv::Point2d> tagPts;
        for (const auto& t : obs.tags) tagPts.emplace_back(t.X, t.Y);
        obs.paths = tracer_.trace(line_.mask(), line_.scale(), proj_, tagPts);
        obs.lineMs = msSince(t1);
        prev_ = LineObservation();
        return obs;
    }
    obs.line = line_.detect(frame, proj_, polys, prev_.valid ? &prev_ : nullptr, hintValid_ ? &hint_ : nullptr);
    obs.lineMs = msSince(t1);
    prev_ = obs.line;
    return obs;
}

// ---------------------------------------------------------------------------------------------------------
// Course mode. The tape first (cheap, at reduced resolution), then the tags, searched only where a tag can be:
// around the ones already followed and around dark squares of about the right size. On a speckled floor the
// whole-frame search costs ten times more, nearly all of it spent on the speckles.
// ---------------------------------------------------------------------------------------------------------
Observation Perception::processCourse(const cv::Mat& frame, const TelloState& st, Observation obs) {
    obs.pathMode = true;
    if (shiftPending_) { obs.mountChanged = true; obs.mountShift = pendingShift_; shiftPending_ = false; pendingShift_ = cv::Point2d(0, 0); }
    obs.mountTrusted = mountTrusted();
    obs.mountPitchDeg = proj_.cameraPitchDeg();
    const Intrinsics& K = proj_.K();
    auto t1 = std::chrono::steady_clock::now();
    std::vector<std::vector<cv::Point2f>> polys;
    if (line_.color() == "dark" || line_.color() == "hsv")     // black tape: the black of a tag must not count as tape
        for (const auto& d : lastRaw_) {
            std::vector<cv::Point2f> poly;
            for (int k = 0; k < 4; ++k) poly.emplace_back((float)d.corners[k].x, (float)d.corners[k].y);
            polys.push_back(poly);
        }
    line_.maskOnly(frame, proj_, polys);
    obs.paths = tracer_.trace(line_.mask(), line_.scale(), proj_, lastTagGround_);
    {   // the piece of floor in the picture, a little inside its border
        const double m = 12.0;
        const double us[4] = {m, K.width - 1 - m, K.width - 1 - m, m}, vs[4] = {m, m, K.height - 1 - m, K.height - 1 - m};
        for (int k = 0; k < 4; ++k) {
            double X, Y;
            if (!proj_.pixelToGroundAny(us[k], vs[k], X, Y)) { obs.view.clear(); break; }
            obs.view.emplace_back(X, Y);
        }
    }
    obs.lineMs = msSince(t1);

    const bool tagFrame = frameCount_ % tagEveryN_ == 0;
    obs.tagsFresh = tagFrame;
    if (!tagFrame) return obs;

    auto t0 = std::chrono::steady_clock::now();
    std::vector<TagDetection> dets;
    std::vector<cv::Rect> wins;
    if (tagWindowsOn_) wins = tagWindows(frame.size(), obs.stamp);
    double winArea = 0;
    for (const auto& w : wins) winArea += (double)w.area();
    // the whole frame: windows switched off or covering most of it anyway, and now and then as a safety net
    const bool full = !tagWindowsOn_ || winArea > 0.6 * frame.cols * frame.rows || obs.stamp - lastFullFrameT_ >= tagFullEveryS_;
    cv::Mat gray;
    if (full) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        dets = tags_.detect(gray, &K, tagSize_);
        lastFullFrameT_ = obs.stamp;
        lastWindows_.clear();
        tagWindowCount_ = -1;
    } else {
        // size of a tag in the picture: decimate as far as it stays 36+ pixels across
        cv::Vec3d ray = proj_.pixelRayBody(K.cx, K.cy);
        const double tagPx = ray[2] < -0.05 ? tagSize_ * K.fx / (proj_.height() / -ray[2]) : 60.0;
        const double dec = tagPx >= 108 ? 3.0 : tagPx >= 72 ? 2.0 : tagPx >= 54 ? 1.5 : 1.0;
        for (const auto& w : wins) {
            cv::cvtColor(frame(w), gray, cv::COLOR_BGR2GRAY);
            Intrinsics Kw = K;
            Kw.cx -= w.x; Kw.cy -= w.y;
            for (auto d : tags_.detect(gray, &Kw, tagSize_, dec)) {
                d.center += cv::Point2d(w.x, w.y);
                for (int k = 0; k < 4; ++k) d.corners[k] += cv::Point2d(w.x, w.y);
                bool dup = false;
                for (auto& e : dets)
                    if (e.id == d.id) { dup = true; if (d.margin > e.margin) e = d; }
                if (!dup) dets.push_back(d);
            }
        }
        lastWindows_ = wins;
        tagWindowCount_ = (int)wins.size();
    }
    lastRaw_ = dets;

    for (auto& m : tagMem_) m.seenNow = false;
    lastTagGround_.clear();
    for (const auto& d : dets) {
        if (d.margin < tagMinMargin_ || d.hamming > 0) continue;
        cv::Point2f c((float)d.center.x, (float)d.center.y);
        TagMemory* mem = nullptr;
        for (auto& m : tagMem_) if (m.id == d.id) mem = &m;
        if (!mem) { tagMem_.push_back(TagMemory()); mem = &tagMem_.back(); mem->id = d.id; mem->vel = cv::Point2f(0, 0); }
        else {
            const double dtS = std::max(1e-3, obs.stamp - mem->lastSeen);
            const cv::Point2f v = (c - mem->center) * (float)(1.0 / dtS);       // pixels per second
            mem->vel = dtS < 0.3 ? 0.5f * mem->vel + 0.5f * v : cv::Point2f(0, 0);
        }
        mem->center = c;
        for (int k = 0; k < 4; ++k) mem->corners[k] = cv::Point2f((float)d.corners[k].x, (float)d.corners[k].y);
        mem->lastSeen = obs.stamp;
        mem->seenNow = true;

        TagObservation t;
        t.id = d.id;
        t.margin = d.margin;
        t.center = d.center;
        for (int k = 0; k < 4; ++k) t.corners[k] = d.corners[k];
        t.poseOk = d.hasPose;
        // the tag lies on the floor like the tape: locate it with the same floor model, in any direction
        if (!proj_.pixelToGroundAny(d.center.x, d.center.y, t.X, t.Y)) continue;
        if (d.hasPose) {
            cv::Vec3d xt = proj_.camToBody(cv::Vec3d(d.R(0, 0), d.R(1, 0), d.R(2, 0)));
            t.yawDeg = mu::rad2deg(std::atan2(xt[1], xt[0]));
            double p, r, h;
            if (GroundProjector::groundFromTagPose(d.R, d.t, p, r, h)) t.heightFromPose = h;
        }
        t.dist = std::sqrt(t.X * t.X + t.Y * t.Y);
        if (t.dist > tagMaxDist_) continue;
        obs.tags.push_back(t);
        lastTagGround_.emplace_back(t.X, t.Y);
        if (d.hasPose) updatePlaneCalib(d, st);
    }
    for (auto it = tagMem_.begin(); it != tagMem_.end();)
        if (obs.stamp - it->lastSeen > 1.0) it = tagMem_.erase(it); else ++it;
    obs.tagMs = msSince(t0);
    return obs;
}

// Windows (full-resolution pixels) worth a tag search: the tags seen a moment ago, moved on by their image speed,
// and every dark, roughly square blob of about a tag's size (on a taped course a tag is the only such thing).
std::vector<cv::Rect> Perception::tagWindows(const cv::Size& full, double stamp) {
    std::vector<cv::Rect> wins;
    const cv::Rect frameRect(0, 0, full.width, full.height);
    auto add = [&](cv::Rect r) {
        r &= frameRect;
        if (r.width < 40 || r.height < 40) return;
        for (auto& w : wins)
            if ((w & r).area() > 0.3 * std::min(w.area(), r.area())) { w |= r; return; }   // overlapping: one window
        wins.push_back(r);
    };
    for (const auto& m : tagMem_) {
        const double age = stamp - m.lastSeen;
        if (age > 0.7) continue;
        const cv::Point2f c = m.center + m.vel * (float)age;
        double diag = 0;
        for (int k = 0; k < 2; ++k) diag = std::max(diag, (double)cv::norm(m.corners[k] - m.corners[k + 2]));
        const double half = 0.62 * diag + 14.0 + cv::norm(m.vel) * age;
        add(cv::Rect((int)(c.x - half), (int)(c.y - half), (int)(2 * half), (int)(2 * half)));
    }
    const cv::Mat& hsv = line_.hsv();
    if (hsv.empty()) return wins;
    const LineParams& lp = line_.params();
    const double sc = line_.scale();
    cv::Vec3d ray = proj_.pixelRayBody(proj_.K().cx, proj_.K().cy);
    if (ray[2] > -0.05) return wins;
    // the search runs at half the processing size: a tag is still 30 pixels there, the floor speckles are gone
    cv::Mat small, dark, tape;
    cv::resize(hsv, small, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
    const double ss = sc * 0.5;
    // "dark" follows the exposure too: under half the brightness of the white surface when that is known
    const int darkV = line_.whiteLevel() > 0 ? std::min(tagDarkV_, (int)std::lround(0.47 * line_.whiteLevel())) : tagDarkV_;
    cv::inRange(small, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, darkV), dark);
    cv::inRange(small, lp.blueLo, lp.blueHi, tape);               // dark blue tape is not a tag
    dark.setTo(0, tape);
    // size of a tag at the image centre (it changes a little over the frame: the limits are loose)
    const double tagPx = tagSize_ * proj_.K().fx * ss / (proj_.height() / -ray[2]);
    const double lo = 0.45 * tagPx, hi = 2.2 * tagPx;
    cv::Mat labels, stats, cents;
    const int n = cv::connectedComponentsWithStats(dark, labels, stats, cents, 8, CV_32S);
    for (int i = 1; i < n; ++i) {
        const int w = stats.at<int>(i, cv::CC_STAT_WIDTH), h = stats.at<int>(i, cv::CC_STAT_HEIGHT), area = stats.at<int>(i, cv::CC_STAT_AREA);
        const int big = std::max(w, h), sm = std::min(w, h);
        if (big < lo || big > hi || sm < 0.5 * lo) continue;
        if (area < 0.2 * w * h) continue;                          // a thin diagonal thing, not a square
        const double pad = 0.45 * big + 4;
        const int x0 = stats.at<int>(i, cv::CC_STAT_LEFT), y0 = stats.at<int>(i, cv::CC_STAT_TOP);
        add(cv::Rect((int)((x0 - pad) / ss), (int)((y0 - pad) / ss), (int)((w + 2 * pad) / ss), (int)((h + 2 * pad) / ss)));
    }
    return wins;
}

// Every tag centre is a point of the floor. Seen at many places in the picture, those points give the floor plane
// in camera coordinates: its normal is the camera tilt (to a fraction of a degree, where the apparent tilt of one
// small tag is only good to several degrees) and its distance compared with the height sensor gives the scale.
void Perception::updatePlaneCalib(const TagDetection& d, const TelloState& st) {
    if (!autoCalib_ || !st.valid || st.heightCm() <= 0) return;
    const double H = st.heightCm() / 100.0;
    if (st.stamp != planeStateStamp_) {       // a new telemetry packet: how fast is the height changing?
        if (planeStateStamp_ >= 0) planeClimb_ = (H - planeLastH_) / std::max(1e-3, st.stamp - planeStateStamp_);
        planeLastH_ = H;
        planeStateStamp_ = st.stamp;
    }
    if (std::fabs(planeClimb_) > 0.08) return;                    // climbing: the height of this packet is not the picture's
    if (d.margin < calibMinMargin_ || d.t[2] < 0.1 || d.t[2] > 4.0) return;
    if (std::fabs(leanPitch_) > 8 || std::fabs(leanRoll_) > 8) return;
    auto it = planeLastPx_.find(d.id);
    if (it != planeLastPx_.end() && cv::norm(it->second - d.center) < 15) return;   // seen at the same place: nothing new
    planeLastPx_[d.id] = d.center;
    plane_.push_back({d.id, d.t, H, leanPitch_, leanRoll_});
    while (plane_.size() > 500) plane_.pop_front();
    if (++planeNew_ >= 5 && plane_.size() >= 20) { planeNew_ = 0; solvePlane(); }
}

// Least squares over the tag sightings i:  scale_k * up(pitch - sp * lean_pitch_i, roll + sr * lean_roll_i) . q_i + H_i = 0
// with up() the floor normal in camera coordinates and one scale per tag k (tags are nominally the same size, but a
// tag printed a little off, or seen through a mirror that is not perfectly flat, must not pass for a tilt).
// Starts from pitchDeg / rollDeg / scale and returns them refined; scale is the mean over the sightings.
bool Perception::fitPlane(double signPitch, double signRoll, bool robust, double& pitchDeg, double& rollDeg, double& scale,
                          double& rms, int& used) const {
    const size_t n = plane_.size();
    if (n < 12) return false;
    std::vector<int> ids, col(n, 0);
    for (size_t i = 0; i < n; ++i) {
        auto it = std::find(ids.begin(), ids.end(), plane_[i].id);
        if (it == ids.end()) { ids.push_back(plane_[i].id); col[i] = (int)ids.size() - 1; }
        else col[i] = (int)(it - ids.begin());
    }
    const int K = (int)ids.size(), N = 2 + K;
    double th = mu::deg2rad(pitchDeg), ph = mu::deg2rad(rollDeg);
    std::vector<double> c(K, scale);
    const double th0 = mu::deg2rad(mountPitch0_), ph0 = mu::deg2rad(mountRoll0_);
    // weak pulls, written against 1 cm of measurement noise: towards the configured mount (6 deg), towards a scale
    // of 1 (15 %) and of the tags towards each other (1.5 %). They only matter while the sightings are few.
    const double wAng = std::pow(0.01 / mu::deg2rad(6.0), 2), wAbs = std::pow(0.01 / 0.15, 2), wTie = std::pow(0.01 / 0.015, 2);
    std::vector<char> use(n, 1);
    std::vector<double> res(n, 0.0);
    auto upAt = [&](const PlaneSample& s, double thv, double phv, cv::Vec3d* dTh, cv::Vec3d* dPh) {
        const double t = thv - mu::deg2rad(signPitch * s.pitch), p = phv + mu::deg2rad(signRoll * s.roll);
        const double sT = std::sin(t), cT = std::cos(t), sP = std::sin(p), cP = std::cos(p);
        if (dTh) *dTh = cv::Vec3d(0, cP * sT, -cP * cT);
        if (dPh) *dPh = cv::Vec3d(-cP, sP * cT, sP * sT);
        return cv::Vec3d(-sP, -cP * cT, -cP * sT);
    };
    for (int iter = 0; iter < 8; ++iter) {
        cv::Mat A = cv::Mat::zeros(N, N, CV_64F), b = cv::Mat::zeros(N, 1, CV_64F);
        int cnt = 0;
        for (size_t i = 0; i < n; ++i) {
            if (!use[i]) continue;
            const PlaneSample& s = plane_[i];
            cv::Vec3d dTh, dPh;
            const cv::Vec3d u = upAt(s, th, ph, &dTh, &dPh);
            const double ci = c[col[i]], uq = u.dot(s.q);
            const double r = ci * uq + s.H;
            const double J[3] = {ci * dTh.dot(s.q), ci * dPh.dot(s.q), uq};
            const int ix[3] = {0, 1, 2 + col[i]};
            for (int p = 0; p < 3; ++p) {
                b.at<double>(ix[p]) -= J[p] * r;
                for (int q = 0; q < 3; ++q) A.at<double>(ix[p], ix[q]) += J[p] * J[q];
            }
            ++cnt;
        }
        if (cnt < 12) return false;
        A.at<double>(0, 0) += wAng; b.at<double>(0) += wAng * (th0 - th);
        A.at<double>(1, 1) += wAng; b.at<double>(1) += wAng * (ph0 - ph);
        double cMean = 0;
        for (double v : c) cMean += v / K;
        for (int k = 0; k < K; ++k) {
            A.at<double>(2 + k, 2 + k) += wAbs + wTie;
            b.at<double>(2 + k) += wAbs * (1.0 - c[k]) + wTie * (cMean - c[k]);
        }
        cv::Mat d;
        if (!cv::solve(A, b, d, cv::DECOMP_CHOLESKY)) return false;
        th += d.at<double>(0); ph += d.at<double>(1);
        for (int k = 0; k < K; ++k) {
            c[k] += d.at<double>(2 + k);
            if (!(c[k] > 0.3 && c[k] < 3.0)) return false;
        }
        for (size_t i = 0; i < n; ++i) res[i] = c[col[i]] * upAt(plane_[i], th, ph, nullptr, nullptr).dot(plane_[i].q) + plane_[i].H;
        if (robust && iter >= 1 && iter < 7) {
            std::vector<double> a(n);
            for (size_t i = 0; i < n; ++i) a[i] = std::fabs(res[i]);
            const double lim = std::max(0.015, 2.5 * 1.4826 * mu::median(a));
            for (size_t i = 0; i < n; ++i) use[i] = a[i] <= lim;
        }
    }
    double sq = 0, cSum = 0;
    used = 0;
    for (size_t i = 0; i < n; ++i) if (use[i]) { sq += res[i] * res[i]; cSum += c[col[i]]; ++used; }
    if (used < 12) return false;
    rms = std::sqrt(sq / used);
    pitchDeg = mu::rad2deg(th);
    rollDeg = mu::rad2deg(ph);
    scale = cSum / used;
    return std::isfinite(pitchDeg) && std::isfinite(rollDeg) && std::isfinite(scale);
}

// The lean of the airframe as the telemetry counts it, measured against the tags: the same floor fit as fitPlane() with
// two more unknowns, the factors the telemetry pitch and roll have to be multiplied with to make every tag sighting
// fall on one floor plane. Returned with their standard errors.
bool Perception::fitLeanGains(double& gPitch, double& sePitch, double& gRoll, double& seRoll) const {
    const size_t n = plane_.size();
    if (n < 40) return false;
    std::vector<int> ids, col(n, 0);
    for (size_t i = 0; i < n; ++i) {
        auto it = std::find(ids.begin(), ids.end(), plane_[i].id);
        if (it == ids.end()) { ids.push_back(plane_[i].id); col[i] = (int)ids.size() - 1; }
        else col[i] = (int)(it - ids.begin());
    }
    const int K = (int)ids.size(), N = 4 + K;
    double th = mu::deg2rad(proj_.cameraPitchDeg()), ph = mu::deg2rad(proj_.cameraRollDeg());
    double gP = effPitchSign(), gR = effRollSign();
    std::vector<double> c(K, 1.0 / heightScale_);
    const double th0 = mu::deg2rad(mountPitch0_), ph0 = mu::deg2rad(mountRoll0_);
    const double wAng = std::pow(0.01 / mu::deg2rad(6.0), 2), wAbs = std::pow(0.01 / 0.15, 2), wTie = std::pow(0.01 / 0.015, 2);
    const double wGain = std::pow(0.01 / 3.0, 2);                 // only there to keep the system solvable without any lean
    cv::Mat A, bvec;
    double sq = 0;
    for (int iter = 0; iter < 7; ++iter) {
        A = cv::Mat::zeros(N, N, CV_64F);
        bvec = cv::Mat::zeros(N, 1, CV_64F);
        sq = 0;
        for (size_t i = 0; i < n; ++i) {
            const PlaneSample& s = plane_[i];
            const double pr = mu::deg2rad(s.pitch), rr = mu::deg2rad(s.roll);
            const double t = th - gP * pr, p = ph + gR * rr;
            const double sT = std::sin(t), cT = std::cos(t), sP = std::sin(p), cP = std::cos(p);
            const cv::Vec3d u(-sP, -cP * cT, -cP * sT), dTh(0, cP * sT, -cP * cT), dPh(-cP, sP * cT, sP * sT);
            const double ci = c[col[i]], uq = u.dot(s.q), a = ci * dTh.dot(s.q), bq = ci * dPh.dot(s.q);
            const double r = ci * uq + s.H;
            const double J[5] = {a, bq, -pr * a, rr * bq, uq};
            const int ix[5] = {0, 1, 2, 3, 4 + col[i]};
            for (int k = 0; k < 5; ++k) {
                bvec.at<double>(ix[k]) -= J[k] * r;
                for (int m = 0; m < 5; ++m) A.at<double>(ix[k], ix[m]) += J[k] * J[m];
            }
            sq += r * r;
        }
        A.at<double>(0, 0) += wAng; bvec.at<double>(0) += wAng * (th0 - th);
        A.at<double>(1, 1) += wAng; bvec.at<double>(1) += wAng * (ph0 - ph);
        A.at<double>(2, 2) += wGain; bvec.at<double>(2) += wGain * (0.0 - gP);
        A.at<double>(3, 3) += wGain; bvec.at<double>(3) += wGain * (0.0 - gR);
        double cMean = 0;
        for (double v : c) cMean += v / K;
        for (int k = 0; k < K; ++k) {
            A.at<double>(4 + k, 4 + k) += wAbs + wTie;
            bvec.at<double>(4 + k) += wAbs * (1.0 - c[k]) + wTie * (cMean - c[k]);
        }
        cv::Mat d;
        if (!cv::solve(A, bvec, d, cv::DECOMP_CHOLESKY)) return false;
        th += d.at<double>(0); ph += d.at<double>(1);
        gP = mu::clamp(gP + d.at<double>(2), -3.0, 3.0);
        gR = mu::clamp(gR + d.at<double>(3), -3.0, 3.0);
        for (int k = 0; k < K; ++k) {
            c[k] += d.at<double>(4 + k);
            if (!(c[k] > 0.3 && c[k] < 3.0)) return false;
        }
    }
    cv::Mat Ainv;
    if (cv::invert(A, Ainv, cv::DECOMP_SVD) == 0) return false;
    const double s2 = std::max(sq / std::max<double>(1.0, (double)n - N), 0.004 * 0.004);   // never trust the fit below 4 mm
    gPitch = gP; gRoll = gR;
    sePitch = std::sqrt(std::max(0.0, s2 * Ainv.at<double>(2, 2)));
    seRoll = std::sqrt(std::max(0.0, s2 * Ainv.at<double>(3, 3)));
    return std::isfinite(gP) && std::isfinite(gR) && std::isfinite(sePitch) && std::isfinite(seRoll);
}

void Perception::solvePlane() {
    // leverage: the tags must have been seen over a good part of the picture
    // (every tag has its own scale, so what counts is how far each one moved through the picture)
    struct Acc { double n = 0, sx = 0, sy = 0, sxx = 0, syy = 0; };
    std::map<int, Acc> acc;
    for (const auto& s : plane_) {
        const double a = s.q[0] / s.q[2], b = s.q[1] / s.q[2];
        Acc& t = acc[s.id];
        t.n += 1; t.sx += a; t.sy += b; t.sxx += a * a; t.syy += b * b;
    }
    const double n = (double)plane_.size();
    double varX = 0, varY = 0;
    for (const auto& kv : acc) {
        const Acc& t = kv.second;
        varX += t.sxx - t.sx * t.sx / t.n;
        varY += t.syy - t.sy * t.sy / t.n;
    }
    const double stdX = std::sqrt(std::max(0.0, varX / n)), stdY = std::sqrt(std::max(0.0, varY / n));
    // the first estimate replaces the configured angle: it has to stand on more than the later refinements
    const double need = calib_.applied ? 0.08 : 0.10;
    if (stdX < need || stdY < need || (!calib_.applied && plane_.size() < 30)) return;

    // Do the telemetry pitch and roll count the lean the way the floor model assumes? The gain that takes the lean out
    // of the tag sightings best is measured: about +1 when the sign is as configured, about -1 when it is the other
    // way round. Nothing is decided while the lean has not varied enough to show.
    if (useImu_ && plane_.size() >= 40 && (!pitchSignKnown_ || !rollSignKnown_)) {
        double gP = 0, seP = 1e9, gR = 0, seR = 1e9;
        if (fitLeanGains(gP, seP, gR, seR)) {
            auto decide = [&](const char* name, double g, double se, double& sign, bool& known) {
                if (known || std::fabs(g) < 0.4 || std::fabs(g) < 5.0 * se) return;
                const double found = g > 0 ? 1.0 : -1.0;
                if (found != sign)
                    LOG_WARN("Telemetry %s sign is the other way round (lean gain %+.2f +- %.2f against the tags): using ground.imu_%s_sign = %+.0f",
                             name, g, se, name, found);
                else
                    LOG_INFO("Telemetry %s sign confirmed by the tags (lean gain %+.2f +- %.2f)", name, g, se);
                sign = found;
                known = true;
            };
            decide("pitch", gP, seP, imuPitchSign_, pitchSignKnown_);
            decide("roll", gR, seR, imuRollSign_, rollSignKnown_);
        }
    }

    double pitch = proj_.cameraPitchDeg(), roll = proj_.cameraRollDeg(), c = 1.0 / heightScale_, rms = 0;
    int used = 0;
    if (!fitPlane(effPitchSign(), effRollSign(), true, pitch, roll, c, rms, used)) return;
    if (!(pitch > -5 && pitch < 135) || std::fabs(roll) > 15 || rms > 0.05) return;
    const bool first = !calib_.applied;
    if (calib_.applied && std::fabs(pitch - calib_.pitchDeg) < 0.3 && std::fabs(roll - calib_.rollDeg) < 0.3) ++calibStable_;
    else calibStable_ = 0;
    if (calib_.applied && used >= 40 && calibStable_ >= 2) mountSettled_ = true;
    calib_.samples = used;
    calib_.pitchDeg = pitch;
    calib_.rollDeg = roll;
    calib_.heightRatio = 1.0 / c;
    calib_.heightM = 0;
    calib_.applied = true;
    {   // where does the floor point that was straight below the drone go? (what the navigator remembers moves with it)
        double u = 0, v = 0, X = 0, Y = 0;
        const bool had = proj_.groundToPixel(0.0, 0.0, u, v);
        proj_.setCameraMount(pitch, roll);
        if (had && proj_.pixelToGroundAny(u, v, X, Y) && std::hypot(X, Y) > 0.004) {
            pendingShift_ += cv::Point2d(X, Y);
            shiftPending_ = true;
        }
    }
    if (heightFromTags_ && c > 0.8 && c < 1.25) heightScale_ = 1.0 / c;
    static double lastLog = 0;
    const double nowS = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    if (first || nowS - lastLog > 3.0) {
        lastLog = nowS;
        LOG_INFO("Floor-plane calibration: camera pitch=%.1f roll=%.1f deg, height tags/sensor=%.3f, fit %.1f cm over %d tag sightings",
                 pitch, roll, 1.0 / c, 100 * rms, used);
    }
}

void Perception::drawDebug(cv::Mat& frame, const Observation& obs) const {
    if (downward_) {
        for (const auto& t : obs.tags) {
            for (int k = 0; k < 4; ++k)
                cv::line(frame, cv::Point((int)t.corners[k].x, (int)t.corners[k].y),
                         cv::Point((int)t.corners[(k + 1) % 4].x, (int)t.corners[(k + 1) % 4].y), cv::Scalar(255, 200, 0), 2);
            cv::putText(frame, "ID" + std::to_string(t.id), cv::Point((int)t.center.x - 12, (int)t.center.y - 8), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                        cv::Scalar(255, 200, 0), 2);
        }
        cv::putText(frame, "DOWNWARD CAMERA (belly) - tags only", cv::Point(8, frame.rows - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        return;
    }
    double s = (double)frame.cols / proj_.K().width;
    if (obs.pathMode) {
        // every traced piece of tape, thin: the navigator's own route is drawn on top by the HUD
        for (const auto& L : obs.paths) {
            std::vector<cv::Point> px;
            for (const auto& q : L.pts) {
                double u, v;
                if (proj_.groundToPixel(q.x, q.y, u, v)) px.emplace_back((int)(u * s), (int)(v * s));
            }
            if (px.size() >= 2) cv::polylines(frame, px, false, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
            for (int e = 0; e < 2 && !px.empty(); ++e) {      // a real tape end: filled dot; cut by the view: ring
                bool cut = e ? L.cutB : L.cutA;
                cv::circle(frame, e ? px.back() : px.front(), 5, cv::Scalar(255, 255, 255), cut ? 1 : -1, cv::LINE_AA);
            }
        }
    } else {
        line_.drawDebug(frame, obs.line, proj_);
    }
    if (obs.pathMode)   // where the tags were looked for
        for (const auto& w : lastWindows_)
            cv::rectangle(frame, cv::Rect((int)(w.x * s), (int)(w.y * s), (int)(w.width * s), (int)(w.height * s)), cv::Scalar(160, 160, 0), 1);
    for (const auto& t : obs.tags) {
        for (int k = 0; k < 4; ++k) {
            cv::Point a((int)(t.corners[k].x * s), (int)(t.corners[k].y * s));
            cv::Point b((int)(t.corners[(k + 1) % 4].x * s), (int)(t.corners[(k + 1) % 4].y * s));
            cv::line(frame, a, b, cv::Scalar(255, 200, 0), 2);
        }
        char buf[96];
        snprintf(buf, sizeof(buf), "ID%d X=%.2f Y=%.2f m", t.id, t.X, t.Y);
        cv::putText(frame, buf, cv::Point((int)(t.center.x * s) - 40, (int)(t.center.y * s) - 12), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(255, 200, 0), 2);
    }
    if (obs.pathMode) {   // floor grid around the drone, 0.5 m squares, and the point straight below it
        for (double g = -2.0; g <= 2.0; g += 0.5) {
            double u0, v0, u1, v1;
            if (proj_.groundToPixel(g, -2.0, u0, v0) && proj_.groundToPixel(g, 2.0, u1, v1))
                cv::line(frame, cv::Point((int)(u0 * s), (int)(v0 * s)), cv::Point((int)(u1 * s), (int)(v1 * s)), cv::Scalar(90, 60, 60), 1);
            if (proj_.groundToPixel(-2.0, g, u0, v0) && proj_.groundToPixel(2.0, g, u1, v1))
                cv::line(frame, cv::Point((int)(u0 * s), (int)(v0 * s)), cv::Point((int)(u1 * s), (int)(v1 * s)), cv::Scalar(90, 60, 60), 1);
        }
        double u, v;
        if (proj_.groundToPixel(0, 0, u, v)) cv::drawMarker(frame, cv::Point((int)(u * s), (int)(v * s)), cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 22, 2);
        return;
    }
    // ground grid: 0.5 m lines ahead and lateral
    for (double X = 0.5; X <= 3.0; X += 0.5) {
        double u0, v0, u1, v1;
        if (proj_.groundToPixel(X, -1.0, u0, v0) && proj_.groundToPixel(X, 1.0, u1, v1))
            cv::line(frame, cv::Point((int)(u0 * s), (int)(v0 * s)), cv::Point((int)(u1 * s), (int)(v1 * s)), cv::Scalar(90, 60, 60), 1);
    }
    for (double Y = -1.0; Y <= 1.0; Y += 0.5) {
        double u0, v0, u1, v1;
        if (proj_.groundToPixel(0.3, Y, u0, v0) && proj_.groundToPixel(3.0, Y, u1, v1))
            cv::line(frame, cv::Point((int)(u0 * s), (int)(v0 * s)), cv::Point((int)(u1 * s), (int)(v1 * s)), cv::Scalar(90, 60, 60), 1);
    }
}
