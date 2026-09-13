#include "vision/Perception.h"

#include <chrono>

#include <opencv2/imgproc.hpp>

#include "util/Log.h"
#include "util/MathUtil.h"

static double msSince(const std::chrono::steady_clock::time_point& t0) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
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
    LOG_INFO("Perception: camera %dx%d f=%.0f pitch=%.1f deg  hfov=%.0f vfov=%.0f  blind zone @%.2fm = %.2fm",
             K.width, K.height, K.fx, proj_.cameraPitchDeg(), K.hfovDeg(), K.vfovDeg(), fixedHeightM_, proj_.blindZoneX());
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
    if (autoCalib_ && calib_.samples >= autoCalibMinSamples_ && calib_.pitchDeg > -5 && calib_.pitchDeg < 85 &&   // up to 85: a mirror mount aims the front camera 60-80 deg down
        std::fabs(calib_.rollDeg) < 15) {
        bool first = !calib_.applied;
        proj_.setCameraMount(calib_.pitchDeg, calib_.rollDeg);
        calib_.applied = true;
        if (first || calib_.samples % 30 == 0)
            LOG_INFO("Ground auto-calibration: camera pitch=%.1f roll=%.1f deg, height(tag)=%.2fm ratio(tag/tof)=%.2f (%d samples)",
                     calib_.pitchDeg, calib_.rollDeg, calib_.heightM, calib_.heightRatio, calib_.samples);
    }
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
    proj_.setHeight(H);
    if (useImu_ && st.valid) proj_.setBodyAttitude(imuPitchSign_ * st.pitch, imuRollSign_ * st.roll);
    else proj_.setBodyAttitude(0, 0);
    obs.heightM = H;
    obs.totalPitchDeg = proj_.totalPitchDeg();
    obs.blindZoneM = proj_.blindZoneX();

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
    if (frameCount_ % tagEveryN_ == 0) {
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
        TagObservation t;
        t.id = d.id;
        t.margin = d.margin;
        t.center = d.center;
        for (int k = 0; k < 4; ++k) t.corners[k] = d.corners[k];
        t.poseOk = d.hasPose;
        bool havePos = false;
        if (d.hasPose && tagPosSource_ == "pose") {
            cv::Vec3d pb = proj_.camToBody(d.t);
            t.X = pb[0]; t.Y = pb[1];
            havePos = t.X > 0.05;
            cv::Vec3d xt = proj_.camToBody(cv::Vec3d(d.R(0, 0), d.R(1, 0), d.R(2, 0)));
            t.yawDeg = mu::rad2deg(std::atan2(xt[1], xt[0]));
            double p, r, h;
            if (GroundProjector::groundFromTagPose(d.R, d.t, p, r, h)) t.heightFromPose = h;
        }
        if (!havePos) havePos = proj_.pixelToGround(d.center.x, d.center.y, t.X, t.Y);
        if (!havePos) continue;
        t.dist = std::sqrt(t.X * t.X + t.Y * t.Y);
        if (t.dist > tagMaxDist_) continue;
        obs.tags.push_back(t);
        if (d.hasPose) updateCalib(d, st);
    }

    // Line
    auto t1 = std::chrono::steady_clock::now();
    obs.line = line_.detect(frame, proj_, polys, prev_.valid ? &prev_ : nullptr, hintValid_ ? &hint_ : nullptr);
    obs.lineMs = msSince(t1);
    prev_ = obs.line;
    return obs;
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
    line_.drawDebug(frame, obs.line, proj_);
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
