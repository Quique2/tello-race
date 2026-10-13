#include "sim/FlightReplay.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <thread>

#include <opencv2/imgcodecs.hpp>

#include "util/Log.h"
#include "util/MathUtil.h"

namespace {
double wallNow() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

// One unit of the last digit the recorder printed: it writes t with six significant digits.
double printedStep(double t) { return t > 0 ? std::pow(10.0, std::floor(std::log10(t)) - 5.0) : 1e-6; }

// The staircase under a series of camera-angle readings: the fit that finds the angle in flight moves it in steps,
// and between two steps every reading is that step plus, while the lean is changing, a burst of up to a degree that
// lasts a few sightings. So: the steps that leave the least absolute misfit (a burst does not pull a median), a
// reading taken with the lean changing counting a third, and a new step only where it takes more than a degree of
// misfit off (four such readings half a degree out do not buy one; a real step of half a degree in each angle,
// seen twice with the lean standing still, does). No step shorter than two readings. Returns the first reading of
// every step (the first is 0).
std::vector<size_t> staircase(const std::vector<double>& p, const std::vector<double>& r, const std::vector<double>& w) {
    const size_t n = p.size();
    if (n < 4) return {0};
    const double price = 1.0;
    std::vector<std::pair<double, double>> v;
    auto misfit = [&](const std::vector<double>& x, size_t a, size_t b) {   // readings a .. b-1 about their weighted median
        v.clear();
        double total = 0, acc = 0, med = x[a], sum = 0;
        for (size_t k = a; k < b; ++k) { v.emplace_back(x[k], w[k]); total += w[k]; }
        std::sort(v.begin(), v.end());
        for (const auto& e : v) { acc += e.second; if (acc >= 0.5 * total) { med = e.first; break; } }
        for (const auto& e : v) sum += e.second * std::fabs(e.first - med);
        return sum;
    };
    std::vector<double> best(n + 1, 1e300);
    std::vector<size_t> from(n + 1, 0);
    best[0] = -price;
    for (size_t b = 2; b <= n; ++b)
        for (size_t a = 0; a + 2 <= b; ++a) {
            if (best[a] > 1e299) continue;
            const double c = best[a] + price + misfit(p, a, b) + misfit(r, a, b);
            if (c < best[b]) { best[b] = c; from[b] = a; }
        }
    std::vector<size_t> starts;
    for (size_t b = n; b > 0; b = from[b]) starts.insert(starts.begin(), from[b]);
    return starts;
}
}  // namespace

FlightReplay::FlightReplay(const Config& cfg, const std::string& videoPath, const std::string& csvPath, bool realtime)
    : cfg_(cfg), videoPath_(videoPath), csvPath_(csvPath), realtime_(realtime), tagEveryN_(std::max(1, cfg.getInt("tag.every_n_frames", 1))),
      jpegShift_(mu::clamp(cfg.getInt("replay.jpeg_shift", 0), 0, 15)), jpegQuality_(mu::clamp(cfg.getInt("replay.jpeg_quality", 0), 0, 100)),
      follow_(cfg.getBool("replay.follow_camera", cfg.getBool("ground.mount_verified", true))), givenPitch_(cfg.getDouble("camera.pitch_deg", 0)),
      givenRoll_(cfg.getDouble("camera.roll_deg", 0)) {}

bool FlightReplay::loadCsv(std::string* err) {
    auto fail = [&](const std::string& why) { if (err) *err = "replay: " + csvPath_ + ": " + why; return false; };
    std::ifstream f(csvPath_);
    if (!f) return fail("cannot open the telemetry csv");
    std::string line;
    if (!std::getline(f, line)) return fail("empty file");
    const std::vector<std::string> head = splitString(trim(line), ',');
    auto col = [&](const char* name) {
        for (size_t i = 0; i < head.size(); ++i) if (trim(head[i]) == name) return (int)i;
        return -1;
    };
    const int cT = col("t"), cSeq = col("seq"), cPhase = col("phase"), cBat = col("bat"), cTof = col("tof"), cH = col("h"), cYaw = col("yaw"),
              cPitch = col("pitch"), cRoll = col("roll"), cTag = col("tag_id"), cTagX = col("tag_x"), cTagY = col("tag_y"), cRc = col("rc_lr"),
              cStT = col("st_t"), cPaths = col("paths");       // not in the recordings made before the replay existed
    if (cT < 0 || cSeq < 0 || cPhase < 0 || cBat < 0 || cTof < 0 || cH < 0 || cYaw < 0 || cPitch < 0 || cRoll < 0 || cRc < 0 ||
        cRc + 3 >= (int)head.size())
        return fail("not a csv of the ground station's RECORD (columns t, seq, phase, bat, tof, h, yaw, pitch, roll, rc_lr.. expected)");
    std::vector<std::string> lines;
    while (std::getline(f, line))
        if (!trim(line).empty()) lines.push_back(trim(line));
    rows_.clear();
    std::vector<double> stamp;                       // the t column as printed
    for (size_t k = 0; k < lines.size(); ++k) {
        const std::vector<std::string> v = splitString(lines[k], ',');
        if (v.size() < head.size()) {
            // The last line may be cut short (the recording was stopped while it was being written). A short one
            // further up cannot be skipped: every row after it would be put on the wrong frame of the video.
            if (k + 1 == lines.size()) break;
            return fail("line " + std::to_string(k + 2) + " is cut short (" + std::to_string(v.size()) + " of " + std::to_string(head.size()) +
                        " values): the rows after it can no longer be matched to the frames of the video");
        }
        Row r;
        stamp.push_back(atof(v[cT].c_str()));
        r.seq = strtoull(v[cSeq].c_str(), nullptr, 10);
        const std::string ph = trim(v[cPhase]);
        r.ground = ph == "IDLE" || ph == "LANDED" || ph == "ABORTED";
        if (cTag >= 0 && cTagX >= 0 && cTagY >= 0) { r.tagId = atoi(v[cTag].c_str()); r.tagX = atof(v[cTagX].c_str()); r.tagY = atof(v[cTagY].c_str()); }
        r.st.valid = true;
        r.st.bat = atoi(v[cBat].c_str()); r.st.tof = atoi(v[cTof].c_str()); r.st.h = atoi(v[cH].c_str());
        r.st.yaw = atoi(v[cYaw].c_str()); r.st.pitch = atoi(v[cPitch].c_str()); r.st.roll = atoi(v[cRoll].c_str());
        for (int s = 0; s < 4; ++s) r.rc[s] = atoi(v[cRc + s].c_str());
        if (cStT >= 0) r.st.stamp = atof(v[cStT].c_str());
        if (cPaths >= 0) r.paths = atoi(v[cPaths].c_str());
        rows_.push_back(r);
    }
    const size_t n = rows_.size();
    if (n < 3) return fail("no rows");

    // ---- the time of every row. The t column is the frame's own time stamp, but printed with six digits: good to
    // 0.01 s in the first 2 h 46 min the station is up and to 0.1 s after that. seq counts the frames the camera
    // delivered, about 24 a second and evenly. So a straight line through (seq, t) gives the fine timing and t keeps it
    // true: the two are weighted by how much of the scatter of t about the line its rounding does NOT explain, and
    // the result stays within what the printed digits allow.
    double ms = 0, mt = 0, sxx = 0, sxy = 0, res2 = 0;
    for (size_t i = 0; i < n; ++i) { ms += (double)rows_[i].seq / n; mt += stamp[i] / n; }
    for (size_t i = 0; i < n; ++i) { const double ds = rows_[i].seq - ms; sxx += ds * ds; sxy += ds * (stamp[i] - mt); }
    if (sxx <= 0 || sxy <= 0) return fail("seq and t do not run forward together");
    const double perFrame = sxy / sxx;               // seconds per delivered frame
    fps_ = 1.0 / perFrame;
    for (size_t i = 0; i < n; ++i) { const double d = stamp[i] - (mt + perFrame * (rows_[i].seq - ms)); res2 += d * d / n; }
    const double step = printedStep(stamp[n / 2]);
    const double trust = res2 > 0 ? mu::clamp(1.0 - step * step / 12.0 / res2, 0.0, 1.0) : 0.0;   // 1: t as printed, 0: the line
    for (size_t i = 0; i < n; ++i) {
        const double onLine = mt + perFrame * (rows_[i].seq - ms);
        rows_[i].t = mu::clamp(onLine + trust * (stamp[i] - onLine), stamp[i] - 0.5 * step, stamp[i] + 0.5 * step);
        if (i > 0) rows_[i].t = std::max(rows_[i].t, rows_[i - 1].t + 1e-3);
    }

    // ---- the telemetry comes ten times a second and the csv holds what was current at every frame. Perception tells
    // the packets apart by their time. A recording that has no st_t column does not say when they came: a row where
    // a value changed shows a packet that came in after the row before it was written and before this one, and it
    // is put half a frame before this row. While nothing changes one is assumed every 0.1 s.
    double packetT = rows_[0].t, due = packetT + 0.1;
    for (size_t i = 0; i < n && cStT < 0; ++i) {
        Row& r = rows_[i];
        const TelloState& a = r.st;
        const TelloState& b = rows_[i ? i - 1 : 0].st;
        const bool changed = a.bat != b.bat || a.tof != b.tof || a.h != b.h || a.yaw != b.yaw || a.pitch != b.pitch || a.roll != b.roll;
        if (changed) { packetT = r.t - 0.5 * std::min(perFrame, r.t - rows_[i - 1].t); due = packetT + 0.1; }
        else if (r.t >= due) { packetT = std::max(due, r.t - perFrame); due = packetT + 0.1; }
        r.st.stamp = packetT;
    }

    // ---- where the flight is in the recording. The row on which the phase leaves the ground is the frame the loop
    // held while the blocking take-off ran (it still shows the floor from the ground, and the frames of those
    // seconds were never looked at); the row on which it is back on the ground is the same for the landing.
    takeoff_ = n;
    for (size_t i = 0; i < n && takeoff_ == n; ++i) if (!rows_[i].ground) takeoff_ = i;
    if (takeoff_ + 1 >= n) return fail("no mission flight in this recording (the phase column never leaves IDLE)");
    last_ = n - 1;
    for (size_t i = takeoff_ + 1; i < n; ++i) if (rows_[i].ground) { last_ = i; break; }
    landS_ = last_ + 1 < n ? rows_[last_ + 1].t - rows_[last_].t : 0.0;
    // A recording that begins in flight was either started from the air (the mission then starts from that hover) or
    // with the mission (the simulator's --record): then its first row is the take-off frame like any other, and the
    // seconds of the take-off are missing after it.
    flying_ = takeoff_ == 0 && rows_[1].t - rows_[0].t < 1.0;
    // The mission clock starts when the blocking take-off returns. The loop then fetched the newest frame and ran the
    // vision on it before the navigator saw it: in the two recordings whose log and csv can be compared to 0.01 s
    // that first step came 0.021 and 0.023 s after the take-off returned. Half a frame interval.
    airborneT_ = flying_ ? rows_[0].t : rows_[takeoff_ + 1].t - 0.5 * perFrame;
    // The tag detector runs on every Nth frame perception gets (tag.every_n_frames), and the rows with a tag show
    // which ones those were in the flight: the replay begins on the row that puts it on the same frames.
    first_ = 0;
    if (tagEveryN_ > 1) {
        for (size_t i = 0; i < n; ++i)
            if (rows_[i].tagId >= 0) { first_ = std::min((i + 1) % (size_t)tagEveryN_, takeoff_); break; }
    }
    LOG_INFO("replay: %s: %zu rows, %.2f frames/s delivered, t column trusted %.0f %% (printed to %.2g s)", csvPath_.c_str(), n, fps_,
             100.0 * trust, step);
    LOG_INFO("replay: take-off on row %zu (seq %llu), flight rows %zu-%zu (seq %llu-%llu, %.1f s), mission clock starts at %.3f; playing from row %zu",
             takeoff_, (unsigned long long)rows_[takeoff_].seq, takeoff_ + 1, last_, (unsigned long long)rows_[std::min(takeoff_ + 1, n - 1)].seq,
             (unsigned long long)rows_[last_].seq, rows_[last_].t - airborneT_, airborneT_, first_);
    return true;
}

// Frame `row` of the video (they are asked for in rising order). With replay.jpeg_shift it goes through one more
// JPEG generation on a block grid moved by that many px: padded on the top and the left, encoded, cut back.
// (replay.jpeg_quality alone: one more generation on the recorder's own grid, which changes next to nothing.)
bool FlightReplay::readFrame(size_t row, cv::Mat& img) {
    img.release();
    while (framesRead_ <= row) {
        const bool ok = framesRead_ < row ? cap_.grab() : cap_.read(img);
        if (!ok) return false;
        ++framesRead_;
    }
    if (img.empty()) return false;
    if (jpegShift_ > 0 || jpegQuality_ > 0) {
        cv::Mat big;
        std::vector<uchar> buf;
        cv::copyMakeBorder(img, big, jpegShift_, (16 - jpegShift_) % 16, jpegShift_, (16 - jpegShift_) % 16, cv::BORDER_REPLICATE);
        if (cv::imencode(".jpg", big, buf, {cv::IMWRITE_JPEG_QUALITY, jpegQuality_ > 0 ? jpegQuality_ : 90}))
            img = cv::imdecode(buf, cv::IMREAD_COLOR)(cv::Rect(jpegShift_, jpegShift_, img.cols, img.rows)).clone();
    }
    return true;
}

// The camera angle the flight had in force, row by row. The csv says where on the floor the flight saw a tag, and
// the video has the pixels it saw it in: the angle that puts the centre of the tag on that spot (with the height and
// the lean of that frame as they are known here) is the one the flight was using. So perception runs over the whole
// recording once on its own, and every frame with a tag gives one reading.
//
// A reading holds two things. The mount: it moves in steps, when the flight's own fit ran, and never between
// sightings. And how far the lean taken here is from the lean the flight took out of that frame (the telemetry comes
// in whole degrees ten times a second and is interpolated between packets whose times the csv does not hold): that
// part is nothing while the lean stands still and up to a degree while it changes. The steps are laid through the
// readings, and what a reading shows beyond its step is kept as the lean correction of that frame: with both, the
// floor is seen in every frame with a tag exactly as the flight saw it.
//
// One thing a single sighting cannot tell from the angle is the factor the height sensor is scaled with (a floor 1 %
// further away looks like a camera tilted a quarter of a degree more towards a tag 40 cm off). That factor is a
// property of the tags and the lens, so this pass fits it from the sightings as the flight did, and the replay is
// given it row by row along with the angle.
bool FlightReplay::followCamera(std::string* err) {
    Perception perception;
    const logx::Level level = logx::level();
    logx::setLevel(logx::WARN);                      // what this pass fits for itself is not the replay's
    const bool ready = perception.init(cfg_, err);
    std::vector<size_t> row;                         // rows with a reading
    std::vector<double> pitch, roll, weight;
    size_t calm = 0;
    cv::Mat img;
    scale_.assign(rows_.size(), 1.0);
    for (size_t i = first_; ready && i <= last_ && readFrame(i, img); ++i) {
        const Row& r = rows_[i];
        scale_[i] = perception.heightScale();        // the one this frame is seen with: a fit on it shows from the next
        const Observation obs = perception.process(img, r.st, r.t, r.seq);
        const TagObservation* t = i > takeoff_ && r.tagId >= 0 ? obs.tag(r.tagId) : nullptr;
        if (!t) continue;
        GroundProjector g = perception.projector();
        double p = g.cameraPitchDeg(), q = g.cameraRollDeg();
        bool ok = true;
        for (int it = 0; it < 5 && ok; ++it) {       // Newton in the two angles: the floor point moves almost evenly with them
            const double h = 0.05;
            double X0 = 0, Y0 = 0, Xp = 0, Yp = 0, Xq = 0, Yq = 0;
            g.setCameraMount(p, q);
            ok = g.pixelToGroundAny(t->center.x, t->center.y, X0, Y0);
            g.setCameraMount(p + h, q);
            ok = ok && g.pixelToGroundAny(t->center.x, t->center.y, Xp, Yp);
            g.setCameraMount(p, q + h);
            ok = ok && g.pixelToGroundAny(t->center.x, t->center.y, Xq, Yq);
            const double a = (Xp - X0) / h, b = (Xq - X0) / h, cc = (Yp - Y0) / h, d = (Yq - Y0) / h, det = a * d - b * cc;
            if (!ok || std::fabs(det) < 1e-12) { ok = false; break; }
            const double ex = r.tagX - X0, ey = r.tagY - Y0;
            p += (d * ex - b * ey) / det;
            q += (a * ey - cc * ex) / det;
        }
        if (!ok || std::fabs(p - givenPitch_) > 15 || std::fabs(q - givenRoll_) > 15) continue;   // not a reading of the mount
        // standing still: the same pitch and roll in every row of the last 0.35 s (the lean is taken from 0.1 s
        // back, between two packets 0.1 s apart)
        bool still = true;
        for (size_t j = i; j > takeoff_ && rows_[j].t >= r.t - 0.35 && still; --j)
            still = rows_[j].st.pitch == r.st.pitch && rows_[j].st.roll == r.st.roll;
        calm += still;
        row.push_back(i); pitch.push_back(p); roll.push_back(q); weight.push_back(still ? 1.0 : 1.0 / 3.0);
    }
    logx::setLevel(level);
    if (!ready) return false;
    for (size_t k = 0; k < row.size(); ++k)
        LOG_DEBUG("RPL camera t=%.2f seq=%llu pitch=%.2f roll=%.2f height x%.3f%s", rows_[row[k]].t - airborneT_, (unsigned long long)rows_[row[k]].seq,
                  pitch[k], roll[k], scale_[row[k]], weight[k] == 1.0 ? "" : " (lean changing)");
    cap_.release();
    if (!cap_.open(videoPath_)) { if (err) *err = "replay: cannot open video file " + videoPath_; return false; }
    framesRead_ = 0;
    mount_.clear();
    leanFix_.assign(rows_.size(), cv::Point2d(0, 0));
    if (row.empty()) {
        LOG_INFO("replay: the flight saw no tag: the camera angle stays the configured one (%.2f / %.2f deg)", givenPitch_, givenRoll_);
        return true;
    }
    const std::vector<size_t> starts = staircase(pitch, roll, weight);
    for (size_t k = 0; k < starts.size(); ++k) {
        const size_t a = starts[k], b = k + 1 < starts.size() ? starts[k + 1] : row.size();
        // the step is its calm readings where it has any (they are exact), all of its readings otherwise
        std::vector<double> vp, vr;
        for (int pass = 0; pass < 2 && vp.empty(); ++pass)
            for (size_t j = a; j < b; ++j)
                if (pass || weight[j] == 1.0) { vp.push_back(pitch[j]); vr.push_back(roll[j]); }
        // the fit that moved the angle ran on the last sighting of the step before: the next frame had the new one
        mount_.push_back({k == 0 ? first_ : row[a - 1] + 1, mu::median(vp), mu::median(vr)});
    }
    std::vector<char> has(rows_.size(), 0);
    for (size_t k = 0, m = 0; k < row.size(); ++k) {
        while (m + 1 < mount_.size() && mount_[m + 1].row <= row[k]) ++m;
        leanFix_[row[k]] = cv::Point2d(pitch[k] - mount_[m].pitch, roll[k] - mount_[m].roll);
        has[row[k]] = 1;
    }
    // the tag detector runs on every Nth frame: a frame between two sightings takes the mean of theirs
    for (size_t i = takeoff_ + 2; i < last_; ++i)
        if (!has[i] && (has[i - 1] || has[i + 1]))
            leanFix_[i] = (leanFix_[i - 1] * (double)has[i - 1] + leanFix_[i + 1] * (double)has[i + 1]) * (1.0 / (has[i - 1] + has[i + 1]));
    std::string steps;
    for (size_t k = 1; k < mount_.size() && k <= 8; ++k) {
        char b[64];
        snprintf(b, sizeof(b), ", %.1f / %.1f from t=%.2f", mount_[k].pitch, mount_[k].roll, rows_[mount_[k].row].t - airborneT_);
        steps += b;
    }
    if (mount_.size() > 9) steps += ", ...";
    LOG_INFO("replay: camera angle of the flight, read off the %zu frames in which it saw a tag (%zu of them with the lean standing still): "
             "pitch %.2f roll %.2f deg at take-off (configured here: %.2f / %.2f)%s; height sensor x%.3f at the end", row.size(), calm,
             mount_[0].pitch, mount_[0].roll, givenPitch_, givenRoll_, steps.c_str(), scale_[last_]);
    return true;
}

void FlightReplay::applyTo(Config& cfg) const {
    if (mount_.empty()) return;
    char b[32];
    snprintf(b, sizeof(b), "%.4f", mount_[0].pitch);
    cfg.set("camera.pitch_deg", b);
    snprintf(b, sizeof(b), "%.4f", mount_[0].roll);
    cfg.set("camera.roll_deg", b);
    cfg.set("ground.auto_calib", "0");
}

void FlightReplay::aim(Perception& perception) {
    if (mount_.empty()) return;
    GroundProjector& g = perception.projector();
    g.setCameraMount(mount_[mountNext_ - 1].pitch, mount_[mountNext_ - 1].roll);   // without the lean correction of the frame before
    // a step of the flight's own fit: the navigator is told how far the floor moved, as it was told then
    for (; mountNext_ < mount_.size() && mount_[mountNext_].row <= cur_; ++mountNext_)
        perception.setMount(mount_[mountNext_].pitch, mount_[mountNext_].roll);
    const Mount& m = mount_[mountNext_ - 1];
    g.setCameraMount(m.pitch + leanFix_[cur_].x, m.roll + leanFix_[cur_].y);
    perception.setHeightScale(scale_[cur_]);
}

bool FlightReplay::start(std::string* err) {
    if (!loadCsv(err)) return false;
    if (!cap_.open(videoPath_)) { if (err) *err = "replay: cannot open video file " + videoPath_; return false; }
    const double frames = cap_.get(cv::CAP_PROP_FRAME_COUNT);
    if (frames > 0 && std::fabs(frames - (double)rows_.size()) > 1.5)
        LOG_WARN("replay: the video has %.0f frames and the csv %zu rows: row i is taken to be frame i", frames, rows_.size());
    framesRead_ = 0;
    if (!follow_)
        LOG_INFO("replay: camera angle as configured (%.2f / %.2f deg), measured again here from the sightings as on a station just started", givenPitch_, givenRoll_);
    else if (!followCamera(err)) return false;
    next_ = cur_ = telem_ = first_;
    now_ = rows_[first_].t;
    started_ = ended_ = delivered_ = false;
    complete_ = true;
    mountNext_ = 1;
    paceWall_ = wallNow();
    paceRowT_ = now_;
    return true;
}

void FlightReplay::stop() {
    if (cap_.isOpened()) cap_.release();
}

bool FlightReplay::startMissionNow() {
    if (started_ || !delivered_ || cur_ != takeoff_) return false;
    started_ = true;
    return true;
}

bool FlightReplay::command(const std::string& cmd, std::string* reply, double) {
    if (reply) *reply = cmd == "battery?" ? std::to_string(state().bat) : "ok";
    if (cmd == "takeoff") {
        if (!flying_ && cur_ == takeoff_) {
            // the take-off of the flight: the clock and the telemetry go to where they were when it returned
            now_ = std::max(now_, airborneT_);
            telem_ = takeoff_ + 1;
        }
        flying_ = true;
    } else if (cmd == "land" || cmd == "emergency") {
        const bool was = flying_;
        flying_ = false;
        if (was && !ended_) {
            now_ += landS_;                           // the landing took this long in the flight
            finish(cmd == "land" ? "the navigator landed" : "emergency stop");
        }
    }
    return true;
}

// The sticks the navigator sent on the frame just played, against the ones the flight sent on it.
void FlightReplay::compareSticks() {
    if (!delivered_ || !flying_ || cur_ <= takeoff_ || cur_ > last_) return;
    const Row& r = rows_[cur_];
    bool apart = false;
    for (int k = 0; k < 4; ++k) {
        const double d = sent_[k] - r.rc[k];
        diffSq_[k] += d * d;
        if (k != 2 && std::fabs(d) > 25) apart = true;   // height is the same control law on the same telemetry: it never differs
    }
    ++diffN_;
    LOG_DEBUG("RPL t=%.2f seq=%llu sent=(%d,%d,%d,%d) flown=(%d,%d,%d,%d)", now_ - airborneT_, (unsigned long long)r.seq, sent_[0], sent_[1],
              sent_[2], sent_[3], r.rc[0], r.rc[1], r.rc[2], r.rc[3]);
    // apart for half a second on end: a different decision, not the frame-to-frame scatter of two runs of one
    if (!apart) apartRun_ = 0;
    else if (++apartRun_ >= 12 && apartT_ < 0) apartT_ = rows_[cur_ - 11].t - airborneT_;
}

// The tag in the frame just played against where the flight saw it in the same frame. The pixels are the same, so a
// steady difference is the camera angle: the flight had another one in force than the replay has.
void FlightReplay::observed(const Observation& obs) {
    if (!delivered_ || cur_ <= takeoff_ || cur_ > last_) return;
    const Row& r = rows_[cur_];
    if (r.paths >= 0) {      // as many pieces of tape as the flight traced in this frame? (one more or fewer: the pixels)
        ++pathsN_;
        if ((int)obs.paths.size() == r.paths) ++pathsSame_;
        else {
            if (pathsFirstT_ < 0) pathsFirstT_ = now_ - airborneT_;
            LOG_DEBUG("RPL t=%.2f seq=%llu %zu pieces of tape here, %d in the flight", now_ - airborneT_, (unsigned long long)r.seq, obs.paths.size(), r.paths);
        }
    }
    const TagObservation* t = r.tagId >= 0 ? obs.tag(r.tagId) : nullptr;
    if (r.tagId >= 0 && !t) ++tagOnlyFlight_;
    if (r.tagId < 0 && !obs.tags.empty()) ++tagOnlyHere_;
    if (!t) return;
    const double dx = t->X - r.tagX, dy = t->Y - r.tagY;
    if (++tagN_ == 1) { tagFirst_[0] = dx; tagFirst_[1] = dy; tagFirst_[2] = obs.heightM; }
    tagSum_[0] += dx; tagSum_[1] += dy;
    tagSq_ += dx * dx + dy * dy;
    tagHeight_ += obs.heightM;
    LOG_DEBUG("RPL t=%.2f seq=%llu tag ID%d here=(%.3f,%.3f) flown=(%.3f,%.3f)", now_ - airborneT_, (unsigned long long)r.seq, r.tagId, t->X, t->Y,
              r.tagX, r.tagY);
}

void FlightReplay::finish(const char* why) {
    if (ended_) return;
    ended_ = true;
    const double n = std::max(1, diffN_);
    LOG_INFO("replay: %s at mission time %.2f s (recorded flight: %.2f s)", why, now_ - airborneT_, rows_[last_].t + landS_ - airborneT_);
    char apart[96] = "never apart by more than 25 for 0.5 s";
    if (apartT_ >= 0) snprintf(apart, sizeof(apart), "apart by more than 25 for 0.5 s from t=%.2f", apartT_);
    LOG_INFO("replay: sticks against the flight's over %d frames: rms difference lr %.1f fb %.1f ud %.1f yaw %.1f; %s", diffN_,
             std::sqrt(diffSq_[0] / n), std::sqrt(diffSq_[1] / n), std::sqrt(diffSq_[2] / n), std::sqrt(diffSq_[3] / n), apart);
    if (pathsN_ > 0) {
        char first[64] = "";
        if (pathsFirstT_ >= 0) snprintf(first, sizeof(first), "; another number first at t=%.2f", pathsFirstT_);
        LOG_INFO("replay: pieces of tape traced: as many as in the flight on %d of %d frames%s", pathsSame_, pathsN_, first);
    }
    if (tagN_ + tagOnlyFlight_ + tagOnlyHere_ == 0) return;
    const double m = std::max(1, tagN_), mx = tagSum_[0] / m, my = tagSum_[1] / m, h = tagHeight_ / m;
    LOG_INFO("replay: tags against where the flight saw them, on %d frames: mean (%+.3f, %+.3f) m, rms %.3f m; seen only in the flight on %d, only here on %d",
             tagN_, mx, my, std::sqrt(tagSq_ / m), tagOnlyFlight_, tagOnlyHere_);
    // Without the angle taken from the flight (replay.follow_camera=0): 2 cm at flying height is a degree of camera
    // angle, more than the frame-to-frame scatter of the tag centre. A point of the floor moves back as the camera
    // is tilted further down and left as it is rolled.
    const double far = std::max(std::hypot(mx, my), std::hypot(tagFirst_[0], tagFirst_[1]));
    if (mount_.empty() && tagN_ >= 5 && far > 0.02 && h > 0.2 && tagFirst_[2] > 0.2)
        LOG_WARN("replay: the tags lie %.0f cm from where the flight saw them: the flight had another camera angle in force than the one "
                 "configured here (about camera.pitch_deg=%.1f camera.roll_deg=%.1f at its first sighting, %.1f / %.1f on average). "
                 "With replay.follow_camera=1 the angle is taken from the flight",
                 100 * far, givenPitch_ + mu::rad2deg(std::atan2(tagFirst_[0], tagFirst_[2])), givenRoll_ - mu::rad2deg(std::atan2(tagFirst_[1], tagFirst_[2])),
                 givenPitch_ + mu::rad2deg(std::atan2(mx, h)), givenRoll_ - mu::rad2deg(std::atan2(my, h)));
}

bool FlightReplay::latest(Frame& out, uint64_t, double) {
    if (ended_) return false;
    compareSticks();
    if (next_ > last_) { finish(flying_ ? "the recorded flight ends with the navigator still flying" : "the recorded flight ends"); return false; }
    size_t i = next_;
    while (i < last_ && rows_[i + 1].t <= now_) ++i;   // the clock was moved on (a blocking command): those frames were never looked at
    cv::Mat img;
    if (!readFrame(i, img)) { complete_ = false; finish("the video ends before the csv"); return false; }
    if (realtime_) {
        // At the pace of the flight (the seconds a blocking command took are not waited for again). Every frame is
        // due one interval after the last one WAS DUE, not after it came: a sleep always runs a little long, and
        // counted from the late wake-ups 24 of them a second add up to a third more time.
        double due = paceWall_ + mu::clamp(rows_[i].t - paceRowT_, 0.0, 0.25);
        const double w = wallNow();
        if (w < due) std::this_thread::sleep_for(std::chrono::duration<double>(due - w));
        else if (w - due > 0.25) due = w;              // far behind (a slow machine): no catching up in a burst
        paceWall_ = due;
        paceRowT_ = rows_[i].t;
    }
    cur_ = telem_ = i;
    next_ = i + 1;
    delivered_ = true;
    now_ = std::max(now_, rows_[i].t);
    out.image = img;
    out.seq = rows_[i].seq;
    out.stamp = rows_[i].t;
    return true;
}
