#include "sim/FlightReplay.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <thread>

#include "util/Log.h"
#include "util/MathUtil.h"

namespace {
double wallNow() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

// One unit of the last digit the recorder printed: it writes t with six significant digits.
double printedStep(double t) { return t > 0 ? std::pow(10.0, std::floor(std::log10(t)) - 5.0) : 1e-6; }
}  // namespace

FlightReplay::FlightReplay(const Config& cfg, const std::string& videoPath, const std::string& csvPath, bool realtime)
    : videoPath_(videoPath), csvPath_(csvPath), realtime_(realtime), tagEveryN_(std::max(1, cfg.getInt("tag.every_n_frames", 1))) {}

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
              cPitch = col("pitch"), cRoll = col("roll"), cTag = col("tag_id"), cRc = col("rc_lr");
    if (cT < 0 || cSeq < 0 || cPhase < 0 || cBat < 0 || cTof < 0 || cH < 0 || cYaw < 0 || cPitch < 0 || cRoll < 0 || cRc < 0 ||
        cRc + 3 >= (int)head.size())
        return fail("not a csv of the ground station's RECORD (columns t, seq, phase, bat, tof, h, yaw, pitch, roll, rc_lr.. expected)");
    rows_.clear();
    std::vector<double> stamp;                       // the t column as printed
    while (std::getline(f, line)) {
        const std::vector<std::string> v = splitString(trim(line), ',');
        if (v.size() < head.size()) continue;        // a last line cut short when the recording was stopped
        Row r;
        stamp.push_back(atof(v[cT].c_str()));
        r.seq = strtoull(v[cSeq].c_str(), nullptr, 10);
        const std::string ph = trim(v[cPhase]);
        r.ground = ph == "IDLE" || ph == "LANDED" || ph == "ABORTED";
        r.tag = cTag >= 0 && atoi(v[cTag].c_str()) >= 0;
        r.st.valid = true;
        r.st.bat = atoi(v[cBat].c_str()); r.st.tof = atoi(v[cTof].c_str()); r.st.h = atoi(v[cH].c_str());
        r.st.yaw = atoi(v[cYaw].c_str()); r.st.pitch = atoi(v[cPitch].c_str()); r.st.roll = atoi(v[cRoll].c_str());
        for (int k = 0; k < 4; ++k) r.rc[k] = atoi(v[cRc + k].c_str());
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
    // the packets apart by their time: a row where a value changed is a new packet for certain; while nothing
    // changes one is assumed every 0.1 s.
    double packetT = rows_[0].t, due = packetT + 0.1;
    for (size_t i = 0; i < n; ++i) {
        Row& r = rows_[i];
        const TelloState& a = r.st;
        const TelloState& b = rows_[i ? i - 1 : 0].st;
        const bool changed = a.bat != b.bat || a.tof != b.tof || a.h != b.h || a.yaw != b.yaw || a.pitch != b.pitch || a.roll != b.roll;
        if (changed) { packetT = r.t; due = packetT + 0.1; }
        else if (r.t >= due) { packetT = r.t; due = std::max(due + 0.1, r.t + 0.05); }
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
    flying_ = takeoff_ == 0;                          // recorded from the air: the mission starts from that hover
    // The mission clock starts when the blocking take-off returns. The loop then fetched the newest frame and ran the
    // vision on it before the navigator saw it: in the two recordings whose log and csv can be compared to 0.01 s
    // that first step came 0.021 and 0.023 s after the take-off returned. Half a frame interval.
    airborneT_ = takeoff_ == 0 ? rows_[0].t : rows_[takeoff_ + 1].t - 0.5 * perFrame;
    // The tag detector runs on every Nth frame perception gets (tag.every_n_frames), and the rows with a tag show
    // which ones those were in the flight: the replay begins on the row that puts it on the same frames.
    first_ = 0;
    if (tagEveryN_ > 1) {
        for (size_t i = 0; i < n; ++i)
            if (rows_[i].tag) { first_ = std::min((i + 1) % (size_t)tagEveryN_, takeoff_); break; }
    }
    LOG_INFO("replay: %s: %zu rows, %.2f frames/s delivered, t column trusted %.0f %% (printed to %.2g s)", csvPath_.c_str(), n, fps_,
             100.0 * trust, step);
    LOG_INFO("replay: take-off on row %zu (seq %llu), flight rows %zu-%zu (seq %llu-%llu, %.1f s), mission clock starts at %.3f; playing from row %zu",
             takeoff_, (unsigned long long)rows_[takeoff_].seq, takeoff_ + 1, last_, (unsigned long long)rows_[std::min(takeoff_ + 1, n - 1)].seq,
             (unsigned long long)rows_[last_].seq, rows_[last_].t - airborneT_, airborneT_, first_);
    return true;
}

bool FlightReplay::start(std::string* err) {
    if (!loadCsv(err)) return false;
    if (!cap_.open(videoPath_)) { if (err) *err = "replay: cannot open video file " + videoPath_; return false; }
    const double frames = cap_.get(cv::CAP_PROP_FRAME_COUNT);
    if (frames > 0 && std::fabs(frames - (double)rows_.size()) > 1.5)
        LOG_WARN("replay: the video has %.0f frames and the csv %zu rows: row i is taken to be frame i", frames, rows_.size());
    next_ = cur_ = telem_ = first_;
    framesRead_ = 0;
    now_ = rows_[first_].t;
    started_ = ended_ = delivered_ = false;
    lastWall_ = wallNow();
    lastRowT_ = now_;
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

void FlightReplay::finish(const char* why) {
    if (ended_) return;
    ended_ = true;
    const double n = std::max(1, diffN_);
    LOG_INFO("replay: %s at mission time %.2f s (recorded flight: %.2f s)", why, now_ - airborneT_, rows_[last_].t + landS_ - airborneT_);
    char apart[96] = "never apart by more than 25 for 0.5 s";
    if (apartT_ >= 0) snprintf(apart, sizeof(apart), "apart by more than 25 for 0.5 s from t=%.2f", apartT_);
    LOG_INFO("replay: sticks against the flight's over %d frames: rms difference lr %.1f fb %.1f ud %.1f yaw %.1f; %s", diffN_,
             std::sqrt(diffSq_[0] / n), std::sqrt(diffSq_[1] / n), std::sqrt(diffSq_[2] / n), std::sqrt(diffSq_[3] / n), apart);
}

bool FlightReplay::latest(Frame& out, uint64_t, double) {
    if (ended_) return false;
    compareSticks();
    if (next_ > last_) { finish(flying_ ? "the recorded flight ends with the navigator still flying" : "the recorded flight ends"); return false; }
    size_t i = next_;
    while (i < last_ && rows_[i + 1].t <= now_) ++i;   // the clock was moved on (a blocking command): those frames were never looked at
    cv::Mat img;
    while (framesRead_ <= i) {
        const bool ok = framesRead_ < i ? cap_.grab() : cap_.read(img);
        if (!ok) { finish("the video ends before the csv"); return false; }
        ++framesRead_;
    }
    if (img.empty()) { finish("the video ends before the csv"); return false; }
    if (realtime_) {   // at the pace of the flight; the seconds a blocking command took are not waited for again
        const double due = lastWall_ + mu::clamp(rows_[i].t - lastRowT_, 0.0, 0.25);
        if (wallNow() < due) std::this_thread::sleep_for(std::chrono::duration<double>(due - wallNow()));
        lastWall_ = wallNow();
        lastRowT_ = rows_[i].t;
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
