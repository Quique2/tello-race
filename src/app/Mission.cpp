#include "app/Mission.h"
#include "app/WebServer.h"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "control/Navigator.h"
#include "sim/Simulator.h"
#include "tello/TelloDrone.h"
#include "util/Log.h"
#include "vision/Perception.h"

namespace {

double wallNow() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

std::string timestamp() {
    std::time_t t = std::time(nullptr);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&t));
    return buf;
}

class MissionLoop {
public:
    MissionLoop(IDrone& drone, IFrameSource& src, IClock& clock, const Config& cfg, const MissionOptions& opt, Simulator* sim, WebServer* web)
        : drone_(drone), src_(src), clock_(clock), cfg_(cfg), opt_(opt), sim_(sim), nav_(drone, clock, cfg), web_(web) {}
    int run();

private:
    void drawHud(cv::Mat& canvas, const Observation& obs, const TelloState& st, double fps, bool haveObs);
    bool handleKey(int key);
    void startRecording();
    void stopRecording();
    void record(const cv::Mat& frame, const Observation& obs, const TelloState& st);
    bool restartVideo(const char* why);   // re-request the Tello stream and re-open the decoder
    std::string stateJson(const Observation& obs, const TelloState& st, double fps);
    void applyWebCommand(const WebCommand& c);
    void setManual(bool on);

    IDrone& drone_;
    IFrameSource& src_;
    IClock& clock_;
    const Config& cfg_;
    MissionOptions opt_;
    Simulator* sim_;
    Perception perception_;
    Navigator nav_;
    cv::VideoWriter writer_;
    std::ofstream csv_;
    bool recording_ = false, showMask_ = true, showDebug_ = true, quitting_ = false;
    uint64_t seq_ = 0;
    int startAtTag_ = 0;
    int videoRecoveries_ = 0;
    std::string recBase_;
    WebServer* web_ = nullptr;   // owned by runFly/runSim so it outlives simulator episodes
    bool webOn_ = false;
    bool manual_ = false;                 // manual mode: the browser drives the sticks, the navigator is idle
    int manualRc_[4] = {0, 0, 0, 0};
    double manualRcT_ = 0;
};

int MissionLoop::run() {
    std::string err;
    if (!perception_.init(cfg_, &err)) { LOG_ERROR("perception init failed: %s", err.c_str()); return 1; }
    startAtTag_ = opt_.startAtTag;
    const std::string win = "tello_race";
    if (!opt_.headless) cv::namedWindow(win, cv::WINDOW_AUTOSIZE);
    if (opt_.record) startRecording();
    webOn_ = web_ && cfg_.getBool("web.enabled", false);
    if (webOn_ && !web_->running()) {
        WebServer::Options wo;
        wo.port = cfg_.getInt("web.port", 8080);
        wo.token = cfg_.getString("web.token", "");
        wo.viewerToken = cfg_.getString("web.viewer_token", "");
        wo.fps = cfg_.getInt("web.fps", 15);
        wo.jpegQuality = cfg_.getInt("web.jpeg_quality", 70);
        if (!web_->start(wo, &err)) { LOG_ERROR("%s", err.c_str()); webOn_ = false; }
    }
    if (opt_.autoStart) nav_.start(startAtTag_);
    else LOG_INFO("Ready. Press 't' in the video window to take off and start (l/space = land, e = emergency, q = quit).");

    SteadyClock wall;
    Frame frame;
    seq_ = 0;
    if (auto* td = dynamic_cast<TelloDrone*>(&drone_)) perception_.setDownwardCamera(td->downvision());
    double lastFrameWall = wall.now(), fpsT = wall.now(), lastLoop = wall.now(), fps = 0, lastKeepAlive = wall.now();
    int fpsN = 0;
    bool haveFrame = false;
    double lastStateT = 0;
    while (true) {
        if (quitting_ && !nav_.flying()) break;
        WebCommand wc;
        while (webOn_ && web_->popCommand(wc)) applyWebCommand(wc);
        bool got = src_.latest(frame, seq_, 0.25);
        if (got) { seq_ = frame.seq; haveFrame = true; lastFrameWall = wall.now(); videoRecoveries_ = 0; }
        else if (src_.ended()) {
            // A live Tello stream stops on a camera switch or a Wi-Fi hiccup. Ask for it again and
            // re-open the decoder so the ground-station page survives; only give up after that fails.
            bool recoverable = !sim_ && src_.isLive() && !quitting_ && !nav_.flying() && !drone_.isFlying();
            if (!recoverable || ++videoRecoveries_ > 5 || !restartVideo("stream stopped")) {
                LOG_INFO("end of video source");
                break;
            }
        }
        TelloState st = drone_.state();
        Observation obs;
        double hx = 0, hy = 0;
        perception_.setLineHint(nav_.lineHint(hx, hy), hx, hy);
        if (got) obs = perception_.process(frame.image, st, frame.stamp, frame.seq);
        else { obs.stamp = clock_.now(); obs.blindZoneM = perception_.projector().blindZoneX(); }
        if (!sim_ && !nav_.flying() && wall.now() - lastKeepAlive > 8.0) {   // keep SDK mode alive while idle on the ground
            lastKeepAlive = wall.now();
            drone_.queryInt("battery?");
        }
        if (!sim_ && nav_.flying() && wall.now() - lastFrameWall > 2.5) {
            LOG_ERROR("no video for 2.5 s while flying -> landing");
            nav_.requestLand();
        }
        if (manual_) {
            int r0 = manualRc_[0], r1 = manualRc_[1], r2 = manualRc_[2], r3 = manualRc_[3];
            if (wall.now() - manualRcT_ > 0.5) r0 = r1 = r2 = r3 = 0;   // dead-man: no fresh sticks -> hover
            if (drone_.isFlying()) drone_.rc(r0, r1, r2, r3);
        } else if (got || !sim_) {
            nav_.step(obs, st);
        }
        if (recording_ && got) record(frame.image, obs, st);
        if (got && ++fpsN && wall.now() - fpsT > 1.0) { fps = fpsN / (wall.now() - fpsT); fpsN = 0; fpsT = wall.now(); }

        if (webOn_ && wall.now() - lastStateT > 0.1) { web_->publishState(stateJson(obs, st, fps)); lastStateT = wall.now(); }
        bool dump = opt_.dumpEvery > 0 && got && (seq_ % opt_.dumpEvery == 0);
        bool stream = webOn_ && got && web_->streamClients() > 0;
        if (!opt_.headless || dump || stream) {
            cv::Mat canvas = haveFrame ? frame.image.clone() : cv::Mat(720, 960, CV_8UC3, cv::Scalar(30, 30, 30));
            if (showDebug_ && got) perception_.drawDebug(canvas, obs);
            drawHud(canvas, obs, st, fps, got);
            if (sim_) { cv::Mat top = sim_->topView(canvas.rows); cv::hconcat(canvas, top, canvas); }
            if (dump) {
                std::filesystem::create_directories(opt_.dumpDir);
                char name[64];
                snprintf(name, sizeof(name), "/frame_%06llu.png", (unsigned long long)seq_);
                cv::imwrite(opt_.dumpDir + name, canvas);
            }
            if (stream) web_->publishFrame(canvas);
            if (!opt_.headless) {
                if (std::fabs(opt_.uiScale - 1.0) > 1e-3) cv::resize(canvas, canvas, cv::Size(), opt_.uiScale, opt_.uiScale);
                cv::imshow(win, canvas);
                int key = cv::waitKey(1);
                if (key >= 0 && !handleKey(key & 0xff)) break;
            }
        }
        if (sim_ && opt_.realtime && (!opt_.headless || webOn_)) {   // real-time pacing for a window or web viewers
            double target = 1.0 / std::max(1.0, src_.fps());
            double dtw = wall.now() - lastLoop;
            if (dtw < target) wall.sleep(target - dtw);
            lastLoop = wall.now();
        }
        if (sim_ && (sim_->done() || (nav_.finished() && !sim_->isFlying()))) {
            if (!opt_.headless) { cv::waitKey(500); }
            break;
        }
        if (sim_ && clock_.now() > 600) { LOG_WARN("simulation time limit"); break; }
    }
    if (nav_.flying()) {
        LOG_WARN("exiting while flying -> landing");
        nav_.requestLand();
        Observation o;
        nav_.step(o, drone_.state());
    }
    stopRecording();
    if (!opt_.headless) cv::destroyAllWindows();
    return 0;
}

void MissionLoop::applyWebCommand(const WebCommand& c) {
    if (c.cmd != "rc") LOG_INFO("web command: %s %s", c.cmd.c_str(), c.arg.c_str());
    if (c.cmd == "mode") setManual(c.arg == "manual");
    else if (c.cmd == "rc") {
        int a = 0, b = 0, d = 0, e = 0;
        if (manual_ && sscanf(c.arg.c_str(), "%d,%d,%d,%d", &a, &b, &d, &e) == 4) {
            manualRc_[0] = std::max(-100, std::min(100, a)); manualRc_[1] = std::max(-100, std::min(100, b));
            manualRc_[2] = std::max(-100, std::min(100, d)); manualRc_[3] = std::max(-100, std::min(100, e));
            manualRcT_ = wallNow();
        }
    }
    else if (c.cmd == "takeoff") { if (manual_ && !drone_.isFlying()) { if (!drone_.takeoff()) LOG_WARN("manual takeoff refused: %s", drone_.lastReply.c_str()); } }
    else if (c.cmd == "start") { if (!manual_) handleKey('t'); else LOG_WARN("START ignored in manual mode: switch to mission mode first"); }
    else if (c.cmd == "land") { if (manual_) { manualRc_[0] = manualRc_[1] = manualRc_[2] = manualRc_[3] = 0; drone_.land(); } else handleKey('l'); }
    else if (c.cmd == "emergency") handleKey('e');
    else if (c.cmd == "record") handleKey('r');
    else if (c.cmd == "camera") handleKey('v');
    else if (c.cmd == "tag") handleKey('0' + std::max(0, std::min(3, atoi(c.arg.c_str()))));
}

bool MissionLoop::restartVideo(const char* why) {
    for (int attempt = 1; attempt <= 3; ++attempt) {
        LOG_WARN("video: re-requesting the stream (%s, attempt %d/3)", why, attempt);
        src_.stop();
        std::string r;
        drone_.command("streamoff", &r, 3.0);
        clock_.sleep(0.5);
        drone_.streamOn();
        clock_.sleep(0.5);
        std::string e;
        if (src_.start(&e)) { seq_ = 0; LOG_INFO("video: stream re-opened"); return true; }
        LOG_ERROR("video: re-open failed: %s", e.c_str());
        clock_.sleep(1.0);
    }
    return false;
}

void MissionLoop::setManual(bool on) {
    if (on == manual_) return;
    manual_ = on;
    manualRc_[0] = manualRc_[1] = manualRc_[2] = manualRc_[3] = 0;
    manualRcT_ = 0;
    if (on) { nav_.release(); LOG_INFO("MANUAL mode: the browser drives the sticks (dead-man 0.5 s)"); }
    else { if (drone_.isFlying()) drone_.rc(0, 0, 0, 0); LOG_INFO("MISSION mode: press START to run the mission%s", drone_.isFlying() ? " from the current hover" : ""); }
}

std::string MissionLoop::stateJson(const Observation& obs, const TelloState& st, double fps) {
    StateSnapshot s;
    s.manual = manual_; s.flying = drone_.isFlying();
    s.phase = nav_.phaseName(); s.seg = nav_.segmentsDone(); s.t = nav_.missionTime();
    s.telemetry = st.valid; s.bat = st.bat; s.tof = st.tof; s.h = st.h; s.yaw = st.yaw; s.pitch = st.pitch; s.roll = st.roll;
    s.templ = st.templ; s.temph = st.temph; s.fps = fps;
    s.belly = perception_.downwardCamera(); s.recording = recording_;
    std::string dn = drone_.name();
    s.drone = dn.c_str();
    if (!obs.tags.empty()) { s.tagId = obs.tags.front().id; s.tagX = obs.tags.front().X; }
    if (manual_) {   // show the operator the sticks actually applied (zeros once the dead-man has fired)
        bool stale = wallNow() - manualRcT_ > 0.5;
        for (int i = 0; i < 4; ++i) s.rc[i] = stale ? 0 : manualRc_[i];
    } else {
        RcCommand rc = nav_.lastRc();
        s.rc[0] = rc.lr; s.rc[1] = rc.fb; s.rc[2] = rc.ud; s.rc[3] = rc.yaw;
    }
    s.events = nav_.events;
    return stateToJson(s);
}

bool MissionLoop::handleKey(int key) {
    switch (key) {
        case 't': {
            if (nav_.flying()) return true;
            if (!sim_ && !opt_.dryRun) {   // preflight: telemetry and video must be alive before the motors start
                TelloState st = drone_.state();
                if (!st.valid) { LOG_ERROR("preflight: no telemetry from the drone (UDP 8890 blocked?) - not starting"); return true; }
                if (src_.isLive() && src_.fps() < 10) { LOG_ERROR("preflight: video %.1f fps (< 10) - not starting", src_.fps()); return true; }
                if (st.bat >= 0 && st.bat < nav_.params().minBattery) { LOG_ERROR("preflight: battery %d%% too low", st.bat); return true; }
                LOG_INFO("preflight OK: battery %d%%, video %.1f fps, height %d cm", st.bat, src_.fps(), st.tof);
            }
            nav_.start(startAtTag_);
            return true;
        }
        case '0': case '1': case '2': case '3':
            startAtTag_ = key - '0';
            LOG_INFO("start-at-tag = %d (%s)", startAtTag_, startAtTag_ == 0 ? "start line" : "resume from checkpoint tag");
            return true;
        case 'l': case ' ': nav_.requestLand(); return true;
        case 'e': nav_.emergencyStop(); return true;
        case 'r': if (recording_) stopRecording(); else startRecording(); return true;
        case 'm': showMask_ = !showMask_; return true;
        case 'v': {   // toggle front / belly camera (SDK 3.0 "downvision"); the stream is reopened
            auto* td = dynamic_cast<TelloDrone*>(&drone_);
            if (!td) { LOG_INFO("camera switch is only available on the real drone"); return true; }
            if (nav_.flying() || drone_.isFlying()) { LOG_WARN("not switching cameras while flying"); return true; }
            bool on = !td->downvision();
            src_.stop();
            std::string r;
            drone_.command("streamoff", &r, 3.0);   // the belly camera streams 320x240 grey, the front 960x720:
            if (td->setDownvision(on)) {            // the decoder must be torn down before the format changes
                perception_.setDownwardCamera(on);
                LOG_INFO("video: %s camera", on ? "downward (belly)" : "front");
            }
            videoRecoveries_ = 0;
            if (!restartVideo(on ? "belly camera" : "front camera")) LOG_ERROR("video did not come back after the camera switch");
            return true;
        }
        case 'd': showDebug_ = !showDebug_; return true;
        case 'q': case 27:
            if (nav_.flying()) { LOG_WARN("quit requested while flying -> landing first"); nav_.requestLand(); quitting_ = true; return true; }
            return false;
        default: return true;
    }
}

void MissionLoop::drawHud(cv::Mat& canvas, const Observation& obs, const TelloState& st, double fps, bool haveObs) {
    double s = (double)canvas.cols / perception_.projector().K().width;
    cv::Mat overlay = canvas.clone();
    cv::rectangle(overlay, cv::Rect(0, 0, canvas.cols, 118), cv::Scalar(0, 0, 0), -1);
    cv::addWeighted(overlay, 0.55, canvas, 0.45, 0, canvas);
    char buf[256];
    auto put = [&](int line, const char* text, cv::Scalar col = cv::Scalar(255, 255, 255), double sc = 0.55) {
        cv::putText(canvas, text, cv::Point(8, 20 + line * 20), cv::FONT_HERSHEY_SIMPLEX, sc, col, 1, cv::LINE_AA);
    };
    put(0, ((manual_ ? std::string("MANUAL CONTROL   ") : std::string()) + nav_.status()).c_str(), manual_ ? cv::Scalar(0, 165, 255) : cv::Scalar(0, 255, 255), 0.6);
    if (st.valid) {
        snprintf(buf, sizeof(buf), "bat=%d%%  tof=%dcm h=%dcm  yaw=%d pitch=%d roll=%d  v=(%d,%d,%d)  temp=%d-%dC", st.bat, st.tof,
                 st.h, st.yaw, st.pitch, st.roll, st.vgx, st.vgy, st.vgz, st.templ, st.temph);
        put(1, buf, st.bat >= 0 && st.bat < 30 ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 255, 255));
    } else {
        put(1, "no telemetry (state stream) - using fixed altitude for the ground model", cv::Scalar(0, 165, 255));
    }
    const auto& cal = perception_.calib();
    snprintf(buf, sizeof(buf), "video %.1f fps  tag %.1fms line %.1fms  H=%.2fm pitch=%.1f blind=%.2fm  calib n=%d pitch=%.1f roll=%.1f ratio=%.2f%s",
             fps, obs.tagMs, obs.lineMs, obs.heightM, obs.totalPitchDeg, obs.blindZoneM, cal.samples, cal.pitchDeg, cal.rollDeg,
             cal.heightRatio, cal.applied ? " [applied]" : "");
    put(2, buf, cv::Scalar(200, 200, 200), 0.45);
    snprintf(buf, sizeof(buf), "keys: t=start  l/space=land  e=EMERGENCY  r=record%s  v=camera%s  0-3=resume tag(%d)  m=mask d=debug q=quit",
             recording_ ? "[REC]" : "", perception_.downwardCamera() ? "[BELLY]" : "[FRONT]", startAtTag_);
    put(3, buf, recording_ ? cv::Scalar(0, 0, 255) : cv::Scalar(180, 180, 180), 0.45);
    int line = 4;
    for (size_t i = nav_.events.size() > 5 ? nav_.events.size() - 5 : 0; i < nav_.events.size(); ++i)
        put(line++, nav_.events[i].c_str(), cv::Scalar(120, 255, 120), 0.42);
    if (haveObs && nav_.haveTarget) {
        double u, v;
        if (perception_.projector().groundToPixel(nav_.targetX, nav_.targetY, u, v))
            cv::circle(canvas, cv::Point((int)(u * s), (int)(v * s)), 9, cv::Scalar(0, 0, 255), 2);
    }
    if (showMask_ && haveObs) {
        const cv::Mat& m = perception_.lineDetector().mask();
        if (!m.empty()) {
            cv::Mat small, smallC;
            cv::resize(m, small, cv::Size(canvas.cols / 4, canvas.rows / 4), 0, 0, cv::INTER_NEAREST);
            cv::cvtColor(small, smallC, cv::COLOR_GRAY2BGR);
            smallC.copyTo(canvas(cv::Rect(canvas.cols - small.cols - 4, canvas.rows - small.rows - 4, small.cols, small.rows)));
        }
    }
}

void MissionLoop::startRecording() {
    std::filesystem::create_directories(opt_.logDir);
    recBase_ = opt_.logDir + "/run_" + timestamp();
    int w = perception_.projector().K().width, h = perception_.projector().K().height;
    if (!writer_.open(recBase_ + ".avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(w, h))) {
        LOG_WARN("cannot open video writer %s.avi", recBase_.c_str());
    }
    csv_.open(recBase_ + ".csv");
    csv_ << "t,seq,phase,bat,tof,h,yaw,pitch,roll,line_valid,line_mode,near_y,heading,curvature,end_visible,tag_id,tag_x,tag_y,rc_lr,rc_fb,rc_ud,rc_yaw\n";
    logx::setFile(recBase_ + ".log");
    recording_ = true;
    LOG_INFO("recording to %s.{avi,csv,log}", recBase_.c_str());
}

void MissionLoop::stopRecording() {
    if (!recording_) return;
    writer_.release();
    csv_.close();
    recording_ = false;
    LOG_INFO("recording stopped");
}

void MissionLoop::record(const cv::Mat& frame, const Observation& obs, const TelloState& st) {
    if (writer_.isOpened()) {
        int w = perception_.projector().K().width, h = perception_.projector().K().height;
        if (frame.cols == w && frame.rows == h) writer_.write(frame);
        else { cv::Mat r; cv::resize(frame, r, cv::Size(w, h)); writer_.write(r); }
    }
    const TagObservation* tg = obs.tags.empty() ? nullptr : &obs.tags.front();
    RcCommand rc = nav_.lastRc();
    csv_ << obs.stamp << "," << obs.seq << "," << nav_.phaseName() << "," << st.bat << "," << st.tof << "," << st.h << "," << st.yaw << ","
         << st.pitch << "," << st.roll << "," << (obs.line.valid ? 1 : 0) << "," << obs.line.mode << "," << obs.line.nearY << ","
         << obs.line.headingDeg << "," << obs.line.curvature << "," << (obs.line.endVisible ? 1 : 0) << "," << (tg ? tg->id : -1) << ","
         << (tg ? tg->X : 0) << "," << (tg ? tg->Y : 0) << "," << rc.lr << "," << rc.fb << "," << rc.ud << "," << rc.yaw << "\n";
}

}  // namespace

bool openLiveSource(const Config& cfg, const MissionOptions& opt, IClock& clock, std::unique_ptr<IDrone>& drone,
                    std::unique_ptr<IFrameSource>& src, std::string* err) {
    bool tello = opt.source == "udp" || opt.source.rfind("udp://", 0) == 0;
    if (tello) {
        auto td = std::make_unique<TelloDrone>(cfg, clock);
        td->setDryRun(opt.dryRun);
        if (!td->connect(err)) return false;
        if (cfg.getBool("tello.downvision", false)) td->setDownvision(true);
        if (!td->streamOn()) LOG_WARN("streamon was not acknowledged; trying to open the video anyway");
        src = std::make_unique<VideoCaptureSource>(opt.source == "udp" ? td->videoUrl() : opt.source, true);
        drone = std::move(td);
    } else {
        drone = std::make_unique<NullDrone>();
        src = std::make_unique<VideoCaptureSource>(opt.source, opt.realtime);
    }
    return src->start(err);
}

int runFly(const Config& cfg, const MissionOptions& opt) {
    SteadyClock clock;
    std::unique_ptr<IDrone> drone;
    std::unique_ptr<IFrameSource> src;
    std::string err;
    if (!openLiveSource(cfg, opt, clock, drone, src, &err)) { LOG_ERROR("%s", err.c_str()); return 1; }
    if (opt.dryRun) LOG_WARN("DRY RUN: flight commands are logged, not sent");
    WebServer web;
    MissionLoop loop(*drone, *src, clock, cfg, opt, nullptr, &web);
    int rc = loop.run();
    web.stop();
    src->stop();
    drone->disconnect();
    return rc;
}

int runSim(const Config& cfg, const MissionOptions& opt) {
    int finished = 0;
    double sumT = 0;
    std::vector<std::string> rows;
    WebServer web;   // one server for the whole batch: the browser stays connected between episodes
    for (int ep = 0; ep < std::max(1, opt.episodes); ++ep) {
        uint64_t seed = opt.seed + ep;
        TagDetector renderer;
        std::string err;
        if (!renderer.init(cfg.getString("tag.family", "tag36h11"), 2.0, 0.0, 1, false, 0.25, &err)) { LOG_ERROR("%s", err.c_str()); return 1; }
        Simulator sim(cfg, seed, opt.track, renderer);
        MissionLoop loop(sim, sim, sim, cfg, opt, &sim, &web);
        loop.run();
        const auto& r = sim.result();
        char row[256];
        snprintf(row, sizeof(row), "episode %2d seed %llu: %-9s segments=%d/%zu time=%6.1fs  (%s)", ep + 1, (unsigned long long)seed,
                 r.finished ? "FINISHED" : (r.crashed ? "CRASHED" : "FAILED"), r.segments, sim.track().tags.size(), r.time, r.reason.c_str());
        LOG_INFO("%s", row);
        rows.push_back(row);
        if (r.finished) { ++finished; sumT += r.time; }
    }
    web.stop();
    printf("\n==== simulation summary: %d/%d courses completed, mean time %.1fs ====\n", finished, std::max(1, opt.episodes),
           finished ? sumT / finished : 0.0);
    for (auto& r : rows) printf("%s\n", r.c_str());
    return finished == std::max(1, opt.episodes) ? 0 : 2;
}
