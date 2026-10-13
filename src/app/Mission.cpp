#include "app/Mission.h"
#include "app/WebServer.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <sstream>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "control/Navigator.h"
#include "sim/Simulator.h"
#include "tello/TelloDrone.h"
#include "util/Log.h"
#include "vision/Perception.h"

namespace {

// Signal level of the wireless interface that carries the drone link (Linux /proc/net/wireless), dBm; 0 if unknown.
static int readWifiSignalDbm(const std::string& want) {   // want = interface name, or "auto" = strongest live one
#ifdef _WIN32
    return 0;
#else
    FILE* f = fopen("/proc/net/wireless", "r");
    if (!f) return 0;
    // The drone may be on the built-in radio (wlan0) or on a USB adapter (wlan1): report the interface with the
    // strongest live signal; an idle interface reads 0.
    char line[256]; int best = 0;
    while (fgets(line, sizeof(line), f)) {
        char ifname[32]; unsigned status; double link, level;
        if (sscanf(line, " %31[^:]: %x %lf %lf", ifname, &status, &link, &level) == 4) {
            int lv = (int)level;
            if (want != "auto") { if (want == ifname) { best = lv; break; } continue; }
            if (lv != 0 && (best == 0 || lv > best)) best = lv;
        }
    }
    fclose(f);
    if (best == 0 && want != "auto" && want.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789_-.") == std::string::npos) {
        // USB adapters (rtw88) do not appear in /proc/net/wireless: ask iw once a second instead
        std::string cmd = "/usr/sbin/iw dev " + want + " link 2>/dev/null";
        if (FILE* p = popen(cmd.c_str(), "r")) {
            while (fgets(line, sizeof(line), p)) { int lv; if (sscanf(line, " signal: %d dBm", &lv) == 1) best = lv; }
            pclose(p);
        }
    }
    return best;
#endif
}

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
public:
    const std::string& simRestart() const { return simRestart_; }
private:
    bool restartVideo(const char* why);   // re-request the Tello stream and re-open the decoder
    std::string stateJson(const Observation& obs, const TelloState& st, double fps);
    void applyWebCommand(const WebCommand& c);
    void setManual(bool on);
    void setLineColor(const std::string& name);
    void setHeading(const std::string& name);

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
    bool abortEpisode_ = false;      // simulator: end this episode now (settings change from the page)
    std::string simRestart_;         // requested settings "pitch=..;alt=..;track=..;seed=.."
    uint64_t seq_ = 0;
    int startAtTag_ = 0;
    int videoRecoveries_ = 0;
    double lastCameraSwitchT_ = 0;
    std::string recBase_;
    WebServer* web_ = nullptr;   // owned by runFly/runSim so it outlives simulator episodes
    bool webOn_ = false;
    bool manual_ = false;                 // manual mode: the browser drives the sticks, the navigator is idle
    int manualRc_[4] = {0, 0, 0, 0};
    double manualRcT_ = 0;
    double lastStaleLog_ = 0;
    int flipCode_ = 2;     // cv::flip code for live frames; 2 = no flip (camera.flip)
    int blankTopPx_ = 0;   // rows at the top (after the flip) that a short mirror pane does not cover (camera.blank_top_px)
    std::string flipName_ = "none";
    void setFlip(const std::string& f) {   // none | horizontal | vertical | both
        flipCode_ = f == "horizontal" ? 1 : f == "vertical" ? 0 : f == "both" ? -1 : 2;
        flipName_ = flipCode_ == 2 ? "none" : f;
    }
    double percMs_ = 0, iterMs_ = 0, tagMsAcc_ = 0, lineMsAcc_ = 0; int perfN_ = 0; double lastSlowLog_ = 0;
    double totTagMs_ = 0, totLineMs_ = 0, totPercMs_ = 0; long totFrames_ = 0, totTagRuns_ = 0;   // whole run: what the vision costs
public:
    // mean cost of the vision per processed frame over the whole run (ms): everything, tag detector, tape
    void visionCost(double& all, double& tag, double& line) const {
        all = totFrames_ ? totPercMs_ / totFrames_ : 0; tag = totFrames_ ? totTagMs_ / totFrames_ : 0; line = totFrames_ ? totLineMs_ / totFrames_ : 0;
    }
    Perception::Calib calib() const { return perception_.calib(); }
    double rcGain() const { return nav_.rcGain(); }
private:
    // What this flight showed about the mirror and the drone, kept for the next start (ground.learned_file).
    void saveLearned() {
        const std::string path = cfg_.getString("ground.learned_file", "");
        if (path.empty() || !nav_.courseMode()) return;
        const Perception::Calib cal = perception_.calib();
        const bool mount = cal.applied && cal.samples >= 60;
        const bool gain = nav_.rcGainMeasured();
        if (!mount && !gain && !perception_.pitchSignKnown() && !perception_.rollSignKnown()) return;
        Config keep;                                   // what an earlier flight found stays unless this one found it again
        keep.loadFile(path, nullptr);
        char buf[64];
        if (mount) {
            snprintf(buf, sizeof(buf), "%.2f", cal.pitchDeg); keep.set("camera.pitch_deg", buf);
            snprintf(buf, sizeof(buf), "%.2f", cal.rollDeg); keep.set("camera.roll_deg", buf);
            keep.set("ground.mount_verified", "1");
        }
        if (perception_.pitchSignKnown()) keep.set("ground.imu_pitch_sign", perception_.imuPitchSign() < 0 ? "-1" : "1");
        if (perception_.rollSignKnown()) keep.set("ground.imu_roll_sign", perception_.imuRollSign() < 0 ? "-1" : "1");
        if (perception_.pitchSignKnown() && perception_.rollSignKnown()) keep.set("ground.imu_signs_verified", "1");
        if (gain) { snprintf(buf, sizeof(buf), "%.4f", nav_.rcGain()); keep.set("course.rc_gain_ms", buf); }
        std::ofstream f(path);
        if (!f) { LOG_WARN("could not write %s", path.c_str()); return; }
        f << "# Written by tello_race after a flight: what the tags showed about the camera mirror and the drone.\n"
          << "# It is laid over the configuration at the next start. Delete it after changing the mirror clip or the drone.\n";
        std::string section;
        for (const auto& kv : keep.all()) {
            const size_t dot = kv.first.find('.');
            if (dot == std::string::npos) continue;
            if (kv.first.substr(0, dot) != section) { section = kv.first.substr(0, dot); f << "\n[" << section << "]\n"; }
            f << kv.first.substr(dot + 1) << " = " << kv.second << "\n";
        }
        LOG_INFO("kept for the next flight (%s): camera pitch %s roll %s deg, stick gain %s", path.c_str(),
                 keep.getString("camera.pitch_deg", "-").c_str(), keep.getString("camera.roll_deg", "-").c_str(),
                 keep.getString("course.rc_gain_ms", "-").c_str());
    }
    std::string notice_;   // last one-line message for the page (shown once per id)
    int noticeId_ = 0;
    bool prevFlying_ = false;
    double idleFps_ = 0, lastIdleFrame_ = -1e9;   // mission.idle_fps: pictures per second looked at while on the ground
    double lastLearnedT_ = 0;
    bool mountWarned_ = false;
    int lastLearnedSamples_ = -1;
    void notice(const std::string& t) { notice_ = t; ++noticeId_; LOG_INFO("notice: %s", t.c_str()); }
    int wifiDbm_ = 0;      // drone link as seen by the ground station, dBm (0 = unknown)
    std::string wifiIfname_ = "auto";   // tello.wifi_ifname
    double wifiT_ = 0;
};

int MissionLoop::run() {
    std::string err;
    if (!perception_.init(cfg_, &err)) { LOG_ERROR("perception init failed: %s", err.c_str()); return 1; }
    startAtTag_ = opt_.startAtTag;
    const std::string win = "tello_race";
    if (!opt_.headless) cv::namedWindow(win, cv::WINDOW_AUTOSIZE);
    if (!sim_) {
        wifiIfname_ = cfg_.getString("tello.wifi_ifname", "auto");
        setFlip(cfg_.getString("camera.flip", "none"));
        idleFps_ = cfg_.getDouble("mission.idle_fps", 10.0);
        if (flipCode_ != 2) LOG_INFO("camera.flip = %s: live frames are flipped before perception (mirror mount)", flipName_.c_str());
        blankTopPx_ = std::max(0, cfg_.getInt("camera.blank_top_px", 0));
        if (blankTopPx_ > 0) LOG_INFO("camera.blank_top_px = %d: the top rows are painted white (image beyond the mirror pane)", blankTopPx_);
    }
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
    if (webOn_ && sim_) {   // scene for the /sim3d page: track geometry, tags, the rendered floor as a texture
        const Track& tr = sim_->track();
        std::string js = "{\"worldSize\":" + std::to_string(tr.worldSize) + ",\"pxPerM\":" + std::to_string(sim_->pxPerM()) +
                         ",\"hfov\":" + std::to_string(sim_->hfovDeg()) + ",\"vfov\":" + std::to_string(sim_->vfovDeg()) +
                         ",\"pitch\":" + std::to_string(sim_->camPitchTrueDeg()) + ",\"alt\":" + std::to_string(cfg_.getInt("mission.altitude_cm", 0)) +
                         ",\"track\":\"" + opt_.track + "\",\"seed\":" + std::to_string(sim_->seed()) + ",\"segments\":[";
        for (size_t i = 0; i < tr.segments.size(); ++i) {
            js += i ? ",[" : "[";
            for (size_t k = 0; k < tr.segments[i].pts.size(); ++k) {
                char b[64]; snprintf(b, sizeof(b), "%s[%.3f,%.3f]", k ? "," : "", tr.segments[i].pts[k].x, tr.segments[i].pts[k].y); js += b;
            }
            js += "]";
        }
        js += "],\"tags\":[";
        for (size_t i = 0; i < tr.tags.size(); ++i) {
            char b[96]; snprintf(b, sizeof(b), "%s{\"id\":%d,\"x\":%.3f,\"y\":%.3f,\"yaw\":%.1f}", i ? "," : "", tr.tags[i].id, tr.tags[i].x, tr.tags[i].y, tr.tags[i].yawDeg); js += b;
        }
        js += "]}";
        std::vector<unsigned char> png;
        cv::Mat small; cv::resize(sim_->worldImage(), small, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
        cv::imencode(".png", small, png);
        web_->setSimScene(js, png);
        LOG_INFO("web: simulation 3D view at /sim3d (%zu segments, %zu tags, floor texture %dx%d)", tr.segments.size(), tr.tags.size(), small.cols, small.rows);
    }
    if (sim_) {   // blocking go/turn commands: pace them in wall time and stream the pose so viewers see the flight
        bool pace = opt_.realtime && (!opt_.headless || webOn_);
        sim_->setBlockingStepCallback([this, pace](double dtSim) {
            static SteadyClock wc; static double acc = 0, lastPub = -1;
            if (pace) wc.sleep(dtSim);
            acc += dtSim;
            if (webOn_ && (lastPub < 0 || acc - lastPub >= 0.1)) {
                lastPub = acc;
                Observation none; none.stamp = clock_.now();
                web_->publishState(stateJson(none, drone_.state(), 0.0));
            }
        });
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
        if (abortEpisode_) break;
        WebCommand wc;
        while (webOn_ && web_->popCommand(wc)) applyWebCommand(wc);
        double tIter0 = wall.now();
        bool got = src_.latest(frame, seq_, 0.25);
        // On the ground with nothing flying a few pictures a second are plenty: the others are dropped unprocessed
        // (the vision and the page video are most of what the ground station computes).
        if (got && idleFps_ > 0 && !sim_ && !nav_.flying() && !drone_.isFlying()) {
            if (wall.now() - lastIdleFrame_ < 1.0 / idleFps_) {
                seq_ = frame.seq; lastFrameWall = wall.now();
                got = false;
            } else {
                lastIdleFrame_ = wall.now();
            }
        }
        if (got && flipCode_ != 2) cv::flip(frame.image, frame.image, flipCode_);   // mirror mount: un-mirror before perception
        if (got && blankTopPx_ > 0 && blankTopPx_ < frame.image.rows)                 // short pane: those rows see past the mirror
            frame.image(cv::Rect(0, 0, frame.image.cols, blankTopPx_)).setTo(cv::Scalar(255, 255, 255));
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
        // Two statements on purpose: as arguments of one call, hx/hy were copied before lineHint() wrote them
        // (unspecified evaluation order), so the detector always got a (0,0) hint.
        bool haveHint = nav_.lineHint(hx, hy);
        perception_.setLineHint(haveHint, hx, hy);
        if (got) {
            double tp = wall.now();
            obs = perception_.process(frame.image, st, frame.stamp, frame.seq);
            const double pm = (wall.now() - tp) * 1000;
            percMs_ += pm; iterMs_ += (wall.now() - tIter0) * 1000; tagMsAcc_ += obs.tagMs; lineMsAcc_ += obs.lineMs; ++perfN_;
            totPercMs_ += pm; totTagMs_ += obs.tagMs; totLineMs_ += obs.lineMs; ++totFrames_;
        }
        else { obs.stamp = clock_.now(); obs.blindZoneM = perception_.projector().blindZoneX(); }
        // Keep SDK mode alive while idle on the ground. Never while airborne in ANY mode: a blocking query pauses
        // the rc stream until its reply, and on a marginal link that is 3 s of dead sticks every 8 s (the rc
        // stream itself keeps the SDK alive in flight).
        if (!sim_ && !nav_.flying() && !drone_.isFlying() && wall.now() - lastKeepAlive > 8.0) {
            lastKeepAlive = wall.now();
            drone_.queryInt("battery?");
        }
        if (!sim_ && nav_.flying() && wall.now() - lastFrameWall > 2.5) {
            LOG_ERROR("no video for 2.5 s while flying -> landing");
            nav_.requestLand();
        }
        if (manual_) {
            int r0 = manualRc_[0], r1 = manualRc_[1], r2 = manualRc_[2], r3 = manualRc_[3];
            if (wall.now() - manualRcT_ > 0.5) {   // dead-man: no fresh sticks from the page -> hover
                // The page heartbeats the sticks (zeros included) 4x/s while airborne, so silence means the page
                // is hidden/throttled or the operator link is down, not merely idle sticks.
                if (manualRcT_ > 0 && drone_.isFlying() && wall.now() - lastStaleLog_ > 2.0) {
                    LOG_WARN("manual: no stick input or heartbeat from the page for %.1fs (page in background, or operator link?)", wall.now() - manualRcT_);
                    lastStaleLog_ = wall.now();
                }
                r0 = r1 = r2 = r3 = 0;
            }
            if (drone_.isFlying()) drone_.rc(r0, r1, r2, r3);
        } else if (got || !sim_) {
            nav_.step(obs, st);
            if (sim_) sim_->pauseDeviation(nav_.phase() == Phase::TagManeuver && nav_.courseMode());
        }
        if (!sim_ && wall.now() - wifiT_ > 1.0) { wifiT_ = wall.now(); wifiDbm_ = readWifiSignalDbm(wifiIfname_); }
        { bool fl = drone_.isFlying();
          if (prevFlying_ && !fl) { notice("drone is on the ground: TAKE OFF is available again"); saveLearned(); }
          // a mirror that looks far backwards leaves the camera blind ahead of the drone: say so once
          if (!mountWarned_ && nav_.courseMode()) {
              const Perception::Calib cal = perception_.calib();
              if (cal.applied && cal.samples >= 60 && (cal.pitchDeg > 103.0 || cal.pitchDeg < 84.0)) {
                  mountWarned_ = true;
                  char b[200];
                  snprintf(b, sizeof(b), "mirror angle %.0f deg: the camera sees very little of the floor %s the drone. Re-seat the clip (96-99 deg expected)",
                           cal.pitchDeg, cal.pitchDeg > 103.0 ? "ahead of" : "behind");
                  notice(b);
              }
          }
          // held by hand over the tags (no flight at all): the mirror angle found that way is kept too
          if (!fl && !sim_ && wall.now() - lastLearnedT_ > 5.0) {
              lastLearnedT_ = wall.now();
              const Perception::Calib cal = perception_.calib();
              if (cal.applied && cal.samples >= 60 && cal.samples != lastLearnedSamples_) { lastLearnedSamples_ = cal.samples; saveLearned(); }
          }
          prevFlying_ = fl; }
        if (recording_ && got) record(frame.image, obs, st);
        if (got && ++fpsN && wall.now() - fpsT > 1.0) {
            fps = fpsN / (wall.now() - fpsT); fpsN = 0; fpsT = wall.now();
            if (!sim_ && fps < 10 && src_.fps() > 12 && perfN_ > 0 && wall.now() - lastSlowLog_ > 5.0) {   // the camera delivers, the loop does not keep up
                LOG_WARN("slow loop: %.1f fps processed of %.1f delivered; perception %.0f ms/frame (tags %.0f, line %.0f), wait+perception %.0f ms/frame",
                         fps, src_.fps(), percMs_ / perfN_, tagMsAcc_ / perfN_, lineMsAcc_ / perfN_, iterMs_ / perfN_);
                lastSlowLog_ = wall.now();
            }
            percMs_ = iterMs_ = tagMsAcc_ = lineMsAcc_ = 0; perfN_ = 0;
        }

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
    else if (c.cmd == "mirror") {   // live choice of the un-mirroring flip for a mirror mount
        if (sim_) notice("mirror flip applies to the real camera only");
        else { setFlip(c.arg); notice("camera mirror flip: " + flipName_ + " (make it permanent with camera.flip in the config)"); }
    }
    else if (c.cmd == "flip") {   // SDK "flip f|b|l|r": the Tello does the acrobatics itself
        std::string d = c.arg.empty() ? "" : c.arg.substr(0, 1);
        if (d != "f" && d != "b" && d != "l" && d != "r") notice("flip: direction must be f, b, l or r");
        else if (!manual_) notice("flips are only available in MANUAL mode");
        else if (!drone_.isFlying()) notice("flip ignored: the drone is on the ground");
        else if (drone_.state().valid && drone_.state().bat < 55) notice("flip refused: the Tello needs more than about 50 % battery");
        else {
            std::string r;
            bool ok = drone_.command("flip " + d, &r, 8.0);   // blocks ~2 s; the rc stream resumes afterwards
            notice(ok ? "flip done" : "flip refused by the drone: " + (r.empty() ? std::string("no reply") : r));
        }
    }
    else if (c.cmd == "takeoff") {
        if (!manual_) notice("TAKE OFF works in MANUAL mode; in mission mode press START");
        else if (drone_.isFlying()) notice("take-off ignored: the drone reports it is already flying");
        else if (!drone_.takeoff()) { LOG_WARN("manual takeoff refused: %s", drone_.lastReply.c_str()); notice("take-off refused by the drone: " + drone_.lastReply); }
        else manualRcT_ = wallNow();   // takeoff blocks the loop for ~7 s: do not count that as a stick-input gap
    }
    else if (c.cmd == "start") { if (!manual_) handleKey('t'); else LOG_WARN("START ignored in manual mode: switch to mission mode first"); }
    else if (c.cmd == "land") {
        if (manual_) { manualRc_[0] = manualRc_[1] = manualRc_[2] = manualRc_[3] = 0; if (!drone_.land()) notice("land refused by the drone: " + drone_.lastReply + (drone_.isFlying() ? "" : " (it is on the ground)")); }
        else handleKey('l');
    }
    else if (c.cmd == "emergency") handleKey('e');
    else if (c.cmd == "record") handleKey('r');
    else if (c.cmd == "sim") {   // restart the simulation with other settings: pitch=..;alt=..;track=..;seed=..
        if (!sim_) notice("sim settings apply to the simulator only");
        else { simRestart_ = c.arg.empty() ? "same" : c.arg; abortEpisode_ = true; notice("simulation restarting: " + simRestart_); }
    }
    else if (c.cmd == "linecolor") setLineColor(c.arg);
    else if (c.cmd == "heading") setHeading(c.arg);
    else if (c.cmd == "camera") handleKey('v');
    else if (c.cmd == "tag") handleKey('0' + std::max(0, std::min(3, atoi(c.arg.c_str()))));
}

bool MissionLoop::restartVideo(const char* why) {
    // Switching back to the 960x720 front camera needs longer than the 320x240 belly one: the drone
    // re-initialises its encoder and ignores a streamon that arrives too early. Back off on each try,
    // and never report success until a frame has actually been decoded.
    // Measured on the Tello Talent (2026-09-12, battery 39 %): after streamoff and a camera change the drone
    // answers "ok" to a streamon sent 1 s or 2 s later and sends nothing; a streamon sent 3 s later streams.
    // So wait long enough BEFORE streamon, bind the socket right after it, and prove a frame was decoded.
    static const double kSettle[3] = {3.0, 4.5, 6.0};
    for (int attempt = 1; attempt <= 3; ++attempt) {
        double settle = kSettle[attempt - 1];
        LOG_WARN("video: re-requesting the stream (%s, attempt %d/3, settle %.1fs)", why, attempt, settle);
        src_.stop();
        std::string r;
        bool offOk = drone_.command("streamoff", &r, 3.0);
        LOG_INFO("video: streamoff -> %s%s", r.empty() ? "(no reply)" : r.c_str(), offOk ? "" : " [not acknowledged]");
        clock_.sleep(settle);
        bool onOk = drone_.streamOn();
        LOG_INFO("video: streamon -> %s%s", drone_.lastReply.empty() ? "(no reply)" : drone_.lastReply.c_str(),
                 onOk ? "" : " [not acknowledged]");
        // No pause here on purpose: bind the receiving socket immediately. Waiting after streamon
        // lets the drone's first packets hit a closed port, and it then stops sending altogether.
        std::string e;
        if (!src_.start(&e)) { LOG_ERROR("video: re-open failed: %s", e.c_str()); clock_.sleep(0.5); continue; }
        if (src_.waitFirstFrame(5.0)) {
            seq_ = 0;
            LOG_INFO("video: stream re-opened, frames flowing (%s, %.1fs settle)", why, settle);
            return true;
        }
        LOG_WARN("video: no frame within 4 s of streamon (settle %.1fs)", settle);
    }
    LOG_ERROR("video: the drone did not resume the stream after 3 attempts (%s)", why);
    return false;
}

// Which tape the vision follows. Not while a mission is flying: the drone would lose its line in mid-air.
void MissionLoop::setLineColor(const std::string& name) {
    if (!manual_ && nav_.flying()) { notice("colour not changed: the mission is flying (land, or switch to MANUAL first)"); return; }
    if (!perception_.lineDetector().setColor(name)) { notice("unknown tape colour '" + name + "' (blue, yellow or dark)"); return; }
    perception_.resetTracking();
    notice("now following the " + perception_.lineDetector().color() + " tape");
}

// Course mode: does the nose stay where it pointed at take-off, turn with the route, or turn ahead of it? Same rule
// as the colour: not while a mission is in the air.
void MissionLoop::setHeading(const std::string& name) {
    if (name != "fixed" && name != "path" && name != "lead") { notice("unknown heading mode '" + name + "' (fixed, path or lead)"); return; }
    if (!nav_.courseMode()) { notice("heading mode only applies to the course profile"); return; }
    if (!manual_ && nav_.flying()) { notice("heading mode not changed: the mission is flying (land, or switch to MANUAL first)"); return; }
    nav_.setCourseNoseMode(name);
    notice(name == "lead" ? "the nose will turn ahead of the path" : name == "path" ? "the nose will turn with the path" : "the nose will keep its take-off heading");
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
    s.wifi = wifiDbm_;
    s.notice = notice_; s.noticeId = noticeId_;
    s.mirror = flipName_;
    s.lineColor = perception_.lineDetector().color();
    s.heading = nav_.courseNoseMode();
    s.course = nav_.courseMode();
    s.nextTag = nav_.nextTagId();
    s.speed = nav_.speedMs();
    if (manual_) {   // show the operator the sticks actually applied (zeros once the dead-man has fired)
        bool stale = wallNow() - manualRcT_ > 0.5;
        s.rcAge = manualRcT_ > 0 ? wallNow() - manualRcT_ : -1;
        s.stale = stale;
        for (int i = 0; i < 4; ++i) s.rc[i] = stale ? 0 : manualRc_[i];
    } else {
        RcCommand rc = nav_.lastRc();
        s.rc[0] = rc.lr; s.rc[1] = rc.fb; s.rc[2] = rc.ud; s.rc[3] = rc.yaw;
    }
    s.events = nav_.events;
    if (sim_) { s.sim = true; s.simX = sim_->x(); s.simY = sim_->y(); s.simZ = sim_->z(); s.simYaw = sim_->yawDeg(); s.camPitch = sim_->camPitchTrueDeg(); }
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
        case 'b': setLineColor("blue"); return true;
        case 'y': setLineColor("yellow"); return true;
        case 'k': setLineColor("dark"); return true;
        case 'n': {   // nose: fixed -> along the path -> ahead of the path -> fixed
            const std::string nose = nav_.courseNoseMode();
            setHeading(nose == "fixed" ? "path" : nose == "path" ? "lead" : "fixed");
            return true;
        }
        case 'v': {   // toggle front / belly camera (SDK 3.0 "downvision"); the stream is reopened
            auto* td = dynamic_cast<TelloDrone*>(&drone_);
            if (!td) { LOG_INFO("camera switch is only available on the real drone"); return true; }
            if (nav_.flying() || drone_.isFlying()) { LOG_WARN("not switching cameras while flying"); notice("camera switch refused while flying: land first"); return true; }
            if (wallNow() - lastCameraSwitchT_ < 3.0) {   // a switch blocks for seconds; clicks queued meanwhile are dropped
                LOG_WARN("camera switch ignored: the previous one finished %.1fs ago", wallNow() - lastCameraSwitchT_);
                return true;
            }
            bool on = !td->downvision();
            if (drone_.state().valid && drone_.state().temph >= 75)   // 81 C: the switch failed 3/3 with bytes arriving
                notice("drone is hot (" + std::to_string(drone_.state().temph) + " C): camera switching is unreliable when hot, let it cool");
            src_.stop();
            std::string r;
            drone_.command("streamoff", &r, 3.0);   // the belly camera streams 320x240 grey, the front 960x720:
            if (td->setDownvision(on)) {            // the decoder must be torn down before the format changes
                perception_.setDownwardCamera(on);
                LOG_INFO("video: %s camera", on ? "downward (belly)" : "front");
            }
            videoRecoveries_ = 0;
            if (!restartVideo(on ? "belly camera" : "front camera")) LOG_ERROR("video did not come back after the camera switch");
            lastCameraSwitchT_ = wallNow();
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
    // which tape is being followed: a label, and in course mode the route itself drawn over the tape
    const std::string colName = perception_.lineDetector().color();
    const cv::Scalar colBgr = colName == "blue" ? cv::Scalar(255, 120, 0) : colName == "yellow" ? cv::Scalar(0, 220, 255) : cv::Scalar(60, 60, 60);
    {
        std::string label = "FOLLOW " + std::string(colName == "blue" ? "BLUE" : colName == "yellow" ? "YELLOW" : colName == "dark" ? "BLACK" : "HSV");
        if (nav_.courseMode()) {
            const std::string nose = nav_.courseNoseMode();
            label += nose == "lead" ? "  NOSE: AHEAD" : nose == "path" ? "  NOSE: PATH" : "  NOSE: FIXED";
        }
        int base = 0;
        cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &base);
        cv::Rect box(canvas.cols - ts.width - 22, 124, ts.width + 16, ts.height + 12);
        cv::rectangle(canvas, box, colBgr, -1);
        cv::rectangle(canvas, box, cv::Scalar(255, 255, 255), 1);
        cv::putText(canvas, label, cv::Point(box.x + 8, box.y + ts.height + 5), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    colName == "yellow" ? cv::Scalar(0, 0, 0) : cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    }
    if (haveObs && nav_.courseMode() && nav_.courseRoute.size() >= 2) {
        std::vector<cv::Point> px;
        for (const auto& q : nav_.courseRoute) {
            double u, v;
            if (perception_.projector().groundToPixel(q.x, q.y, u, v)) px.emplace_back((int)(u * s), (int)(v * s));
        }
        if (px.size() >= 2) {
            cv::polylines(canvas, px, false, cv::Scalar(255, 255, 255), 7, cv::LINE_AA);   // white rim so it shows on any floor
            cv::polylines(canvas, px, false, colBgr, 3, cv::LINE_AA);
        }
        int w = nav_.courseWaypoint;
        if (w >= 0 && w < (int)nav_.courseRoute.size()) {
            double u, v;
            if (perception_.projector().groundToPixel(nav_.courseRoute[w].x, nav_.courseRoute[w].y, u, v))
                cv::drawMarker(canvas, cv::Point((int)(u * s), (int)(v * s)), cv::Scalar(0, 255, 0), cv::MARKER_TILTED_CROSS, 26, 3);
        }
    }
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
        // Optional video settings of the SDK 3.0 firmware (Tello EDU / Talent). Empty = leave the drone as it is.
        // A lighter stream is a little less work and heat for the drone, and less to decode on the ground.
        struct { const char* key; const char* cmd; } vset[] = {
            {"tello.video_fps", "setfps"}, {"tello.video_bitrate", "setbitrate"}, {"tello.video_resolution", "setresolution"}};
        for (const auto& v : vset) {
            const std::string val = cfg.getString(v.key, "");
            if (val.empty() || opt.dryRun) continue;
            std::string reply;
            const bool ok = td->command(std::string(v.cmd) + " " + val, &reply, 3.0);
            if (ok) LOG_INFO("video: %s %s accepted", v.cmd, val.c_str());
            else LOG_WARN("video: %s %s not accepted (%s): needs SDK 3.0 firmware", v.cmd, val.c_str(), reply.c_str());
        }
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
    Config live = cfg;            // settings the web page may change between episodes (camera angle, altitude, course)
    MissionOptions o = opt;
    for (int ep = 0; ep < std::max(1, o.episodes); ++ep) {
        uint64_t seed = o.seed + ep;
        TagDetector renderer;
        std::string err;
        if (!renderer.init(live.getString("tag.family", "tag36h11"), 2.0, 0.0, 1, false, 0.25, &err)) { LOG_ERROR("%s", err.c_str()); return 1; }
        Simulator sim(live, seed, o.track, renderer);
        MissionLoop loop(sim, sim, sim, live, o, &sim, &web);
        loop.run();
        if (!loop.simRestart().empty()) {   // page asked for other settings: apply them and start over from that seed
            std::string req = loop.simRestart(), item;
            std::istringstream ss(req);
            while (std::getline(ss, item, ';')) {
                size_t eq = item.find('='); if (eq == std::string::npos) continue;
                std::string k = item.substr(0, eq), v = item.substr(eq + 1);
                if (k == "pitch") { live.set("sim.cam_pitch_true_deg", v); live.set("camera.pitch_deg", v); }
                else if (k == "alt") live.set("mission.altitude_cm", v);
                else if (k == "track") o.track = v.empty() ? "random" : v;
                else if (k == "color") live.set("line.color_mode", v);
                else if (k == "heading") live.set("course.heading", v);
                else if (k == "seed") o.seed = (uint64_t)std::max(0LL, atoll(v.c_str()));
            }
            LOG_INFO("simulation restarted from the page: %s -> pitch %s deg, altitude %s cm, track %s, seed %llu", req.c_str(),
                     live.getString("sim.cam_pitch_true_deg", "?").c_str(), live.getString("mission.altitude_cm", "?").c_str(), o.track.c_str(),
                     (unsigned long long)o.seed);
            ep = -1;           // restart the count at the requested seed
            continue;
        }
        const auto& r = sim.result();
        double vAll, vTag, vLine;
        loop.visionCost(vAll, vTag, vLine);
        const Perception::Calib cal = loop.calib();
        LOG_INFO("vision cost per frame: %.1f ms (tags %.1f ms, tape %.1f ms)   camera found at pitch %.1f roll %.1f deg, height tags/sensor %.3f (%d sightings; true %.1f)   stick gain %.4f",
                 vAll, vTag, vLine, cal.pitchDeg, cal.rollDeg, cal.heightRatio, cal.samples, sim.camPitchTrueDeg(), loop.rcGain());
        char row[400], pass[96] = "";
        for (size_t i = 0; i < r.tagPass.size() && i < 6; ++i)
            snprintf(pass + strlen(pass), sizeof(pass) - strlen(pass), "%s%.0f", i ? "/" : "", std::min(999.0, r.tagPass[i] * 100.0));
        snprintf(row, sizeof(row), "episode %2d seed %llu: %-9s segments=%d/%zu time=%6.1fs  path dev mean %.3f rms %.3f max %.3f m  tag pass %s cm  speed %.2f m/s  (%s)",
                 ep + 1, (unsigned long long)seed, r.finished ? "FINISHED" : (r.crashed ? "CRASHED" : "FAILED"), r.segments, sim.track().tags.size(), r.time,
                 r.devN ? r.devSum / r.devN : 0.0, r.devN ? std::sqrt(r.devSq / r.devN) : 0.0, r.devMax, pass, r.time > 0 ? r.pathLen / r.time : 0.0, r.reason.c_str());
        LOG_INFO("%s", row);
        rows.push_back(row);
        if (r.finished) { ++finished; sumT += r.time; }
    }
    web.stop();
    printf("\n==== simulation summary: %d/%d courses completed, mean time %.1fs ====\n", finished, std::max(1, opt.episodes),
           finished ? sumT / finished : 0.0);
    for (auto& r : rows) printf("%s\n", r.c_str());
    return finished == std::max(1, o.episodes) ? 0 : 2;
}
