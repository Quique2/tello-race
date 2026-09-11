#include "app/Tools.h"

#include <filesystem>
#include <memory>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "tello/TelloDrone.h"
#include "util/Log.h"
#include "util/MathUtil.h"
#include "vision/Perception.h"

static const char* tagMeaning(int id) {
    switch (id) { case 1: return "FORWARD"; case 2: return "LEFT"; case 3: return "RIGHT"; case 4: return "FINISH"; default: return "unused"; }
}

int toolGenTags(const Config& cfg, const MissionOptions& opt) {
    TagDetector td;
    std::string err, family = cfg.getString("tag.family", "tag36h11");
    if (!td.init(family, 2.0, 0.0, 1, false, 0.25, &err)) { LOG_ERROR("%s", err.c_str()); return 1; }
    std::filesystem::create_directories(opt.outDir);
    double sizeM = cfg.getDouble("tag.size_m", 0.16);
    for (auto& s : splitString(opt.ids, ',')) {
        int id = atoi(trim(s).c_str());
        cv::Mat bits = td.renderTag(id);
        if (bits.empty()) { LOG_ERROR("cannot render id %d for %s", id, family.c_str()); continue; }
        int cell = std::max(1, opt.sizePx / bits.cols);
        cv::Mat big;
        cv::resize(bits, big, cv::Size(bits.cols * cell, bits.rows * cell), 0, 0, cv::INTER_NEAREST);
        cv::Mat page(big.rows + 180, big.cols + 120, CV_8UC1, cv::Scalar(255));
        big.copyTo(page(cv::Rect(60, 60, big.cols, big.rows)));
        char label[160];
        snprintf(label, sizeof(label), "%s  ID %d  = %s", family.c_str(), id, tagMeaning(id));
        cv::putText(page, label, cv::Point(60, big.rows + 110), cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0), 2);
        snprintf(label, sizeof(label), "print so the BLACK square measures %.1f cm (tag.size_m = %.3f)", sizeM * 100 * 1.0, sizeM);
        cv::putText(page, label, cv::Point(60, big.rows + 150), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0), 1);
        std::string path = opt.outDir + "/" + family + "_id" + std::to_string(id) + ".png";
        cv::imwrite(path, page);
        LOG_INFO("wrote %s (%dx%d px, black square = %d px)", path.c_str(), page.cols, page.rows, (int)(big.cols * td.blackSquareFraction()));
    }
    LOG_INFO("Print at a scale where the black square edge = tag.size_m; measure it with a ruler and set tag.size_m in the config.");
    return 0;
}

// ---------------------------------------------------------------------------
namespace {
struct ColorPick {
    cv::Mat hsv;                 // current frame in HSV (full resolution)
    double scale = 1.0;
    int mode = 0;                // 0 = line, 1 = white strip
    std::vector<cv::Vec3b> line, white;
};
void onMouse(int event, int x, int y, int, void* ud) {
    if (event != cv::EVENT_LBUTTONDOWN) return;
    auto* p = (ColorPick*)ud;
    if (p->hsv.empty()) return;
    int fx = (int)(x / p->scale), fy = (int)(y / p->scale);
    if (fx >= p->hsv.cols) return;   // click on the mask half
    std::vector<cv::Vec3b>& dst = p->mode == 0 ? p->line : p->white;
    for (int dy = -4; dy <= 4; ++dy)
        for (int dx = -4; dx <= 4; ++dx) {
            int xx = fx + dx, yy = fy + dy;
            if (xx < 0 || yy < 0 || xx >= p->hsv.cols || yy >= p->hsv.rows) continue;
            dst.push_back(p->hsv.at<cv::Vec3b>(yy, xx));
        }
    LOG_INFO("sampled %s at (%d,%d): %zu px", p->mode == 0 ? "LINE" : "WHITE", fx, fy, dst.size());
}
// Range from samples with margins; handles hue wrap-around (red/orange) with a second range.
void rangeOf(const std::vector<cv::Vec3b>& v, cv::Scalar& lo, cv::Scalar& hi, cv::Scalar& lo2, cv::Scalar& hi2, bool& two) {
    int hLo1 = 180, hHi1 = -1, hLo2 = 180, hHi2 = -1, sLo = 255, sHi = 0, vLo = 255, vHi = 0;
    for (auto& p : v) {
        if (p[0] < 90) { hLo1 = std::min(hLo1, (int)p[0]); hHi1 = std::max(hHi1, (int)p[0]); }
        else { hLo2 = std::min(hLo2, (int)p[0]); hHi2 = std::max(hHi2, (int)p[0]); }
        sLo = std::min(sLo, (int)p[1]); sHi = std::max(sHi, (int)p[1]);
        vLo = std::min(vLo, (int)p[2]); vHi = std::max(vHi, (int)p[2]);
    }
    const int mh = 6, ms = 35, mv = 35;
    sLo = std::max(0, sLo - ms); sHi = std::min(255, sHi + ms); vLo = std::max(0, vLo - mv); vHi = std::min(255, vHi + mv);
    two = hHi1 >= 0 && hHi2 >= 0 && (hLo2 - hHi1) > 60;
    if (two) {
        lo = cv::Scalar(0, sLo, vLo); hi = cv::Scalar(std::min(179, hHi1 + mh), sHi, vHi);
        lo2 = cv::Scalar(std::max(0, hLo2 - mh), sLo, vLo); hi2 = cv::Scalar(180, sHi, vHi);
    } else {
        int a = std::min(hLo1, hLo2), b = std::max(hHi1, hHi2);
        lo = cv::Scalar(std::max(0, a - mh), sLo, vLo); hi = cv::Scalar(std::min(180, b + mh), sHi, vHi);
    }
}
}

int toolCalibColor(const Config& cfgIn, const MissionOptions& optIn) {
    Config cfg = cfgIn;
    MissionOptions opt = optIn;
    opt.dryRun = true;
    SteadyClock clock;
    std::unique_ptr<IDrone> drone;
    std::unique_ptr<IFrameSource> src;
    std::string err;
    if (!openLiveSource(cfg, opt, clock, drone, src, &err)) { LOG_ERROR("%s", err.c_str()); return 1; }
    Perception per;
    if (!per.init(cfg, &err)) { LOG_ERROR("%s", err.c_str()); return 1; }
    ColorPick pick;
    pick.scale = 0.6;
    const std::string win = "calib-color  (click LINE pixels; l=line mode, w=white mode, r=reset, s=save, q=quit)";
    cv::namedWindow(win, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(win, onMouse, &pick);
    Frame frame;
    uint64_t seq = 0;
    LOG_INFO("Click on the LINE in the video (several places, near and far). Press 'w' then click the WHITE strip. 's' saves to %s", cfg.path().c_str());
    while (true) {
        if (src->latest(frame, seq, 0.3)) seq = frame.seq;
        else if (src->ended()) break;
        if (frame.image.empty()) { if (cv::waitKey(20) == 'q') break; continue; }
        cv::cvtColor(frame.image, pick.hsv, cv::COLOR_BGR2HSV);
        cv::Scalar lo, hi, lo2, hi2;
        bool two = false;
        cv::Mat mask = cv::Mat::zeros(frame.image.size(), CV_8UC1);
        if (!pick.line.empty()) {
            rangeOf(pick.line, lo, hi, lo2, hi2, two);
            cv::inRange(pick.hsv, lo, hi, mask);
            if (two) { cv::Mat m2; cv::inRange(pick.hsv, lo2, hi2, m2); mask |= m2; }
        }
        cv::Mat wmask = cv::Mat::zeros(frame.image.size(), CV_8UC1);
        if (!pick.white.empty()) {
            cv::Scalar wlo, whi, d1, d2; bool t2;
            rangeOf(pick.white, wlo, whi, d1, d2, t2);
            cv::inRange(pick.hsv, cv::Scalar(0, 0, wlo[2]), cv::Scalar(180, whi[1], 255), wmask);
        }
        cv::Mat vis, maskC, both;
        cv::cvtColor(mask, maskC, cv::COLOR_GRAY2BGR);
        maskC.setTo(cv::Scalar(255, 255, 255), wmask & ~mask);
        maskC.setTo(cv::Scalar(0, 255, 255), mask);
        cv::hconcat(frame.image, maskC, both);
        cv::resize(both, vis, cv::Size(), pick.scale, pick.scale);
        char buf[200];
        snprintf(buf, sizeof(buf), "mode=%s  line samples=%zu  white samples=%zu", pick.mode == 0 ? "LINE" : "WHITE", pick.line.size(), pick.white.size());
        cv::putText(vis, buf, cv::Point(8, 22), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        if (!pick.line.empty()) {
            snprintf(buf, sizeof(buf), "line hsv_lo=%d,%d,%d hsv_hi=%d,%d,%d%s", (int)lo[0], (int)lo[1], (int)lo[2], (int)hi[0], (int)hi[1], (int)hi[2], two ? " (+wrap range)" : "");
            cv::putText(vis, buf, cv::Point(8, 46), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 0), 1);
        }
        cv::imshow(win, vis);
        int k = cv::waitKey(1) & 0xff;
        if (k == 'q' || k == 27) break;
        if (k == 'l') pick.mode = 0;
        if (k == 'w') pick.mode = 1;
        if (k == 'r') { (pick.mode == 0 ? pick.line : pick.white).clear(); }
        if (k == 's') {
            if (!pick.line.empty()) {
                double meanV = 0, meanS = 0;
                for (auto& p : pick.line) { meanV += p[2]; meanS += p[1]; }
                meanV /= pick.line.size(); meanS /= pick.line.size();
                if (meanV < 110 && meanS < 90) {
                    cfg.set("line.color_mode", "dark");
                    cfg.set("line.dark_v_max", std::to_string((int)hi[2]));
                    cfg.set("line.dark_s_max", std::to_string(std::min(255, (int)hi[1] + 30)));
                    LOG_INFO("line looks dark/black: color_mode=dark dark_v_max=%d dark_s_max=%d", (int)hi[2], std::min(255, (int)hi[1] + 30));
                } else {
                    cfg.set("line.color_mode", "hsv");
                    snprintf(buf, sizeof(buf), "%d, %d, %d", (int)lo[0], (int)lo[1], (int)lo[2]); cfg.set("line.hsv_lo", buf);
                    snprintf(buf, sizeof(buf), "%d, %d, %d", (int)hi[0], (int)hi[1], (int)hi[2]); cfg.set("line.hsv_hi", buf);
                    if (two) {
                        snprintf(buf, sizeof(buf), "%d, %d, %d", (int)lo2[0], (int)lo2[1], (int)lo2[2]); cfg.set("line.hsv_lo2", buf);
                        snprintf(buf, sizeof(buf), "%d, %d, %d", (int)hi2[0], (int)hi2[1], (int)hi2[2]); cfg.set("line.hsv_hi2", buf);
                    }
                    LOG_INFO("line color saved as hsv range");
                }
            }
            if (!pick.white.empty()) {
                cv::Scalar wlo, whi, d1, d2; bool t2;
                rangeOf(pick.white, wlo, whi, d1, d2, t2);
                cfg.set("line.white_s_max", std::to_string((int)whi[1]));
                cfg.set("line.white_v_min", std::to_string((int)wlo[2]));
                LOG_INFO("white strip: white_s_max=%d white_v_min=%d", (int)whi[1], (int)wlo[2]);
            }
            if (cfg.saveFile(cfg.path())) LOG_INFO("saved %s", cfg.path().c_str());
            else LOG_ERROR("could not write %s", cfg.path().c_str());
        }
    }
    cv::destroyAllWindows();
    src->stop();
    drone->disconnect();
    return 0;
}

int toolCalibGround(const Config& cfgIn, const MissionOptions& optIn) {
    Config cfg = cfgIn;
    MissionOptions opt = optIn;
    opt.dryRun = true;
    SteadyClock clock;
    std::unique_ptr<IDrone> drone;
    std::unique_ptr<IFrameSource> src;
    std::string err;
    if (!openLiveSource(cfg, opt, clock, drone, src, &err)) { LOG_ERROR("%s", err.c_str()); return 1; }
    Perception per;
    if (!per.init(cfg, &err)) { LOG_ERROR("%s", err.c_str()); return 1; }
    per.setAutoCalib(false);
    const std::string win = "calib-ground  (s=save camera pitch/roll, z=save corrected tag size, q=quit)";
    cv::namedWindow(win, cv::WINDOW_AUTOSIZE);
    LOG_INFO("Put an AprilTag flat on the floor 1-2 m in front of the camera. Hover the drone (tof gives the height)");
    LOG_INFO("or place it on a box and pass --set ground.height_source=fixed --set mission.altitude_cm=<camera height>.");
    Frame frame;
    uint64_t seq = 0;
    double lastPrint = 0;
    while (true) {
        if (src->latest(frame, seq, 0.3)) seq = frame.seq;
        else if (src->ended()) break;
        if (frame.image.empty()) { if (cv::waitKey(20) == 'q') break; continue; }
        TelloState st = drone->state();
        Observation obs = per.process(frame.image, st, clock.now(), seq);
        cv::Mat canvas = frame.image.clone();
        per.drawDebug(canvas, obs);
        const auto& c = per.calib();
        char buf[256];
        snprintf(buf, sizeof(buf), "samples=%d  camera pitch=%.1f deg roll=%.1f deg  height(tag pose)=%.2fm  tof=%.2fm  ratio=%.2f", c.samples,
                 c.pitchDeg, c.rollDeg, c.heightM, st.heightCm() > 0 ? st.heightCm() / 100.0 : -1.0, c.heightRatio);
        cv::putText(canvas, buf, cv::Point(8, 24), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 2);
        if (c.heightRatio > 0) {
            snprintf(buf, sizeof(buf), "if tof is right, true tag size = %.3f m (config has %.3f)", per.tagSizeM() / c.heightRatio, per.tagSizeM());
            cv::putText(canvas, buf, cv::Point(8, 48), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 0), 1);
        }
        cv::imshow(win, canvas);
        if (clock.now() - lastPrint > 1.0 && c.samples > 0) {
            lastPrint = clock.now();
            LOG_INFO("calib: n=%d pitch=%.2f roll=%.2f height(tag)=%.3f ratio=%.3f", c.samples, c.pitchDeg, c.rollDeg, c.heightM, c.heightRatio);
        }
        int k = cv::waitKey(1) & 0xff;
        if (k == 'q' || k == 27) break;
        if (k == 's' && c.samples >= 5) {
            snprintf(buf, sizeof(buf), "%.2f", c.pitchDeg); cfg.set("camera.pitch_deg", buf);
            snprintf(buf, sizeof(buf), "%.2f", c.rollDeg); cfg.set("camera.roll_deg", buf);
            if (cfg.saveFile(cfg.path())) LOG_INFO("saved camera.pitch_deg=%.2f camera.roll_deg=%.2f to %s", c.pitchDeg, c.rollDeg, cfg.path().c_str());
        }
        if (k == 'z' && c.heightRatio > 0) {
            snprintf(buf, sizeof(buf), "%.4f", per.tagSizeM() / c.heightRatio);
            cfg.set("tag.size_m", buf);
            if (cfg.saveFile(cfg.path())) LOG_INFO("saved tag.size_m=%s", buf);
        }
    }
    cv::destroyAllWindows();
    src->stop();
    drone->disconnect();
    return 0;
}

int toolTelemetry(const Config& cfg, const MissionOptions& optIn) {
    MissionOptions opt = optIn;
    opt.dryRun = true;
    SteadyClock clock;
    std::unique_ptr<IDrone> drone;
    std::unique_ptr<IFrameSource> src;
    std::string err;
    if (!openLiveSource(cfg, opt, clock, drone, src, &err)) { LOG_ERROR("%s", err.c_str()); return 1; }
    LOG_INFO("connected to %s; battery? -> %d", drone->name().c_str(), drone->queryInt("battery?"));
    double t0 = clock.now(), lastPrint = 0;
    int frames = 0;
    uint64_t seq = 0;
    Frame frame;
    cv::Size size;
    while (clock.now() - t0 < opt.durationS) {
        if (src->latest(frame, seq, 0.2)) { seq = frame.seq; ++frames; size = frame.image.size(); }
        if (clock.now() - lastPrint >= 0.5) {
            lastPrint = clock.now();
            TelloState st = drone->state();
            LOG_INFO("state: %s | video frames=%d (%dx%d) fps=%.1f", st.valid ? st.summary().c_str() : "(none yet)", frames, size.width,
                     size.height, src->fps());
        }
    }
    TelloState st = drone->state();
    LOG_INFO("---- summary after %.0fs ----", opt.durationS);
    LOG_INFO("telemetry: %s", st.valid ? "OK" : "NONE (check firewall for UDP 8890)");
    LOG_INFO("video: %d frames, %.1f fps %s", frames, frames / opt.durationS, frames == 0 ? "(NONE: check firewall for UDP 11111)" : "");
    src->stop();
    drone->disconnect();
    return (st.valid && frames > 0) ? 0 : 2;
}
