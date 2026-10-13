// tello_race - autonomous line + AprilTag navigation for the DJI Tello competition (MR3003C)
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "app/Mission.h"
#include "app/Tools.h"
#include "util/Config.h"
#include "util/Log.h"

static void usage() {
    printf(
        "tello_race <command> [options]\n\n"
        "commands:\n"
        "  fly            connect to the Tello and run the mission (press 't' to take off)\n"
        "  sim            run the mission in the built-in simulator\n"
        "  replay         run perception + navigator logic on a recorded video (no drone)\n"
        "  telemetry      connect, print telemetry/battery and test the video stream\n"
        "  calib-color    interactive HSV picker for the line / white strip colors\n"
        "  calib-ground   measure camera pitch/roll/height from a floor AprilTag\n"
        "  gen-tags       write printable AprilTag PNGs (IDs 1-4 by default)\n\n"
        "options:\n"
        "  --config <file>        configuration file (default config/default.ini)\n"
        "  --profile <file>       lay another .ini over the configuration, e.g. config/course_tec.ini (repeatable)\n"
        "  --set sec.key=value    override a configuration value (repeatable)\n"
        "  --source <spec>        udp (default) | file:<video> | camera:<index>\n"
        "  --telemetry <csv>      replay: the .csv recorded with that video; the flight is then replayed as it was flown\n"
        "                         (its frames, telemetry and timing; give the settings it started with via --set;\n"
        "                         --set replay.jpeg_quality=90 plays the frames through one more JPEG generation)\n"
        "  --dry-run              fly: connect for video/telemetry but never send flight commands\n"
        "  --start-at-tag <1-3>   resume from a checkpoint: the drone stands on that tag\n"
        "  --auto-start           take off immediately (sim default)\n"
        "  --record               save video + telemetry CSV under logs/\n"
        "  --headless             no windows (sim batch runs)\n"
        "  --no-realtime          sim/replay as fast as possible\n"
        "  --track <random|example|folder>  simulator track: a generated one, or a course folder made from a photo\n"
        "  --episodes <n> --seed <n>  simulator batch settings\n"
        "  --ui-scale <f>         window scale factor (default 1.0)\n"
        "  --ids 1,2,3,4 --size-px 600 --out <dir>   gen-tags settings\n"
        "  --duration <s>         telemetry/calibration duration\n"
        "  --log-level debug|info|warn\n");
}

static bool findConfig(std::string& path, const char* argv0) {
    FILE* f = fopen(path.c_str(), "r");
    if (f) { fclose(f); return true; }
    std::string exe = argv0 ? argv0 : "";
    size_t slash = exe.find_last_of("/\\");
    std::string dir = slash == std::string::npos ? "." : exe.substr(0, slash);
    std::string alt = dir + "/" + path;
    f = fopen(alt.c_str(), "r");
    if (f) { fclose(f); path = alt; return true; }
    alt = dir + "/../" + path;
    f = fopen(alt.c_str(), "r");
    if (f) { fclose(f); path = alt; return true; }
    return false;
}

int main(int argc, char** argv) {
    if (argc < 2) { usage(); return 1; }
    std::string cmd = argv[1];
    if (cmd == "help" || cmd == "--help" || cmd == "-h") { usage(); return 0; }
    MissionOptions opt;
    for (int i = 2; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&](const char* name) -> std::string {
            if (i + 1 >= argc) { fprintf(stderr, "missing value for %s\n", name); exit(1); }
            return argv[++i];
        };
        if (a == "--config") opt.configPath = next("--config");
        else if (a == "--set") opt.overrides.push_back(next("--set"));
        else if (a == "--profile") opt.profiles.push_back(next("--profile"));
        else if (a == "--source") opt.source = next("--source");
        else if (a == "--telemetry") opt.telemetry = next("--telemetry");
        else if (a == "--dry-run") opt.dryRun = true;
        else if (a == "--start-at-tag") opt.startAtTag = atoi(next("--start-at-tag").c_str());
        else if (a == "--auto-start") opt.autoStart = true;
        else if (a == "--record") opt.record = true;
        else if (a == "--headless") opt.headless = true;
        else if (a == "--no-realtime") opt.realtime = false;
        else if (a == "--track") opt.track = next("--track");
        else if (a == "--episodes") opt.episodes = atoi(next("--episodes").c_str());
        else if (a == "--seed") opt.seed = strtoull(next("--seed").c_str(), nullptr, 10);
        else if (a == "--ui-scale") opt.uiScale = atof(next("--ui-scale").c_str());
        else if (a == "--dump-every") opt.dumpEvery = atoi(next("--dump-every").c_str());
        else if (a == "--dump-dir") opt.dumpDir = next("--dump-dir");
        else if (a == "--ids") opt.ids = next("--ids");
        else if (a == "--size-px") opt.sizePx = atoi(next("--size-px").c_str());
        else if (a == "--out") opt.outDir = next("--out");
        else if (a == "--duration") opt.durationS = atof(next("--duration").c_str());
        else if (a == "--log-dir") opt.logDir = next("--log-dir");
        else if (a == "--log-level") {
            std::string l = next("--log-level");
            logx::setLevel(l == "debug" ? logx::DEBUG : l == "warn" ? logx::WARN : logx::INFO);
        } else { fprintf(stderr, "unknown option %s\n", a.c_str()); usage(); return 1; }
    }
    Config cfg;
    std::string err;
    if (!findConfig(opt.configPath, argv[0])) {
        fprintf(stderr, "config file not found: %s\n", opt.configPath.c_str());
        return 1;
    }
    if (!cfg.loadFile(opt.configPath, &err)) { fprintf(stderr, "%s\n", err.c_str()); return 1; }
    for (auto p : opt.profiles) {          // found like the main file: as given, next to the executable, or one level up
        if (!findConfig(p, argv[0])) { fprintf(stderr, "profile not found: %s\n", p.c_str()); return 1; }
        Config layer;                      // merged key by key: the main file stays the one the calibration tools rewrite
        if (!layer.loadFile(p, &err)) { fprintf(stderr, "%s\n", err.c_str()); return 1; }
        for (const auto& kv : layer.all()) cfg.set(kv.first, kv.second);
        LOG_INFO("profile: %s", p.c_str());
    }
    // What earlier flights measured (mirror angle, lean signs, stick gain) is laid over the configuration; --set still
    // wins. The simulator leaves that file alone unless sim.use_learned is set.
    std::string learnedPath;
    {
        Config probe = cfg;                // which file, and whether to use it, may themselves come from --set
        for (auto& o : opt.overrides)
            if (!probe.setFromArg(o, &err)) { fprintf(stderr, "%s\n", err.c_str()); return 1; }
        std::string learned = probe.getString("ground.learned_file", "");
        if (!opt.telemetry.empty()) learned.clear();   // a recorded flight is replayed with what IT started with (--set), not with what is known today
        const bool use = !learned.empty() && (cmd != "sim" || probe.getBool("sim.use_learned", false));
        if (use) {
            if (learned.find_first_of("/\\") == std::string::npos) {     // a bare name lives next to the main configuration file
                size_t slash = opt.configPath.find_last_of("/\\");
                learned = (slash == std::string::npos ? std::string(".") : opt.configPath.substr(0, slash)) + "/" + learned;
            }
            Config layer;
            if (layer.loadFile(learned, nullptr)) {
                for (const auto& kv : layer.all()) cfg.set(kv.first, kv.second);
                LOG_INFO("learned in earlier flights (%s): camera pitch %s roll %s deg, stick gain %s m/s per unit", learned.c_str(),
                         cfg.getString("camera.pitch_deg", "?").c_str(), cfg.getString("camera.roll_deg", "?").c_str(),
                         cfg.getString("course.rc_gain_ms", "?").c_str());
            }
        }
        learnedPath = use ? learned : "";
    }
    for (auto& o : opt.overrides)
        if (!cfg.setFromArg(o, &err)) { fprintf(stderr, "%s\n", err.c_str()); return 1; }
    cfg.set("ground.learned_file", learnedPath);        // resolved (or switched off): this is where it is written back
    LOG_INFO("tello_race: command=%s config=%s", cmd.c_str(), opt.configPath.c_str());

    if (cmd == "fly") return runFly(cfg, opt);
    if (cmd == "replay") {
        if (opt.source == "udp") { fprintf(stderr, "replay needs --source file:<video>\n"); return 1; }
        if (!opt.telemetry.empty()) return runReplay(cfg, opt);
        opt.dryRun = true;
        opt.autoStart = true;
        return runFly(cfg, opt);
    }
    if (cmd == "sim") { opt.sim = true; opt.autoStart = true; return runSim(cfg, opt); }
    if (cmd == "telemetry") return toolTelemetry(cfg, opt);
    if (cmd == "calib-color") return toolCalibColor(cfg, opt);
    if (cmd == "calib-ground") return toolCalibGround(cfg, opt);
    if (cmd == "gen-tags") return toolGenTags(cfg, opt);
    fprintf(stderr, "unknown command %s\n", cmd.c_str());
    usage();
    return 1;
}
