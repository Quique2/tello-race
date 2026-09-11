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
        "  --set sec.key=value    override a configuration value (repeatable)\n"
        "  --source <spec>        udp (default) | file:<video> | camera:<index>\n"
        "  --dry-run              fly: connect for video/telemetry but never send flight commands\n"
        "  --start-at-tag <1-3>   resume from a checkpoint: the drone stands on that tag\n"
        "  --auto-start           take off immediately (sim default)\n"
        "  --record               save video + telemetry CSV under logs/\n"
        "  --headless             no windows (sim batch runs)\n"
        "  --no-realtime          sim/replay as fast as possible\n"
        "  --track <random|example>  simulator track kind\n"
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
        else if (a == "--source") opt.source = next("--source");
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
    for (auto& o : opt.overrides)
        if (!cfg.setFromArg(o, &err)) { fprintf(stderr, "%s\n", err.c_str()); return 1; }
    LOG_INFO("tello_race: command=%s config=%s", cmd.c_str(), opt.configPath.c_str());

    if (cmd == "fly") return runFly(cfg, opt);
    if (cmd == "replay") {
        if (opt.source == "udp") { fprintf(stderr, "replay needs --source file:<video>\n"); return 1; }
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
