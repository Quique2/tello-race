#pragma once
#include <memory>
#include <string>
#include <vector>

#include "tello/IDrone.h"
#include "util/Clock.h"
#include "util/Config.h"
#include "vision/FrameSource.h"

struct MissionOptions {
    std::string configPath = "config/default.ini";
    std::vector<std::string> overrides;      // section.key=value
    std::vector<std::string> profiles;       // extra .ini files laid over the main one (a course, a mirror, ...)
    std::string source = "udp";              // udp | file:<path> | camera:<n>
    bool sim = false;
    std::string track = "random";            // random | example
    int episodes = 1;
    uint64_t seed = 1;
    bool headless = false;
    bool realtime = true;
    bool record = false;
    std::string logDir = "logs";
    int startAtTag = 0;
    bool autoStart = false;
    bool dryRun = false;
    double uiScale = 1.0;
    int dumpEvery = 0;                       // write the HUD canvas every N frames (debugging without a window)
    std::string dumpDir = "logs/frames";
    // tools
    std::string outDir = "tags";
    std::string ids = "1,2,3,4";
    int sizePx = 600;
    double durationS = 10;
};

// Opens the drone + video source described by the options (real Tello, replay file or webcam).
bool openLiveSource(const Config& cfg, const MissionOptions& opt, IClock& clock, std::unique_ptr<IDrone>& drone,
                    std::unique_ptr<IFrameSource>& src, std::string* err);

int runFly(const Config& cfg, const MissionOptions& opt);   // real drone / replay / dry run
int runSim(const Config& cfg, const MissionOptions& opt);   // simulator episodes
