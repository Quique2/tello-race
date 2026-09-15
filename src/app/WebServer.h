#pragma once
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

struct mg_context;
struct mg_connection;

struct WebCommand {
    std::string cmd;    // start | land | emergency | record | camera | tag
    std::string arg;
};

// Ground-station web interface: one page (embedded HTML), an MJPEG stream of the annotated camera view,
// and a WebSocket carrying telemetry/mission state to browsers and commands back. Token protected:
// the operator token allows commands, the viewer token only watching.
class WebServer {
public:
    struct Options {
        int port = 8080;
        std::string token;          // operator token ("" = no authentication, LAN use only)
        std::string viewerToken;    // optional watch-only token
        int fps = 15;
        int jpegQuality = 70;
    };
    ~WebServer();
    bool start(const Options& o, std::string* err);
    void stop();
    bool running() const { return ctx_ != nullptr; }
    void publishFrame(const cv::Mat& bgr);            // throttled to Options::fps, JPEG encoded once per frame
    void publishState(const std::string& json);       // broadcast to all websocket clients
    bool popCommand(WebCommand& out);
    int streamClients() const { return streamClients_; }
    int wsClients();

    // civetweb callbacks (public for the C trampolines)
    int onRequest(mg_connection* c);
    int onWsConnect(const mg_connection* c);
    void onWsReady(mg_connection* c);
    int onWsData(mg_connection* c, int bits, char* data, size_t len);
    void onWsClose(const mg_connection* c);

private:
    enum Role { NONE, VIEWER, OPERATOR };
    Role roleFor(const mg_connection* c) const;   // from the "token" query parameter
    int serveIndex(mg_connection* c);
    int serveStream(mg_connection* c);
    int serveStatus(mg_connection* c);

    mg_context* ctx_ = nullptr;
    Options opt_;
    std::mutex frameMtx_;
    std::condition_variable frameCv_;
    std::vector<unsigned char> jpeg_;
    uint64_t frameSeq_ = 0;
    double lastEncode_ = 0;
    std::mutex wsMtx_;
    std::map<mg_connection*, Role> ws_;
    std::string lastState_;
    std::mutex cmdMtx_;
    std::deque<WebCommand> cmds_;
    std::atomic<int> streamClients_{0};
};

// HTML page served at "/" (src/app/WebPage.cpp)
const char* webPageHtml();

// Telemetry + mission snapshot serialized for the page (built here so no JSON code lives in the mission loop)
struct StateSnapshot {
    const char* phase = "IDLE";
    int seg = 0;
    double t = 0;
    bool telemetry = false;
    int bat = -1, tof = 0, h = 0, yaw = 0, pitch = 0, roll = 0, templ = 0, temph = 0;
    double fps = 0;
    bool belly = false, recording = false, manual = false, flying = false;
    const char* drone = "";
    int tagId = 0;
    double tagX = 0;
    int rc[4] = {0, 0, 0, 0};
    double rcAge = -1;     // s since the page last sent sticks (manual mode), -1 otherwise
    bool stale = false;    // dead-man has released the sticks
    int wifi = 0;          // drone link signal, dBm (0 = unknown)
    std::string notice;    // one-line message for the page, shown once per noticeId
    std::string mirror = "none";   // camera.flip in effect
    int noticeId = 0;
    std::vector<std::string> events;
};
std::string stateToJson(const StateSnapshot& s);
