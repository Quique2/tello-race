#include "app/WebServer.h"

#include <chrono>
#include <cstring>

#include <opencv2/imgcodecs.hpp>

#include "civetweb.h"
#include "util/Log.h"

namespace {
double wallNow() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
int reqTrampoline(mg_connection* c, void* ud) { return static_cast<WebServer*>(ud)->onRequest(c); }
int wsConnectTrampoline(const mg_connection* c, void* ud) { return static_cast<WebServer*>(ud)->onWsConnect(c); }
void wsReadyTrampoline(mg_connection* c, void* ud) { static_cast<WebServer*>(ud)->onWsReady(c); }
int wsDataTrampoline(mg_connection* c, int bits, char* data, size_t len, void* ud) { return static_cast<WebServer*>(ud)->onWsData(c, bits, data, len); }
void wsCloseTrampoline(const mg_connection* c, void* ud) { static_cast<WebServer*>(ud)->onWsClose(c); }
std::string queryVar(const mg_connection* c, const char* name) {
    const mg_request_info* ri = mg_get_request_info(c);
    if (!ri || !ri->query_string) return "";
    char buf[256] = {0};
    int n = mg_get_var(ri->query_string, strlen(ri->query_string), name, buf, sizeof(buf));
    return n > 0 ? std::string(buf, n) : "";
}
}

WebServer::~WebServer() { stop(); }

bool WebServer::start(const Options& o, std::string* err) {
    opt_ = o;
    mg_init_library(0);
    std::string port = std::to_string(o.port);
    const char* options[] = {"listening_ports", port.c_str(), "num_threads", "16", "enable_directory_listing", "no",
                             "request_timeout_ms", "2000", "websocket_timeout_ms", "3600000", "enable_keep_alive", "yes",   // 2 s: a vanished browser must not stall the control loop's broadcasts
                             "tcp_nodelay", "1", nullptr};
    mg_callbacks cb;
    memset(&cb, 0, sizeof(cb));
    ctx_ = mg_start(&cb, this, options);
    if (!ctx_) { if (err) *err = "cannot start the web server on port " + port + " (port in use?)"; return false; }
    mg_set_request_handler(ctx_, "/", reqTrampoline, this);
    mg_set_websocket_handler(ctx_, "/ws", wsConnectTrampoline, wsReadyTrampoline, wsDataTrampoline, wsCloseTrampoline, this);
    LOG_INFO("Web interface: http://localhost:%d/  (%s)", o.port, o.token.empty() ? "no token: LAN use only" : "operator token required");
    return true;
}

void WebServer::stop() {
    if (!ctx_) return;
    frameCv_.notify_all();
    mg_stop(ctx_);
    ctx_ = nullptr;
    std::lock_guard<std::mutex> lk(wsMtx_);
    ws_.clear();
}

void WebServer::publishFrame(const cv::Mat& bgr) {
    if (!ctx_ || bgr.empty() || streamClients_ == 0) return;
    double now = wallNow();
    if (now - lastEncode_ < 1.0 / std::max(1, opt_.fps)) return;
    lastEncode_ = now;
    std::vector<unsigned char> buf;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, opt_.jpegQuality};
    if (!cv::imencode(".jpg", bgr, buf, params)) return;
    {
        std::lock_guard<std::mutex> lk(frameMtx_);
        jpeg_.swap(buf);
        ++frameSeq_;
    }
    frameCv_.notify_all();
}

void WebServer::publishState(const std::string& json) {
    if (!ctx_) return;
    std::lock_guard<std::mutex> lk(wsMtx_);
    lastState_ = json;
    for (auto it = ws_.begin(); it != ws_.end();) {
        int n = mg_websocket_write(it->first, MG_WEBSOCKET_OPCODE_TEXT, json.data(), json.size());
        if (n <= 0) it = ws_.erase(it);   // gone (browser closed or reloaded): never write to it again
        else ++it;
    }
}

bool WebServer::popCommand(WebCommand& out) {
    std::lock_guard<std::mutex> lk(cmdMtx_);
    if (cmds_.empty()) return false;
    out = cmds_.front();
    cmds_.pop_front();
    return true;
}

int WebServer::wsClients() {
    std::lock_guard<std::mutex> lk(wsMtx_);
    return (int)ws_.size();
}

WebServer::Role WebServer::roleFor(const mg_connection* c) const {
    if (opt_.token.empty()) return OPERATOR;
    std::string t = queryVar(c, "token");
    if (t == opt_.token) return OPERATOR;
    if (!opt_.viewerToken.empty() && t == opt_.viewerToken) return VIEWER;
    return NONE;
}

int WebServer::onRequest(mg_connection* c) {
    const mg_request_info* ri = mg_get_request_info(c);
    std::string uri = (ri && ri->local_uri) ? ri->local_uri : "/";
    if (uri == "/" || uri == "/index.html") return serveIndex(c);
    if (uri == "/stream.mjpg") return serveStream(c);
    if (uri == "/api/status") return serveStatus(c);
    if (uri == "/sim3d" || uri == "/sim3d.html") return serveBlob(c, "text/html; charset=utf-8", sim3dPageHtml(), strlen(sim3dPageHtml()), false);
    if (uri == "/sim/scene.json" || uri == "/sim/world.png") {
        std::string js; std::vector<unsigned char> png;
        { std::lock_guard<std::mutex> lk(wsMtx_); js = sceneJson_; png = worldPng_; }
        if (js.empty()) { mg_send_http_error(c, 404, "no simulation scene"); return 404; }
        if (uri == "/sim/scene.json") return serveBlob(c, "application/json", js.data(), js.size(), true);
        return serveBlob(c, "image/png", (const char*)png.data(), png.size(), true);
    }
    mg_send_http_error(c, 404, "not found");
    return 404;
}

int WebServer::serveIndex(mg_connection* c) {
    const char* html = webPageHtml();
    size_t len = strlen(html);
    mg_printf(c, "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nCache-Control: no-cache\r\nContent-Length: %zu\r\nConnection: close\r\n\r\n", len);
    mg_write(c, html, len);
    return 200;
}

// one-shot response with a body; needToken = a token query parameter is required (viewer or operator)
int WebServer::serveBlob(mg_connection* c, const char* contentType, const char* data, size_t len, bool needToken) {
    if (needToken && roleFor(c) == NONE) { mg_send_http_error(c, 401, "token required"); return 401; }
    mg_printf(c, "HTTP/1.1 200 OK\r\nContent-Type: %s\r\nCache-Control: no-cache\r\nContent-Length: %zu\r\nConnection: close\r\n\r\n", contentType, len);
    mg_write(c, data, len);
    return 200;
}

void WebServer::setSimScene(const std::string& sceneJson, const std::vector<unsigned char>& worldPng) {
    std::lock_guard<std::mutex> lk(wsMtx_);
    sceneJson_ = sceneJson; worldPng_ = worldPng;
}

int WebServer::serveStatus(mg_connection* c) {
    if (roleFor(c) == NONE) { mg_send_http_error(c, 401, "token required"); return 401; }
    std::string s;
    { std::lock_guard<std::mutex> lk(wsMtx_); s = lastState_.empty() ? "{}" : lastState_; }
    mg_printf(c, "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-cache\r\nContent-Length: %zu\r\nConnection: close\r\n\r\n", s.size());
    mg_write(c, s.data(), s.size());
    return 200;
}

// MJPEG over HTTP: one long-lived response per browser, a new JPEG part whenever a frame is published.
int WebServer::serveStream(mg_connection* c) {
    if (roleFor(c) == NONE) { mg_send_http_error(c, 401, "token required"); return 401; }
    if (mg_printf(c, "HTTP/1.1 200 OK\r\nCache-Control: no-cache, no-store\r\nPragma: no-cache\r\nConnection: close\r\n"
                     "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n") <= 0) return 200;
    ++streamClients_;
    uint64_t seen = 0;
    std::vector<unsigned char> buf;
    while (ctx_) {
        bool fresh;
        {
            std::unique_lock<std::mutex> lk(frameMtx_);
            fresh = frameCv_.wait_for(lk, std::chrono::seconds(2), [&] { return frameSeq_ > seen || !ctx_; });
            if (!ctx_) break;
            if (fresh) { seen = frameSeq_; buf = jpeg_; }
        }
        if (!fresh) { if (mg_write(c, "\r\n", 2) <= 0) break; continue; }   // idle keep-alive, detects a gone client
        if (mg_printf(c, "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n", buf.size()) <= 0) break;
        if (mg_write(c, buf.data(), buf.size()) <= 0) break;
        if (mg_write(c, "\r\n", 2) <= 0) break;
    }
    --streamClients_;
    return 200;
}

int WebServer::onWsConnect(const mg_connection* c) { return roleFor(c) == NONE ? 1 : 0; }   // non-zero rejects

void WebServer::onWsReady(mg_connection* c) {
    Role r = roleFor(c);
    std::string state;
    {
        std::lock_guard<std::mutex> lk(wsMtx_);
        ws_[c] = r;
        state = lastState_;
    }
    std::string hello = std::string("{\"hello\":true,\"role\":\"") + (r == OPERATOR ? "operator" : "viewer") + "\"}";
    mg_websocket_write(c, MG_WEBSOCKET_OPCODE_TEXT, hello.data(), hello.size());
    if (!state.empty()) mg_websocket_write(c, MG_WEBSOCKET_OPCODE_TEXT, state.data(), state.size());
    LOG_INFO("web: client connected (%s)", r == OPERATOR ? "operator" : "viewer");
}

int WebServer::onWsData(mg_connection* c, int bits, char* data, size_t len) {
    int op = bits & 0x0f;
    if (op == 0x8) return 0;      // close frame
    if (op != 0x1) return 1;      // only text messages carry commands
    std::string msg(data, len);
    auto field = [&](const char* key) -> std::string {   // tolerant of whitespace: "key" : "value"
        std::string k = std::string("\"") + key + "\"";
        size_t p = msg.find(k);
        if (p == std::string::npos) return "";
        p = msg.find(':', p + k.size());
        if (p == std::string::npos) return "";
        p = msg.find('"', p + 1);
        if (p == std::string::npos) return "";
        size_t e = msg.find('"', p + 1);
        return e == std::string::npos ? "" : msg.substr(p + 1, e - p - 1);
    };
    std::string cmd = field("cmd"), arg = field("arg");
    if (cmd.empty()) return 1;
    Role r = NONE;
    {
        std::lock_guard<std::mutex> lk(wsMtx_);
        auto it = ws_.find(c);
        if (it != ws_.end()) r = it->second;
    }
    if (r != OPERATOR) {
        const char* deny = "{\"error\":\"operator token required\"}";
        mg_websocket_write(c, MG_WEBSOCKET_OPCODE_TEXT, deny, strlen(deny));
        return 1;
    }
    std::lock_guard<std::mutex> lk(cmdMtx_);
    cmds_.push_back({cmd, arg});
    return 1;
}

void WebServer::onWsClose(const mg_connection* c) {
    std::lock_guard<std::mutex> lk(wsMtx_);
    ws_.erase(const_cast<mg_connection*>(c));
}

std::string stateToJson(const StateSnapshot& s) {
    auto esc = [](const std::string& in) {
        std::string o;
        for (char ch : in) {
            if (ch == '"') o += '\'';
            else if (ch == '\n' || ch == '\r') o += ' ';
            else if (ch == (char)92) o += '/';
            else o += ch;
        }
        return o;
    };
    std::string ev;
    for (size_t i = s.events.size() > 6 ? s.events.size() - 6 : 0; i < s.events.size(); ++i)
        ev += (ev.empty() ? "\"" : ",\"") + esc(s.events[i]) + "\"";
    char simBuf[256] = "";
    if (s.sim) snprintf(simBuf, sizeof(simBuf), ",\"sim\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f,\"yaw\":%.1f,\"camPitch\":%.1f}", s.simX, s.simY, s.simZ, s.simYaw, s.camPitch);
    char buf[2560];
    snprintf(buf, sizeof(buf),
             "{\"phase\":\"%s\",\"seg\":%d,\"t\":%.1f,\"telemetry\":%s,\"bat\":%d,\"tof\":%d,\"h\":%d,\"yaw\":%d,\"pitch\":%d,"
             "\"roll\":%d,\"templ\":%d,\"temph\":%d,\"fps\":%.1f,\"camera\":\"%s\",\"recording\":%s,\"drone\":\"%s\","
             "\"mode\":\"%s\",\"flying\":%s,\"tag\":%d,\"tagX\":%.2f,\"mirror\":\"%s\",\"noticeId\":%d,\"notice\":\"%s\",\"wifi\":%d,\"rcAge\":%.1f,\"stale\":%s,\"rc\":[%d,%d,%d,%d],\"events\":[%s]%s}",
             s.phase, s.seg, s.t, s.telemetry ? "true" : "false", s.bat, s.tof, s.h, s.yaw, s.pitch, s.roll, s.templ, s.temph, s.fps,
             s.belly ? "belly" : "front", s.recording ? "true" : "false", s.drone, s.manual ? "manual" : "mission", s.flying ? "true" : "false", s.tagId, s.tagX, s.mirror.c_str(), s.noticeId, s.notice.c_str(), s.wifi, s.rcAge, s.stale ? "true" : "false", s.rc[0], s.rc[1], s.rc[2], s.rc[3],
             ev.c_str(), simBuf);
    return buf;
}
