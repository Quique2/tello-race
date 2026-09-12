#include "tello/TelloDrone.h"

#include <chrono>
#include <cstring>

#include "util/Log.h"
#include "util/MathUtil.h"

TelloDrone::TelloDrone(const Config& cfg, IClock& clock)
    : clock_(clock),
      ip_(cfg.getString("tello.ip", "192.168.10.1")),
      cmdPort_((uint16_t)cfg.getInt("tello.cmd_port", 8889)),
      statePort_((uint16_t)cfg.getInt("tello.state_port", 8890)),
      localPort_((uint16_t)cfg.getInt("tello.local_port", 9000)),
      videoUrl_(cfg.getString("tello.video_url", "udp://0.0.0.0:11111")),
      rcPeriodS_(cfg.getInt("tello.rc_period_ms", 50) / 1000.0),
      rcStaleS_(cfg.getDouble("tello.rc_stale_s", 0.5)) {}

TelloDrone::~TelloDrone() { disconnect(); }

bool TelloDrone::connect(std::string* err) {
    if (!cmdSock_.open(localPort_, err)) return false;
    if (!stateSock_.open(statePort_, err)) return false;
    running_ = true;
    replyThread_ = std::thread(&TelloDrone::replyLoop, this);
    stateThread_ = std::thread(&TelloDrone::stateLoop, this);

    bool ok = false;
    std::string reply;
    for (int attempt = 1; attempt <= 4 && !ok; ++attempt) {
        LOG_INFO("Tello: entering SDK mode (attempt %d)...", attempt);
        ok = command("command", &reply, 3.0);
    }
    if (!ok) {
        if (err) *err = "no answer from the Tello at " + ip_ + " (connected to the TELLO-xxxx Wi-Fi? firewall allowed?)";
        disconnect();
        return false;
    }
    rcThread_ = std::thread(&TelloDrone::rcLoop, this);
    int bat = queryInt("battery?");
    std::string sdk;
    command("sdk?", &sdk, 2.0);
    LOG_INFO("Tello connected: battery=%d%% sdk=%s", bat, sdk.c_str());
    return true;
}

bool TelloDrone::setDownvision(bool on) {
    std::string reply;
    bool ok = command(on ? "downvision 1" : "downvision 0", &reply, 3.0);
    if (ok) downvision_ = on;
    else LOG_WARN("downvision not accepted (%s): needs a Tello EDU / Talent with SDK 3.0 firmware", reply.c_str());
    return ok;
}

void TelloDrone::disconnect() {
    if (!running_) return;
    if (downvision_) { std::string r; command("downvision 0", &r, 2.0); downvision_ = false; }   // leave the front camera on
    running_ = false;
    replyCv_.notify_all();
    if (rcThread_.joinable()) rcThread_.join();
    if (replyThread_.joinable()) replyThread_.join();
    if (stateThread_.joinable()) stateThread_.join();
    cmdSock_.close();
    stateSock_.close();
}

static bool isFlightCommand(const std::string& cmd) {
    static const char* fc[] = {"takeoff", "land", "emergency", "up", "down", "left", "right", "forward",
                               "back", "cw", "ccw", "flip", "go", "curve", "stop", "speed"};
    std::string op = cmd.substr(0, cmd.find(' '));
    for (const char* f : fc) if (op == f) return true;
    return false;
}

bool TelloDrone::command(const std::string& cmd, std::string* reply, double timeoutS) {
    if (dryRun_ && isFlightCommand(cmd)) {
        LOG_INFO("[dry-run] would send: %s", cmd.c_str());
        if (reply) *reply = "ok";
        lastReply = "ok";
        return true;
    }
    std::lock_guard<std::mutex> cmdLock(cmdMtx_);
    inCommand_ = true;
    uint64_t seqBefore;
    {
        std::lock_guard<std::mutex> lk(replyMtx_);
        seqBefore = replySeq_;
    }
    LOG_DEBUG("Tello >> %s", cmd.c_str());
    if (!cmdSock_.sendTo(ip_, cmdPort_, cmd.data(), cmd.size())) {
        inCommand_ = false;
        if (reply) *reply = "send failed";
        return false;
    }
    std::string text;
    bool got = false;
    {
        std::unique_lock<std::mutex> lk(replyMtx_);
        got = replyCv_.wait_for(lk, std::chrono::duration<double>(timeoutS),
                                [&] { return replySeq_ > seqBefore || !running_; });
        if (got && replySeq_ > seqBefore) text = lastReplyText_;
        else got = false;
    }
    inCommand_ = false;
    if (!got) {
        LOG_WARN("Tello: timeout (%.1fs) waiting for reply to '%s'", timeoutS, cmd.c_str());
        if (reply) *reply = "timeout";
        return false;
    }
    LOG_DEBUG("Tello << %s", text.c_str());
    if (reply) *reply = text;
    lastReply = text;
    bool isQuery = !cmd.empty() && cmd.back() == '?';
    bool ok = text.rfind("ok", 0) == 0 || (isQuery && text.rfind("error", 0) != 0);
    if (ok) {
        if (cmd == "takeoff") flying_ = true;
        if (cmd == "land" || cmd == "emergency") flying_ = false;
    } else {
        LOG_WARN("Tello: command '%s' answered '%s'", cmd.c_str(), text.c_str());
        if (cmd == "emergency") flying_ = false;
    }
    return ok;
}

void TelloDrone::replyLoop() {
    char buf[1500];
    while (running_) {
        int n = cmdSock_.recvFrom(buf, sizeof(buf) - 1, 200);
        if (n <= 0) continue;
        buf[n] = 0;
        std::string s(buf, n);
        while (!s.empty() && (s.back() == '\r' || s.back() == '\n' || s.back() == ' ' || s.back() == 0)) s.pop_back();
        std::lock_guard<std::mutex> lk(replyMtx_);
        lastReplyText_ = s;
        ++replySeq_;
        replyCv_.notify_all();
    }
}

void TelloDrone::stateLoop() {
    char buf[2048];
    while (running_) {
        int n = stateSock_.recvFrom(buf, sizeof(buf) - 1, 200);
        if (n <= 0) continue;
        buf[n] = 0;
        TelloState st;
        if (TelloState::parse(std::string(buf, n), st)) {
            st.stamp = clock_.now();
            std::lock_guard<std::mutex> lk(stateMtx_);
            state_ = st;
        }
    }
}

void TelloDrone::sendRcNow(int lr, int fb, int ud, int yaw) {
    char buf[64];
    snprintf(buf, sizeof(buf), "rc %d %d %d %d", mu::clamp(lr, -100, 100), mu::clamp(fb, -100, 100),
             mu::clamp(ud, -100, 100), mu::clamp(yaw, -100, 100));
    cmdSock_.sendTo(ip_, cmdPort_, buf, strlen(buf));
    rcSentStamp_ = clock_.now();
}

void TelloDrone::rc(int lr, int fb, int ud, int yaw) {
    std::lock_guard<std::mutex> lk(rcMtx_);
    rc_[0] = lr; rc_[1] = fb; rc_[2] = ud; rc_[3] = yaw;
    rcStamp_ = clock_.now();
    if (dryRun_) return;
    if (!inCommand_ && flying_) sendRcNow(lr, fb, ud, yaw);
}

// Keep-alive / watchdog: re-sends the latest sticks periodically; if the controller stops updating them
// (vision loop stalled), sends neutral sticks so the drone hovers instead of flying away.
void TelloDrone::rcLoop() {
    while (running_) {
        std::this_thread::sleep_for(std::chrono::duration<double>(rcPeriodS_));
        if (inCommand_ || !flying_) continue;
        std::lock_guard<std::mutex> lk(rcMtx_);
        double now = clock_.now();
        if (now - rcSentStamp_ < rcPeriodS_ * 0.9) continue;
        if (now - rcStamp_ > rcStaleS_) {
            if (rc_[0] || rc_[1] || rc_[2] || rc_[3])
                LOG_WARN("Tello: rc watchdog -> hover (no fresh command for %.2fs)", now - rcStamp_);
            rc_[0] = rc_[1] = rc_[2] = rc_[3] = 0;
        }
        sendRcNow(rc_[0], rc_[1], rc_[2], rc_[3]);
    }
}

TelloState TelloDrone::state() const {
    std::lock_guard<std::mutex> lk(stateMtx_);
    return state_;
}

double TelloDrone::stateAge() const {
    std::lock_guard<std::mutex> lk(stateMtx_);
    return state_.valid ? clock_.now() - state_.stamp : 1e9;
}

bool TelloDrone::streamOn() {
    std::string reply;
    return command("streamon", &reply, 5.0);
}
