#pragma once
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

#include "tello/IDrone.h"
#include "tello/UdpSocket.h"
#include "util/Clock.h"
#include "util/Config.h"

// Real DJI Tello over the text UDP SDK (command port 8889, state port 8890, video port 11111).
class TelloDrone : public IDrone {
public:
    TelloDrone(const Config& cfg, IClock& clock);
    ~TelloDrone() override;

    std::string name() const override { return "tello"; }
    bool connect(std::string* err) override;
    void disconnect() override;
    bool command(const std::string& cmd, std::string* reply, double timeoutS) override;
    void rc(int lr, int fb, int ud, int yaw) override;
    TelloState state() const override;
    bool streamOn() override;
    bool isFlying() const override { return flying_; }
    bool onGround() const;   // telemetry says h == 0 and tof <= 10

    std::string videoUrl() const { return videoUrl_; }
    void setDryRun(bool on) { dryRun_ = on; }   // video + telemetry only, no flight commands
    bool setDownvision(bool on);                // SDK 3.0: stream the belly camera (320x240 grey) instead of the front one
    bool downvision() const { return downvision_; }
    double stateAge() const;      // seconds since the last telemetry packet (large if none)

private:
    void replyLoop();
    void stateLoop();
    void rcLoop();
    void sendRcNow(int lr, int fb, int ud, int yaw);

    IClock& clock_;
    std::string ip_;
    uint16_t cmdPort_, statePort_, localPort_;
    std::string videoUrl_;
    double rcPeriodS_, rcStaleS_;

    UdpSocket cmdSock_, stateSock_;
    std::thread replyThread_, stateThread_, rcThread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> inCommand_{false};
    std::atomic<bool> flying_{false};
    double groundSince_ = 0;
    bool dryRun_ = false;
    bool downvision_ = false;

    std::mutex cmdMtx_;                 // serializes blocking commands
    std::mutex replyMtx_;
    std::condition_variable replyCv_;
    std::string lastReplyText_;
    uint64_t replySeq_ = 0;

    mutable std::mutex stateMtx_;
    TelloState state_;

    std::mutex rcMtx_;
    int rc_[4] = {0, 0, 0, 0};
    double rcStamp_ = -1e9, rcSentStamp_ = -1e9;
};
