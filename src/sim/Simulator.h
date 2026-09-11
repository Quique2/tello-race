#pragma once
#include <deque>
#include <random>
#include <string>
#include <vector>

#include "sim/Track.h"
#include "tello/IDrone.h"
#include "util/Clock.h"
#include "util/Config.h"
#include "vision/CameraModel.h"
#include "vision/FrameSource.h"

struct SimParams {
    double fps = 30;
    int latencyFrames = 3;
    double camPitchTrueDeg = 12, camRollTrueDeg = 0.5;
    double rcVel = 0.02;           // m/s per rc unit (forward / lateral)
    double rcVz = 0.01;            // m/s per rc unit (vertical)
    double rcYawRate = 1.2;        // deg/s per rc unit
    double tau = 0.35, yawTau = 0.2;
    double leanDegPerMs = 8;       // airframe tilt per m/s of body velocity
    double takeoffAltM = 0.8;
    double goSpeedErr = 0.04, goPosNoiseM = 0.03, turnErrDeg = 2.0;
    double driftSigma = 0.02;      // random-walk hover drift (m/s)
    double noiseSigma = 4, blurPerMs = 2.5;
    double batteryStart = 85;
    double boundsMargin = 0.3, reachRadiusM = 0.35, finishRadiusM = 0.45;
    double idleLimitS = 60, maxTimeS = 300;
    void load(const Config& cfg);
};

// Kinematic Tello + pinhole camera looking at a rendered track. Implements the drone, the video source
// and the clock so the navigator runs unmodified (optionally faster than real time).
class Simulator : public IDrone, public IFrameSource, public IClock {
public:
    Simulator(const Config& cfg, uint64_t seed, const std::string& trackKind, const TagDetector& tagRenderer);

    // IClock
    double now() const override { return t_; }
    void sleep(double s) override { advance(s); }
    // IDrone
    std::string name() const override { return "sim"; }
    bool connect(std::string*) override { return true; }
    void disconnect() override {}
    bool command(const std::string& cmd, std::string* reply, double timeoutS) override;
    void rc(int lr, int fb, int ud, int yaw) override;
    TelloState state() const override;
    bool streamOn() override { return true; }
    bool isFlying() const override { return flying_; }
    // IFrameSource
    bool start(std::string*) override { return true; }
    void stop() override {}
    bool latest(Frame& out, uint64_t afterSeq, double timeoutS) override;
    bool isLive() const override { return true; }
    bool ended() const override { return false; }
    double fps() const override { return p_.fps; }

    struct Result {
        int segments = 0; bool finished = false, crashed = false, landed = false, tookOff = false;
        double time = 0; std::string reason;
    };
    const Result& result() const { return res_; }
    bool done() const;
    cv::Mat topView(int sizePx) const;
    const Track& track() const { return track_; }
    double x() const { return x_; }
    double y() const { return y_; }
    double z() const { return z_; }
    double yawDeg() const { return psi_; }

private:
    void advance(double dt);
    void physicsStep(double dt);
    void checkProgress();
    void moveBlocking(double dxB, double dyB, double dzB, double speedMs);
    void turnBlocking(double deg);
    cv::Mat render();

    SimParams p_;
    Track track_;
    TrackStyle style_;
    cv::Mat world_;
    GroundProjector truth_;
    std::mt19937_64 rng_;
    double t_ = 0, takeoffT_ = 0, lastProgressT_ = 0;
    uint64_t seq_ = 0;
    double x_ = 0, y_ = 0, z_ = 0, psi_ = 90, psi0_ = 90, zTarget_ = 0;
    double vxB_ = 0, vyB_ = 0, vz_ = 0, yawRate_ = 0;
    double cmdVx_ = 0, cmdVy_ = 0, cmdVz_ = 0, cmdYaw_ = 0;
    double driftX_ = 0, driftY_ = 0;
    double bodyPitch_ = 0, bodyRoll_ = 0;
    bool flying_ = false;
    std::deque<cv::Mat> latencyQ_;
    std::vector<cv::Point2d> trail_;
    Result res_;
    int nextTag_ = 0;
    cv::Mat mapX_, mapY_;
};
