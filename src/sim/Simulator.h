#pragma once
#include <deque>
#include <functional>
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
    double leanDegPerMs = 8;       // airframe tilt per m/s of body velocity (holding the speed against the drag)
    double leanDegPerMs2 = 0;      // airframe tilt per m/s^2 (what it takes to accelerate: 5.8 deg is the physics)
    double leanTau = 0.10;         // s, how fast the airframe tilts
    double takeoffAltM = 0.8;
    double goSpeedErr = 0.04, goPosNoiseM = 0.03, turnErrDeg = 2.0;
    double driftSigma = 0.02;      // random-walk hover drift (m/s)
    double noiseSigma = 4, blurPerMs = 2.5;
    double batteryStart = 85;
    double boundsMargin = 0.3, reachRadiusM = 0.35, finishRadiusM = 0.45;
    double idleLimitS = 60, maxTimeS = 300;
    double tracePoseS = 0;   // >0: log the true pose every N s of sim time (debug)
    double exposureS = 0;    // >0: motion blur from the real image motion during the exposure (replaces blurPerMs)
    int exposureSamples = 1; // >1: the picture is the mean of up to this many views along the way flown and turned during the
                             // exposure, so a turning nose smears the picture round the drone (1: one straight smear for all of it)
    double startJitterM = 0, startYawJitterDeg = 0;   // placement error of the drone at the start (sigma)
    double startYawOffsetDeg = 0;                     // the drone is put down turned by this much (180 = facing backwards)
    double rcVelLat = 0;     // m/s per rc unit sideways (0 = same as forward)
    double telemetryHz = 0;  // >0: the state is refreshed at this rate (the real Tello sends ~10 Hz)
    double tofScale = 1.0;   // the height sensor reads this times the true height
    double focalScale = 1.0; // the true focal length is this times camera.fx / fy
    double rcDeadband = 0;   // stick values smaller than this do nothing
    double imageGain = 1.0;  // brightness of the picture (the real camera may expose a white floor much darker)
    double camAheadM = 0;    // the camera's viewpoint is this far ahead of the drone's centre (mirror at the nose)
    double rcDelayS = 0;     // a stick command starts to act this long after it was sent (measured on the real drone: 0.3 s)
    bool inertial = false;   // true: while the nose turns, the drone keeps moving the way it was going over the floor
                             // (as a real one does) and only comes round with its response time
    // What the real camera shows and a clean rendering does not (all off unless set; see docs/COURSE_TEC.md).
    std::vector<double> glareLamps;   // x, y, height (m) of every lamp whose reflection on the glossy tape washes it pale
    double glareSigmaM = 0.11;        // size of the washed patch on the floor
    double glareStrength = 0.7;       // how far the tape goes towards white in the middle of the patch (0 - 1)
    double chromaBlurPx = 0;          // the camera gives colour more coarsely than brightness: a colour edge is this much softer (sigma, pixels)
    double blockNoiseSigma = 0;       // colour error of whole 8x8 pixel blocks, as video compression leaves it (BGR levels)
    double tintSigma = 0;             // colour error that stays on the floor from frame to frame (blue against green, levels) ...
    double tintPatchM = 0.012;        // ... in patches of this size
    double tintWideSigma = 0;         // ... and from one end of the course to the other (the light is not the same colour everywhere)
    double tintDriftSigma = 0;        // ... and of the whole picture, wandering slowly (the camera's white balance)
    double mirrorEdgePx = 0;          // > 0: the lower edge of the mirror, out of focus, darkens this many rows at the bottom
    double mirrorEdgeTiltPx = 0;      // ... and reaches this much higher at the right border than at the left
    cv::Scalar mirrorEdgeColor{81, 85, 100};
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
        double devSum = 0, devSq = 0, devMax = 0; int devN = 0;   // distance to the route (tape + links) while flying
        std::vector<double> tagPass;      // closest approach to every tag, in course order (m)
        double pathLen = 0;               // distance flown (m)
    };
    const Result& result() const { return res_; }
    // a manoeuvre that leaves the route on purpose (measuring the camera angle over a tag) is not a tracking error
    void pauseDeviation(bool on) { devPaused_ = on; }
    bool done() const;
    cv::Mat topView(int sizePx) const;
    const Track& track() const { return track_; }
    double x() const { return x_; }
    double y() const { return y_; }
    double z() const { return z_; }
    double yawDeg() const { return psi_; }
    const cv::Mat& worldImage() const { return world_; }         // rendered floor (top view, style_.pxPerM per metre)
    double pxPerM() const { return style_.pxPerM; }
    double camPitchTrueDeg() const { return p_.camPitchTrueDeg; }
    uint64_t seed() const { return seed_; }
    // called after every physics step of a blocking command (go / turn / move): lets the host pace it in wall
    // time and publish the intermediate poses, so viewers see the flight instead of a jump
    void setBlockingStepCallback(std::function<void(double dtSim)> cb) { blockingCb_ = std::move(cb); }
    double hfovDeg() const { return truth_.K().hfovDeg(); }
    double vfovDeg() const { return truth_.K().vfovDeg(); }

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
    double t_ = 0, takeoffT_ = 0, lastProgressT_ = 0, lastTraceT_ = -1e9;
    std::function<void(double)> blockingCb_;
    std::vector<cv::Point2d> route_;
    bool landing_ = false, takingOff_ = false, devPaused_ = false;
    mutable TelloState heldState_;
    mutable double heldStateT_ = -1e9;
    TelloState trueState() const;
    uint64_t seed_ = 0;
    uint64_t seq_ = 0;
    double x_ = 0, y_ = 0, z_ = 0, psi_ = 90, psi0_ = 90, zTarget_ = 0;
    double vxB_ = 0, vyB_ = 0, vz_ = 0, yawRate_ = 0;
    double cmdVx_ = 0, cmdVy_ = 0, cmdVz_ = 0, cmdYaw_ = 0;
    struct PendingRc { double t, vx, vy, vz, yaw; };
    std::deque<PendingRc> rcQueue_;   // sticks on their way to the motors (sim.rc_delay_s)
    double driftX_ = 0, driftY_ = 0;
    double bodyPitch_ = 0, bodyRoll_ = 0, leanAx_ = 0, leanAy_ = 0;
    bool flying_ = false;
    std::deque<cv::Mat> latencyQ_;
    std::vector<cv::Point2d> trail_;
    Result res_;
    int nextTag_ = 0;
    cv::Mat mapX_, mapY_;
    cv::Mat bodyX_, bodyY_;           // floor point of every pixel as seen from the camera, drone axes (sim.exposure_samples)
    cv::Mat renderExposure(int n);
    cv::Mat glare_;                   // per pixel: how much of a lamp's reflection lies there (sim.glare_lamps)
    double floorWhite_ = 255;         // brightness of the white surface of the rendered floor: what a reflection washes towards
    void applyGlare(cv::Mat& img) const;
    void applyMirrorEdge(cv::Mat& img) const;
    void applyChromaBlur(cv::Mat& img) const;
    void addBlockNoise(cv::Mat& f);
    cv::Mat blockNoise_;              // the block errors of the last picture: most of each is still there in the next
    double tintDrift_ = 0;            // where the picture's blue against green stands at the moment (sim.tint_drift_sigma)
    void tintWorld();
};
