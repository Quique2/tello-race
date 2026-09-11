#pragma once
#include <deque>
#include <set>
#include <string>
#include <vector>

#include "tello/IDrone.h"
#include "util/Clock.h"
#include "util/Config.h"
#include "util/MathUtil.h"
#include "vision/Perception.h"

enum class Phase { Idle, Takeoff, Acquire, Follow, TagApproach, TagManeuver, GapCoast, Search, Lost, Finish, Landed, Aborted };

struct NavParams {
    // mission
    double altitudeCm = 50, takeoffSettleS = 1.0, maxFlightS = 0, postCommandBlindS = 0.5;
    int minBattery = 25;
    // follow (pure pursuit + lateral correction)
    double vMax = 60, vMin = 15, lookaheadM = 0.45;
    double kYaw = 1.2, kdYaw = 0.15, kTan = 0.6, kLat = 0.8, kdLat = 0.2, maxYaw = 60, maxLat = 35;
    double slowAngleDeg = 25, curvSlow = 1.5, rcSlew = 250;
    double lostCoastS = 0.4, lostHoverS = 1.0;
    double kAlt = 1.0, maxUd = 30, altDeadCm = 4;
    double tagSlowDistM = 1.8;
    // tag handling
    int confirmFrames = 2;
    double approachDistM = 1.4, settleS = 0.8, settleMaxS = 2.5, gapM = 0.8, lineEndAssocM = 1.0, stopOvershootM = 0.15;
    double curvedApproachDeg = 15, tagEndAssocM = 0.6, acquireMaxOffM = 0.75;
    bool backAfterTurn = true, forwardFastPath = false;
    int goSpeed = 60;
    bool goYLeft = true, ignoreVisited = true;
    std::string turnMode = "sdk";
    double rcTurnRate = 50, rcTurnTolDeg = 4, rcTurnK = 1.5;
    // acquire
    double alignYawTolDeg = 8, alignLatTolCm = 12, vAcquire = 20, acquireTimeoutS = 1.5, rotateInPlaceDeg = 35, followRotateDeg = 25;
    // gap
    double coastRc = 30, gapTimeoutS = 6, kYawHold = 1.0, gapAlignDeg = 20;
    // search
    double sweepDeg = 35, sweepRate = 40, maxSearchS = 30;
    int backCm = 30, riseCm = 25;
    void load(const Config& cfg);
};

struct RcCommand { int lr = 0, fb = 0, ud = 0, yaw = 0; };

// Mission state machine: takeoff -> follow line -> read tag -> maneuver -> acquire next line ... -> land.
class Navigator {
public:
    Navigator(IDrone& drone, IClock& clock, const Config& cfg);
    void start(int startAtTag = 0);      // 0 = start line; 1/2/3 = resume standing on that tag (checkpoint)
    void requestLand();                  // graceful stop
    void emergencyStop();
    void step(const Observation& obs, const TelloState& st);

    Phase phase() const { return phase_; }
    const char* phaseName() const;
    static const char* phaseName(Phase p);
    std::string status() const;
    RcCommand lastRc() const { return rc_; }
    int segmentsDone() const { return segmentsDone_; }
    double missionTime() const;
    bool finished() const { return phase_ == Phase::Landed || phase_ == Phase::Aborted; }
    bool flying() const { return phase_ != Phase::Idle && !finished(); }
    const NavParams& params() const { return p_; }

    double targetX = 0, targetY = 0;
    bool haveTarget = false;
    std::vector<std::string> events;

private:
    void setPhase(Phase ph, const char* why);
    void sendRc(double lr, double fb, double ud, double yaw, bool instant = false);
    void hover(bool instant = true);
    double altitudeUd(const TelloState& st);
    void updateTagTrack(const Observation& obs);
    bool tagConfirmed() const;
    void updateEndTangent(const Observation& obs, const TelloState& st);
    double endTangentRelDeg(const TelloState& st) const;
    bool pursuitTarget(const LineObservation& line, double& tx, double& ty, double* tangentDeg = nullptr) const;

    void doTakeoff(const TelloState& st);
    void doAcquire(const Observation& obs, const TelloState& st);
    void doFollow(const Observation& obs, const TelloState& st);
    void doTagApproach(const Observation& obs, const TelloState& st);
    void doTagManeuver(const Observation& obs, const TelloState& st);
    void doGapCoast(const Observation& obs, const TelloState& st);
    void doSearch(const Observation& obs, const TelloState& st);
    void doFinish();
    void beginAcquire(const char* why);
    void beginAcquireAfterManeuver(const char* why);
    void stepBackForView();
    void afterBlockingCommand();
    bool lineNearExpectedStart(const LineObservation& line, const TelloState& st);
    bool visitedTagAhead(const Observation& obs) const;
    void beginGap(const char* why, const TelloState& st);
    void beginSearch(const char* why, const TelloState& st);
    void beginTurn(int deg, const TelloState& st);

    IDrone& drone_;
    IClock& clock_;
    NavParams p_;
    Phase phase_ = Phase::Idle;
    double phaseStart_ = 0, missionStart_ = 0, lastStep_ = 0, now_ = 0, prevStep_ = 0, blindUntil_ = -1e9;
    int alignFrames_ = 0;
    int segmentsDone_ = 0, startAtTag_ = 0;
    std::set<int> visited_;
    bool landRequested_ = false;

    RcCommand rc_;
    mu::SlewLimiter slewLr_, slewFb_, slewUd_, slewYaw_;

    LineObservation lastLine_;
    double lastLineSeen_ = -1e9, blindZone_ = 0.8;
    double prevAngle_ = 0, prevLat_ = 0;

    struct TagTrack { int id = -1; int count = 0; double X = 0, Y = 0, lastSeen = -1e9; bool confirmed = false; } tag_;
    int pendingDir_ = 0;
    bool forwardTagAhead_ = false;
    double lastFwdTagX_ = 9.0, lookahead_ = 0;
    bool endYawValid_ = false;
    double endAbsYaw_ = 0, endYawSin_ = 0, endYawCos_ = 0, endYawStamp_ = -1e9;
    int endYawN_ = 0;
    struct EndSample { double absYaw, endX, t; };
    std::deque<EndSample> endSamples_;

    double approachStart_ = 0;
    std::vector<double> apX_, apY_, apT_;
    double maneuverX_ = 0, maneuverY_ = 0;
    bool turning_ = false;
    int turnTargetYaw_ = 0;
    double turnStart_ = 0, turnStableSince_ = -1;

    double expectX_ = 0.8, expectY_ = 0, acqBackM_ = 0;
    bool acqFromManeuver_ = false;
    int acqAbsYaw_ = 0, backwardFrames_ = 0;
    int gapYaw0_ = 0;
    bool gapYawValid_ = false;
    double gapAlignedAt_ = -1;

    int searchStep_ = 0, sweepDir_ = 1, searchSweeps_ = 0, searchYaw0_ = 0;
    double searchStart_ = 0, searchStepStart_ = 0;
    bool searchYawValid_ = false, searchDidBack_ = false, searchDidRise_ = false, searchFromAcquire_ = false, searchBentAway_ = false;
    int sweepFirst_ = 1;
};
