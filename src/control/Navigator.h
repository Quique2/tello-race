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
    double tagSlowDistM = 0.5;
    // tag handling
    int confirmFrames = 2;
    double approachDistM = 0, approachMarginM = 0.3, endTangentMinTagM = 0.45, settleS = 0.8, settleMaxS = 2.5, gapM = 0.8, lineEndAssocM = 1.0, stopOvershootM = 0.15;
    double approachBackoffM = 0.5;   // a tag found closer than this (under the drone, from altitude) is measured after backing off
    double curvedApproachDeg = 15, tagEndAssocM = 0.6, acquireMaxOffM = 0.75;
    bool backAfterTurn = true, forwardFastPath = false;
    int goSpeed = 60;
    bool goYLeft = true, ignoreVisited = true;
    double velScale = 0.01, vgySign = -1;   // telemetry vgx/vgy -> m/s, and the sign that makes +vgy = left (CALIBRATE)
    std::string turnMode = "sdk";
    double rcTurnRate = 50, rcTurnTolDeg = 4, rcTurnK = 1.5;
    // acquire
    double alignYawTolDeg = 8, alignLatTolCm = 12, vAcquire = 20, acquireTimeoutS = 1.5, rotateInPlaceDeg = 35, followRotateDeg = 25;
    // gap
    double coastRc = 30, gapTimeoutS = 6, kYawHold = 1.0, gapAlignDeg = 20;
    // search
    double sweepDeg = 35, sweepRate = 40, maxSearchS = 30;
    int backCm = 30, riseCm = 25, maxBackCm = 60;
    void load(const Config& cfg);
};

struct RcCommand { int lr = 0, fb = 0, ud = 0, yaw = 0; };

// Course mode: the camera looks steeply down (mirror), so every piece of tape around the drone is seen from
// above. The navigator follows the tape itself, flies over each tag at its end and takes the nearest other
// tape end as the start of the next segment. The drone keeps its heading and moves sideways/backwards freely.
struct CourseParams {
    std::vector<int> order{1, 2, 3, 4};   // tags in the order they are visited; the last one is the finish
    int finishId = 4;
    double vCruise = 0.45, vMin = 0.08, vWaypoint = 0.20, vClimb = 0.25;   // m/s
    double vGap = 0.25;                   // m/s, across the gap between a tag and the start of the next tape
    double lookaheadM = 0.05, lookaheadPerMs = 0.45;                         // preview distance = base + per m/s of speed
    double tagLookPastM = 0.02;           // the preview point goes no further than this beyond a tag not yet flown over
    double kCross = 1.5;                  // 1/s: pull back onto the route per metre of cross-track error
    double kBias = 0.4, biasMax = 0.12;   // 1/s^2, m/s: integral of that error = what it takes to hold against a steady drift
    double biasZoneM = 0.08;              // ... only errors smaller than this are integrated (a big one is not a drift)
    bool calibAtFirstTag = true;          // camera angle not known yet: measure it over the first tag before going on
    double mountPitchMinDeg = 84, mountPitchMaxDeg = 104;   // outside this the camera is blind behind / ahead of the drone: land
    double traceHz = 5;                   // debug log: this many lines a second about what the navigator is doing (0 = none)
    // course.heading = fixed: the drone keeps the heading it took off with and moves sideways / backwards as needed.
    // course.heading = path: the nose turns to point along the route. Same path following underneath.
    // course.heading = lead: as path, but the nose turns AHEAD of the route (the lead* values below). A mode of its
    // own so that the two that have flown the real course stay exactly as they flew.
    bool headingFollow = false;           // path or lead: the nose turns
    bool headingLead = false;             // lead
    double headingLookM = 0.25;           // the nose aims at the route this far beyond the preview point
    double maxYawRc = 60;                 // yaw stick limit while turning with the path
    double leadKYaw = 1.5, leadMaxYawRc = 100;   // lead: yaw stick units per degree, and the yaw stick limit
    double leadLookM = 0.15, leadLookS = 0.7;    // lead: the nose takes the direction the route HAS this far beyond the
                                                 // preview point, plus this many seconds of travel (it needs time to come round)
    double leadSlowDeg = 25;              // lead, > 0: fly slower while the nose is further than this from where it should point
    double leadSlowMin = 0.45;            // ... down to this share of the speed, 60 degrees further off
    double camAheadM = 0.06;              // the camera looks down from the mirror, this far ahead of the drone's centre
    double maxRangeM = 0;                 // > 0: land when the drone seems to be further than this from where it took off
    double accel = 0.7, latAccel = 0.5;                                      // m/s^2: braking and cornering limits
    double cornerBaseM = 0.075;           // m: a corner is measured between two chords of this length
    double rcGain = 0.015;                // m/s per stick unit (start value; measured in flight when adaptGain)
    bool adaptGain = true;
    double respTau = 0.35;                // s, how fast the drone follows a new stick command
    double latencyS = 0.20;               // s, age of the image when it is used
    double cmdDelayS = 0;                 // s, a stick command starts to act this long after it was sent
    double shapeTau = 0;                  // s, > 0: the sticks are overdriven so that the drone answers this fast
    double shapeMaxMs = 0.35;             // m/s, the most that overdrive may add
    double tagCaptureM = 0.05, startCaptureM = 0.06, tagAssocM = 0.45, nextSearchM = 1.0, firstSearchM = 1.2;
    double lostCoastS = 0.35, lostLandS = 5.0, endWaitS = 1.5;
    double finishCenterM = 0.035, finishCenterMaxS = 2.5;
    double finishAltCm = 0, finishDescentM = 0.6;   // > 0: come down to this height over the last stretch before the finish tag
    double maxUd = 60, maxRc = 70, kYawHold = 1.5;
    void load(const Config& cfg);
};

// Mission state machine: takeoff -> follow line -> read tag -> maneuver -> acquire next line ... -> land.
class Navigator {
public:
    Navigator(IDrone& drone, IClock& clock, const Config& cfg);
    void start(int startAtTag = 0);      // 0 = start line; 1/2/3 = resume standing on that tag (checkpoint)
    void requestLand();                  // graceful stop
    void release();                      // hand the sticks to the operator (manual mode): navigator goes idle, drone keeps hovering
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
    // where the next line is expected (body frame) while acquiring after a tag maneuver
    bool lineHint(double& X, double& Y) const {
        if (!acqFromManeuver_ || (phase_ != Phase::Acquire && phase_ != Phase::Search)) return false;
        X = expectX_; Y = expectY_; return true;
    }

    double targetX = 0, targetY = 0;
    bool haveTarget = false;
    std::vector<std::string> events;

    // course mode: what the HUD draws on the video
    bool courseMode() const { return courseMode_; }
    const CourseParams& courseParams() const { return cp_; }
    std::vector<cv::Point2d> courseRoute;     // route ahead in the body frame (m)
    int courseWaypoint = -1;                  // index in courseRoute of the next tag / line start to fly over
    int nextTagId() const;
    double speedMs() const;                   // modelled ground speed
    double rcGain() const { return cs_.gain; }
    bool rcGainMeasured() const { return cs_.gainSamples >= 20; }
    // the nose: "fixed" (keeps the take-off heading), "path" (turns with the route) or "lead" (turns ahead of it)
    const char* courseNoseMode() const { return cp_.headingLead ? "lead" : cp_.headingFollow ? "path" : "fixed"; }
    bool setCourseNoseMode(const std::string& name);                     // false: no such mode, nothing changed

private:
    void setPhase(Phase ph, const char* why);
    void sendRc(double lr, double fb, double ud, double yaw, bool instant = false);
    void hover(bool instant = true);
    double altitudeUd(const TelloState& st);
    void updateTagTrack(const Observation& obs);
    bool tagConfirmed() const;
    void updateEndTangent(const Observation& obs, const TelloState& st);
    double endTangentRelDeg(const TelloState& st) const;
    double approachDist() const;
    bool pursuitTarget(const LineObservation& line, double& tx, double& ty, double* tangentDeg = nullptr) const;

    // ---- course mode (NavigatorCourse.cpp)
    void stepCourse(const Observation& obs, const TelloState& st);
    void courseTakeoff(const TelloState& st);
    void courseFly(const Observation& obs, const TelloState& st);
    void courseCommand(const cv::Point2d& vel, const TelloState& st);
    void courseTagReached(int id, const cv::Point2d& pos, const Observation& obs, int activeLine);
    bool courseChooseNext(const Observation& obs, int excludeLine);
    bool courseFindNext(const Observation& obs, int excludeLine, const cv::Point2d& anchor, const cv::Point2d& arrivalDir,
                        int& line, int& end, double& dist) const;
    void courseLand(const char* phaseWhy, Phase endPhase);

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
    CourseParams cp_;
    bool courseMode_ = false;
    bool courseCalibDone_ = false;        // the camera angle was measured over a tag in an earlier flight of this run
    struct CourseState {
        enum Stage { First, Line, SeekNext, Finish, Calib } stage = First;
        int calibLeg = 0;                            // step of the manoeuvre that measures the camera angle over a tag
        double calibLegStart = 0;
        int k = 0;                                   // index in cp_.order of the next tag
        double stageStart = 0, lastT = 0, lastRouteT = 0, endSince = -1, blindUntil = -1e9;
        // memory in the body frame, moved every frame by the motion estimate
        bool havePrev = false;
        cv::Point2d prevTarget, prevBehind, travelDir{1, 0};
        double prevExtra = 0;                        // how far beyond the end of the tape the previous preview point was
        cv::Point2d bias{0, 0};                      // m/s: velocity added to hold against a steady drift
        cv::Point2d fromStart{0, 0};                 // m: where the drone is from its take-off point, by dead reckoning
        bool haveStart = false, startCaptured = true;
        cv::Point2d startPos, anchor, arrivalDir{1, 0};
        int anchorTag = -1;                          // the tag the anchor is (-1: the end of a tape without tag)
        int goalTag = -1;                            // the tag at the end of the tape being followed, once it has been seen
        struct TagMem { int id; cv::Point2d p; double seen; bool fresh; cv::Point2d prevP; double prevSeen; };
        std::vector<TagMem> tags;
        // motion model: the commanded velocity seen through the drone's response, and its recent history
        cv::Point2d vCmd, vModel;
        cv::Point2d vPred;                           // modelled velocity when a command sent now starts to act
        cv::Point2d vW;                              // the modelled velocity in a frame that does not turn with the nose
        double yawRate = 0;                          // deg/s, how fast the nose is turning (telemetry)
        double lastPsiImg = 0;                       // lead: heading at the time of the previous picture
        bool psiImgValid = false;
        std::deque<std::pair<double, cv::Point2d>> cmdHist;   // what was sent and when: it acts cmdDelayS later
        std::deque<std::pair<double, cv::Point2d>> vHist;
        double gain = 0.015;
        int reversedCount = 0, gainSamples = 0;
        bool yawValid = false, yaw0Valid = false, yawHoldOff = false;
        int lastYaw = 0, yaw0 = 0;
        double yawRef = 0;                           // heading being held (telemetry degrees, clockwise positive)
        std::deque<std::pair<double, int>> yawHist;  // recent telemetry headings: the picture is older than the newest one
        double altBoostCm = 0, lastTraceT = -1e9;
        bool finalDescent = false;                   // on the last stretch: flying at the finish height
        int seekRises = 0;
        double seekOverT = 0;                        // time spent over the tag while looking for the next tape
    } cs_;
    Phase phase_ = Phase::Idle;
    double phaseStart_ = 0, missionStart_ = 0, lastStep_ = 0, now_ = 0, prevStep_ = 0, blindUntil_ = -1e9;
    int alignFrames_ = 0;
    double gateFailStart_ = -1;
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
    std::vector<std::pair<int, std::pair<double, double>>> lastTagIds_;

    double approachStart_ = 0;
    int lastTurnTag_ = 0;
    double lastManeuverT_ = -1e9;   // tag of the last maneuver: while it stays in view it anchors the expected next start
    std::vector<double> apX_, apY_, apT_;
    double maneuverX_ = 0, maneuverY_ = 0;
    bool turning_ = false;
    int turnTargetYaw_ = 0;
    double turnStart_ = 0, turnStableSince_ = -1;

    double expectX_ = 0.8, expectY_ = 0, acqBackM_ = 0;
    // last maneuver tag in the current body frame: dead-reckoned from yaw + telemetry velocity, re-synced when seen
    bool anchorValid_ = false;
    double anchorX_ = 0, anchorY_ = 0, anchorT_ = 0;
    int anchorYaw_ = 0;
    void setAnchor(double X, double Y, const TelloState& st);
    void updateAnchor(const TelloState& st);
    bool acqFromManeuver_ = false;
    int acqAbsYaw_ = 0, backwardFrames_ = 0;
    int gapYaw0_ = 0;
    bool gapYawValid_ = false;
    double gapAlignedAt_ = -1;

    int searchStep_ = 0, sweepDir_ = 1, searchSweeps_ = 0, searchYaw0_ = 0;
    double searchStart_ = 0, searchStepStart_ = 0;
    bool searchYawValid_ = false, searchDidBack_ = false, searchDidRise_ = false, searchFromAcquire_ = false, searchBentAway_ = false;
    int sweepFirst_ = 1, searchBackTotal_ = 0;
    double lastSearchEnd_ = -1e9;
};
