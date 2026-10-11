#pragma once
#include <deque>
#include <map>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "tello/TelloState.h"
#include "util/Config.h"
#include "vision/CameraModel.h"
#include "vision/LineDetector.h"
#include "vision/PathTracer.h"
#include "vision/TagDetector.h"

struct TagObservation {
    int id = -1;
    double margin = 0;
    double X = 0, Y = 0;          // ground position in the body frame (m): X forward, Y left
    double dist = 0;
    double yawDeg = 0;            // tag x-axis direction relative to the drone heading (+ left)
    bool poseOk = false;
    double heightFromPose = 0;    // camera height implied by the tag pose (needs the right tag size)
    cv::Point2d center;
    cv::Point2d corners[4];
};

struct Observation {
    double stamp = 0;
    uint64_t seq = 0;
    LineObservation line;
    std::vector<TagObservation> tags;
    double heightM = 0;
    double totalPitchDeg = 0;
    double blindZoneM = 0;
    double lineMs = 0, tagMs = 0;
    // steep (mirror) view: every piece of tape in sight as a centre-line on the floor, instead of one line ahead
    bool pathMode = false;
    std::vector<PathLine> paths;
    bool tagsFresh = true;            // false on the frames where the tag detector was skipped
    std::vector<cv::Point2d> view;    // the piece of floor in the picture: its four corners in the body frame (m)
    // The camera angle was corrected since the previous frame: floor points near the drone are now seen this much
    // further (m, body frame). Positions remembered from before have to be moved by it.
    bool mountChanged = false;
    cv::Point2d mountShift{0, 0};
    bool mountTrusted = true;         // the camera angle is known (configured as verified, or measured and settled)
    double mountPitchDeg = 0;         // the camera angle in use (deg down from the horizon; 90 = straight down)
    const TagObservation* tag(int id) const {
        for (auto& t : tags) if (t.id == id) return &t;
        return nullptr;
    }
};

// Runs tag + line detection on a frame and produces metric observations in the drone body frame.
class Perception {
public:
    bool init(const Config& cfg, std::string* err);
    Observation process(const cv::Mat& frame, const TelloState& st, double stamp, uint64_t seq);
    void resetTracking() { prev_ = LineObservation(); }
    void setLineHint(bool valid, double X, double Y) { hintValid_ = valid; hint_ = cv::Point2d(X, Y); }
    void drawDebug(cv::Mat& frame, const Observation& obs) const;

    GroundProjector& projector() { return proj_; }
    const GroundProjector& projector() const { return proj_; }
    LineDetector& lineDetector() { return line_; }
    const PathTracer& pathTracer() const { return tracer_; }
    bool pathMode() const { return pathMode_; }
    // nav.mode = auto | line | course: "course" (follow the tape seen from above) when the camera looks steeply down
    static bool courseMode(const Config& cfg);
    TagDetector& tagDetector() { return tags_; }
    double tagSizeM() const { return tagSize_; }

    struct Calib { int samples = 0; bool applied = false; double pitchDeg = 0, rollDeg = 0, heightM = 0, heightRatio = 0; };
    const Calib& calib() const { return calib_; }
    int tagWindowCount() const { return tagWindowCount_; }   // windows searched for tags in the last frame; -1 = the whole frame
    // telemetry lean signs in use, and whether the tags have confirmed (or corrected) them in this run
    double imuPitchSign() const { return imuPitchSign_; }
    double imuRollSign() const { return imuRollSign_; }
    bool pitchSignKnown() const { return pitchSignKnown_; }
    bool rollSignKnown() const { return rollSignKnown_; }
    bool imuSignsVerified() const { return imuVerified_ || (pitchSignKnown_ && rollSignKnown_); }
    bool mountTrusted() const { return mountVerified_ || (calib_.applied && calib_.samples >= 40 && calibStable_ >= 2); }
    // Manual calibration hook (calib-ground tool): feed a pose and get the estimate
    void setAutoCalib(bool on) { autoCalib_ = on; }
    // Belly camera mode: tags are detected (no pose), the line/floor overlay is skipped (camera looks straight down)
    void setDownwardCamera(bool on) { downward_ = on; resetTracking(); }
    bool downwardCamera() const { return downward_; }

private:
    void updateCalib(const TagDetection& d, const TelloState& st);
    // ---- course mode (camera looking down)
    Observation processCourse(const cv::Mat& frame, const TelloState& st, Observation obs);
    std::vector<cv::Rect> tagWindows(const cv::Size& full, double stamp);   // where a tag can be: tracked ones + dark squares
    void updatePlaneCalib(const TagDetection& d, const TelloState& st);
    void solvePlane();
    bool fitPlane(double signPitch, double signRoll, bool robust, double& pitchDeg, double& rollDeg, double& scale, double& rms, int& used) const;
    bool fitLeanGains(double& gPitch, double& sePitch, double& gRoll, double& seRoll) const;

    TagDetector tags_;
    LineDetector line_;
    PathTracer tracer_;
    bool pathMode_ = false;
    bool tagWindowsOn_ = true;               // tag.windows: look for tags in small windows instead of the whole frame
    int tagDarkV_ = 100;                     // tag.dark_v_max: a tag is a dark square of about the right size
    double tagFullEveryS_ = 1.0;             // tag.full_frame_every_s: whole-frame search this often (safety net)
    double lastFullFrameT_ = -1e9;
    int tagWindowCount_ = 0;
    std::vector<cv::Rect> lastWindows_;
    std::vector<cv::Point2d> lastTagGround_;
    // camera tilt from where the tags are in space: every tag centre is a point of the floor plane
    struct PlaneSample { int id; cv::Vec3d q; double H, pitch, roll; };
    std::deque<PlaneSample> plane_;
    std::map<int, cv::Point2d> planeLastPx_;
    int planeNew_ = 0;
    double mountPitch0_ = 0, mountRoll0_ = 0;   // configured mount: the starting point of the fit
    bool heightFromTags_ = false;            // ground.height_from_tags: scale the height sensor with the tags
    double heightScale_ = 1.0;
    double planeStateStamp_ = -1, planeLastH_ = 0, planeClimb_ = 0;
    bool pitchSignKnown_ = false, rollSignKnown_ = false;
    bool mountVerified_ = true;              // ground.mount_verified: camera.pitch_deg / roll_deg are measured values
    int calibStable_ = 0;                    // solves in a row that changed the angles by less than 0.3 deg
    bool shiftPending_ = false;
    cv::Point2d pendingShift_{0, 0};
    GroundProjector proj_;
    LineObservation prev_;
    bool hintValid_ = false;
    bool downward_ = false;
    cv::Point2d hint_;
    Calib calib_;

    std::string heightSource_ = "tof";
    double fixedHeightM_ = 0.5;
    bool useImu_ = true;
    double imuPitchSign_ = 1, imuRollSign_ = 1;
    // the picture is older than the telemetry: the lean is taken from that much earlier (ground.attitude_delay_s)
    double attDelayS_ = 0, leanPitch_ = 0, leanRoll_ = 0;
    // ground.imu_signs_verified = false: the lean is left out of the floor model until the tags have shown which way
    // the telemetry counts it (a wrong sign doubles the error it is meant to remove; none at all costs little)
    bool imuVerified_ = true;
    double effPitchSign() const { return !useImu_ ? 0.0 : (imuVerified_ || pitchSignKnown_) ? imuPitchSign_ : 0.0; }
    double effRollSign() const { return !useImu_ ? 0.0 : (imuVerified_ || rollSignKnown_) ? imuRollSign_ : 0.0; }
    struct AttSample { double t, pitch, roll; };
    std::deque<AttSample> attHist_;
    void attitudeAt(double t, const TelloState& st, double& pitch, double& roll) const;
    double tagSize_ = 0.16;
    int tagEveryN_ = 1;
    double tagMinMargin_ = 25;
    double tagMaxDist_ = 3.0;
    std::string tagPosSource_ = "pose";
    bool autoCalib_ = true;
    int autoCalibMinSamples_ = 12;
    double calibMaxDist_ = 2.2, calibMinMargin_ = 40;
    std::deque<double> pitchS_, rollS_, heightS_, ratioS_;
    uint64_t frameCount_ = 0;
    std::vector<TagDetection> lastRaw_;
    struct TagMemory { int id = -1; cv::Point2f corners[4]; cv::Point2f center, vel; double lastSeen = 0; bool seenNow = false; };
    std::vector<TagMemory> tagMem_;
};
