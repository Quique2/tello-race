#pragma once
#include <deque>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "tello/TelloState.h"
#include "util/Config.h"
#include "vision/CameraModel.h"
#include "vision/LineDetector.h"
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
    TagDetector& tagDetector() { return tags_; }
    double tagSizeM() const { return tagSize_; }

    struct Calib { int samples = 0; bool applied = false; double pitchDeg = 0, rollDeg = 0, heightM = 0, heightRatio = 0; };
    const Calib& calib() const { return calib_; }
    // Manual calibration hook (calib-ground tool): feed a pose and get the estimate
    void setAutoCalib(bool on) { autoCalib_ = on; }
    // Belly camera mode: tags are detected (no pose), the line/floor overlay is skipped (camera looks straight down)
    void setDownwardCamera(bool on) { downward_ = on; resetTracking(); }
    bool downwardCamera() const { return downward_; }

private:
    void updateCalib(const TagDetection& d, const TelloState& st);

    TagDetector tags_;
    LineDetector line_;
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
