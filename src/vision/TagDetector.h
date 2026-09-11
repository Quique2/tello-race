#pragma once
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "vision/CameraModel.h"

struct apriltag_detector;
struct apriltag_family;

struct TagDetection {
    int id = -1;
    int hamming = 0;
    double margin = 0;                 // decision margin (higher = more confident)
    cv::Point2d center;
    cv::Point2d corners[4];
    bool hasPose = false;
    cv::Matx33d R;                     // tag -> camera rotation
    cv::Vec3d t;                       // tag center in camera frame (m)
    double poseErr = 0;
    double sizePx() const;             // mean edge length in pixels
};

// Thin C++ wrapper over the AprilRobotics apriltag C library.
class TagDetector {
public:
    TagDetector() = default;
    ~TagDetector();
    TagDetector(const TagDetector&) = delete;
    TagDetector& operator=(const TagDetector&) = delete;

    bool init(const std::string& family, double quadDecimate, double quadSigma, int threads,
              bool refineEdges, double decodeSharpening, std::string* err = nullptr);
    // K (full-resolution intrinsics of 'gray') and tagSizeM enable pose estimation.
    std::vector<TagDetection> detect(const cv::Mat& gray, const Intrinsics* K = nullptr, double tagSizeM = 0);
    double lastMs() const { return lastMs_; }
    const std::string& family() const { return family_; }

    // Bitmap of a tag (1 pixel per module, includes the white border), CV_8UC1 0/255. Empty on error.
    cv::Mat renderTag(int id) const;
    // Fraction of the rendered bitmap width that is the black square (what "tag size" refers to)
    double blackSquareFraction() const;

private:
    apriltag_detector* td_ = nullptr;
    apriltag_family* tf_ = nullptr;
    std::string family_;
    double lastMs_ = 0;
};
