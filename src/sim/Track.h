#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "vision/TagDetector.h"

// World frame: X east, Y north (meters), heading yaw in degrees counter-clockwise from +X.
struct TrackTag { int id = 0; double x = 0, y = 0, yawDeg = 0; };
struct TrackSegment { std::vector<cv::Point2d> pts; };

struct Track {
    double worldSize = 9.0;                 // square world [-size/2, size/2]
    cv::Point2d start{0, -3.0};
    double startYawDeg = 90;
    std::vector<TrackSegment> segments;     // in course order
    std::vector<TrackTag> tags;             // one per segment end, last one is FINISH (id 4)
    std::vector<TrackSegment> distractors;
    std::vector<cv::Scalar> distractorColors;
    std::vector<bool> distractorOnStrip;
    std::string description;
};

struct TrackStyle {
    double pxPerM = 100;
    double tagSizeM = 0.16;                 // black square edge
    double lineWidthCm = 3, stripWidthCm = 10;
    cv::Scalar lineColor{25, 25, 25};
    cv::Scalar carpet{118, 108, 100};
    uint64_t textureSeed = 1;
};

class TrackBuilder {
public:
    static Track example();                              // layout from the competition slides
    static Track random(uint64_t seed, double worldSize = 9.0);
    static cv::Mat render(const Track& t, const TrackStyle& s, const TagDetector& tags);
    static cv::Point2d worldToPx(const Track& t, double pxPerM, const cv::Point2d& p);
};
