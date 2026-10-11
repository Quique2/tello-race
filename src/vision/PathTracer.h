#pragma once
#include <vector>

#include <opencv2/core.hpp>

#include "util/Config.h"
#include "vision/CameraModel.h"

// One piece of tape seen from above, as a centre-line in the drone body frame (metres: x forward, y left).
struct PathLine {
    std::vector<cv::Point2d> pts;      // ordered along the line, about stepM apart
    bool cutA = false, cutB = false;   // the first / last point is at the edge of the visible floor: the tape goes on out of view
    double length = 0;                 // m
    double widthCm = 0;                // median tape width
    cv::Point2d tangentA() const;      // unit vector pointing OUT of the line at its first point
    cv::Point2d tangentB() const;      // unit vector pointing OUT of the line at its last point
};

struct TraceParams {
    double cellM = 0.0125;             // resolution of the top-down grid
    double maxRangeM = 2.5;            // floor further than this from the drone is ignored
    double minLengthM = 0.08;
    double minWidthCm = 2.0, maxWidthCm = 11.0;
    double stepM = 0.025;              // spacing of the returned points
    double joinGapM = 0.20;            // pieces of one tape separated by a crossing tape or a glare patch are joined
    double joinCos = 0.6;
    double tagGuardM = 0.14;           // never join across a tag
    int edgeMarginPx = 10;             // full-resolution pixels: a line end this close to the image border is "cut"
    void load(const Config& cfg);
};

// Turns the binary tape mask of a camera that looks steeply down into metric centre-lines: the mask is warped
// onto a top-down grid (the floor is a plane, so this is one homography), thinned to one-cell-wide skeletons
// and every skeleton is walked from one end to the other.
class PathTracer {
public:
    void configure(const Config& cfg) { p_.load(cfg); }
    const TraceParams& params() const { return p_; }
    // mask: tape mask at `scale` times the full camera resolution. tags: tag centres on the floor (body frame).
    std::vector<PathLine> trace(const cv::Mat& mask, double scale, const GroundProjector& proj,
                                const std::vector<cv::Point2d>& tags);
    bool supported(const GroundProjector& proj) const;   // the whole image must look at the floor
    const cv::Mat& grid() const { return grid_; }        // last top-down mask (debug)

private:
    TraceParams p_;
    cv::Mat grid_, skel_, dist_;
    cv::Mat1i index_;
};
