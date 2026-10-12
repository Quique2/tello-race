#pragma once
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "util/Config.h"
#include "vision/CameraModel.h"

struct LineParams {
    int procWidth = 640;
    std::string colorMode = "dark";        // "dark" (low value, any hue) | "hsv" (explicit ranges) | "blue" | "yellow" (tape presets)
    cv::Scalar blueLo{95, 80, 40}, blueHi{132, 255, 255}, yellowLo{16, 90, 90}, yellowHi{38, 255, 255};
    double whiteDilateCm = 0;              // > 0: the white-surface test reaches this far on the floor (scales with the height)
    int darkVMax = 90, darkSMax = 255;
    cv::Scalar hsvLo1{0, 0, 0}, hsvHi1{180, 255, 255}, hsvLo2{0, 0, 0}, hsvHi2{0, 0, 0};
    bool hsvRange2 = false;
    bool whiteCheck = true;                // line must lie on the white strip
    int whiteSMax = 70, whiteVMin = 150, whiteDilatePx = 12;
    bool whiteAuto = false;                // the white surface may come out darker than whiteVMin (exposure, shade): follow it down
    // A reflection washes the colour out of a glossy tape. With tapeSAuto the saturation limit of the tape colour comes
    // down to just above the white surface's own saturation (never above the configured limit, never below tapeSMin).
    bool tapeSAuto = false;
    int tapeSMin = 34, tapeSMargin = 22;
    double minWidthCm = 1.0, maxWidthCm = 8.0;
    int morphOpen = 3, morphClose = 7;
    int minComponentArea = 60;
    int bands = 12;
    double bandStartM = 0.02, bandStepM = 0.15, bandMaxM = 3.0;
    double maxJumpCm = 30;
    int horizonMarginPx = 12;
    int tagMaskDilatePx = 10;
    void load(const Config& cfg);
};

struct LinePoint {
    double u = 0, v = 0;       // full-resolution pixel
    double X = 0, Y = 0;       // ground, body frame (m)
    double widthCm = 0;
};

struct LineObservation {
    bool valid = false;
    int mode = 0;                     // 1 = band chain, 2 = component axis
    std::vector<LinePoint> pts;       // ordered nearest -> farthest
    double nearX = 0, nearY = 0, farX = 0, farY = 0;
    double headingDeg = 0;            // direction of the line near the drone, + = bending left
    double curvature = 0;             // 1/m, + = left
    double lengthM = 0;
    bool endVisible = false;          // the line ends inside the field of view
    double endX = 0, endY = 0;
    bool endTangentValid = false;     // direction of the line at its end (body frame, + = left)
    double endTangentDeg = 0;
    double AX = 0, AY = 0, BX = 0, BY = 0;   // component endpoints (A nearest to the drone)
    double confidence = 0;
    int maskPixels = 0;
    bool lateralAt(double X, double& Y) const;   // interpolate Y along the chain
};

class LineDetector {
public:
    void configure(const Config& cfg) { p_.load(cfg); }
    LineParams& params() { return p_; }
    const LineParams& params() const { return p_; }

    // maskPolys: full-resolution polygons (e.g. AprilTags) to exclude from the line mask
    // hint: ground point (body frame) where the wanted line is expected to start (e.g. 80 cm after a tag turn)
    LineObservation detect(const cv::Mat& bgr, const GroundProjector& proj,
                           const std::vector<std::vector<cv::Point2f>>& maskPolys, const LineObservation* prev,
                           const cv::Point2d* hint = nullptr);
    const cv::Mat& mask() const { return mask_; }
    const cv::Mat& hsv() const { return hsv_; }          // processing-size HSV image of the last frame
    // steep (mirror) view: only the tape mask is needed, the top-down tracer does the rest
    void maskOnly(const cv::Mat& bgr, const GroundProjector& proj, const std::vector<std::vector<cv::Point2f>>& maskPolys) { buildMask(bgr, proj, maskPolys); }
    // which tape to follow: "dark" | "hsv" | "blue" | "yellow"; false when the name is unknown
    bool setColor(const std::string& name);
    const std::string& color() const { return p_.colorMode; }
    double scale() const { return scale_; }
    void drawDebug(cv::Mat& frame, const LineObservation& obs, const GroundProjector& proj) const;

private:
    void buildMask(const cv::Mat& bgr, const GroundProjector& proj, const std::vector<std::vector<cv::Point2f>>& polys);
    bool scanBands(const GroundProjector& proj, const LineObservation* prev, const cv::Point2d* hint, LineObservation& obs);
    bool scanComponents(const GroundProjector& proj, LineObservation& obs, const cv::Point2d& expect, bool haveExpect = false);
    void finalize(LineObservation& obs) const;
    bool lineContinuesBeyond(const GroundProjector& proj, const std::vector<LinePoint>& chain) const;

    LineParams p_;
    cv::Mat hsv_, mask_, white_;
    int whiteVUsed_ = 0, whiteLevel_ = 0;
public:
    int whiteVUsed() const { return whiteVUsed_; }      // the brightness limit the white test worked with in the last frame
    int whiteLevel() const { return whiteLevel_; }      // brightness of the white surface in the last frame (0 = not measured)
private:
    double scale_ = 1.0;
    struct BandDebug { int row; std::vector<std::pair<int, int>> runs; int chosen; };
    std::vector<BandDebug> bandDbg_;
};
