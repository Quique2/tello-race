#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "vision/TagDetector.h"

// World frame: X east, Y north (meters), heading yaw in degrees counter-clockwise from +X.
struct TrackTag {
    int id = 0; double x = 0, y = 0, yawDeg = 0;
    bool moved = false;                     // the sheet lies elsewhere than in the photo of the course (sim.tag_offsets) ...
    double photoX = 0, photoY = 0;          // ... which shows it here
};
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
    // course built from a photo (tools/course_from_photo.py): the floor is a top-down image instead of a drawing
    std::string worldImage;                 // file with worldSize * pxPerM pixels per side
    double pxPerM = 0;                      // > 0: resolution of worldImage
    bool tagsInImage = false;               // the floor image already shows the real tags: draw none
    double tagPaperW = 0, tagPaperH = 0;    // sheet a drawn tag sits on (m); 0 = letter size
    std::vector<cv::Point2d> tarp;          // outline of the white tarp in the photo (empty: not known)
    // full route in course order (segments joined by straight links): reference for the path-tracking error
    std::vector<cv::Point2d> route() const;
};

struct TrackStyle {
    double pxPerM = 100;
    double tagSizeM = 0.16;                 // black square edge
    double lineWidthCm = 3, stripWidthCm = 10;
    cv::Scalar lineColor{25, 25, 25};
    cv::Scalar carpet{118, 108, 100};
    uint64_t textureSeed = 1;
    cv::Scalar tagBlack{0, 0, 0};           // the black of a drawn tag (the real camera shows a dark blue-grey)
    double floorGain = 1.0;                 // brightness of the photographed floor outside the tarp (the real camera shows it far darker)
};

class TrackBuilder {
public:
    static Track example();                              // layout from the competition slides
    static Track random(uint64_t seed, double worldSize = 9.0);
    // course folder (or its course.txt) written by tools/course_from_photo.py
    static bool fromFile(const std::string& path, Track& t, std::string* err);
    static cv::Mat render(const Track& t, const TrackStyle& s, const TagDetector& tags);
    static cv::Mat renderTags(cv::Mat img, const Track& t, const TrackStyle& s, const TagDetector& tags);
    static cv::Point2d worldToPx(const Track& t, double pxPerM, const cv::Point2d& p);
    // a tag's sheet is loose: put it somewhere else than the course file has it (the segment it ends is cut or extended to it)
    static void moveTag(Track& t, int id, double dx, double dy);
};
