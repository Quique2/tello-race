#include "vision/TagDetector.h"

#include <chrono>

extern "C" {
#include "apriltag.h"
#include "apriltag_pose.h"
#include "common/image_u8.h"
#include "common/matd.h"
#include "tag16h5.h"
#include "tag25h9.h"
#include "tag36h11.h"
#include "tagStandard41h12.h"
}

#include "util/Log.h"

double TagDetection::sizePx() const {
    double s = 0;
    for (int i = 0; i < 4; ++i) {
        const cv::Point2d& a = corners[i];
        const cv::Point2d& b = corners[(i + 1) % 4];
        s += std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    }
    return s / 4.0;
}

TagDetector::~TagDetector() {
    if (td_) apriltag_detector_destroy(td_);
    if (tf_) {
        if (family_ == "tag36h11") tag36h11_destroy(tf_);
        else if (family_ == "tag25h9") tag25h9_destroy(tf_);
        else if (family_ == "tag16h5") tag16h5_destroy(tf_);
        else if (family_ == "tagStandard41h12") tagStandard41h12_destroy(tf_);
    }
}

bool TagDetector::init(const std::string& family, double quadDecimate, double quadSigma, int threads,
                       bool refineEdges, double decodeSharpening, std::string* err) {
    family_ = family;
    if (family == "tag36h11") tf_ = tag36h11_create();
    else if (family == "tag25h9") tf_ = tag25h9_create();
    else if (family == "tag16h5") tf_ = tag16h5_create();
    else if (family == "tagStandard41h12") tf_ = tagStandard41h12_create();
    else { if (err) *err = "unknown AprilTag family: " + family; return false; }
    td_ = apriltag_detector_create();
    apriltag_detector_add_family(td_, tf_);
    td_->quad_decimate = (float)quadDecimate;
    td_->quad_sigma = (float)quadSigma;
    td_->nthreads = threads;
    td_->refine_edges = refineEdges ? 1 : 0;
    td_->decode_sharpening = decodeSharpening;
    td_->debug = 0;
    LOG_INFO("AprilTag detector: family=%s decimate=%.1f sigma=%.1f threads=%d", family.c_str(), quadDecimate,
             quadSigma, threads);
    return true;
}

std::vector<TagDetection> TagDetector::detect(const cv::Mat& grayIn, const Intrinsics* K, double tagSizeM) {
    std::vector<TagDetection> out;
    if (!td_ || grayIn.empty()) return out;
    auto t0 = std::chrono::steady_clock::now();
    cv::Mat gray = grayIn.isContinuous() ? grayIn : grayIn.clone();
    image_u8_t im{gray.cols, gray.rows, (int32_t)gray.step[0], gray.data};
    zarray_t* dets = apriltag_detector_detect(td_, &im);
    for (int i = 0; i < zarray_size(dets); ++i) {
        apriltag_detection_t* det = nullptr;
        zarray_get(dets, i, &det);
        TagDetection d;
        d.id = det->id;
        d.hamming = det->hamming;
        d.margin = det->decision_margin;
        d.center = cv::Point2d(det->c[0], det->c[1]);
        for (int k = 0; k < 4; ++k) d.corners[k] = cv::Point2d(det->p[k][0], det->p[k][1]);
        if (K && tagSizeM > 0) {
            apriltag_detection_info_t info;
            info.det = det;
            info.tagsize = tagSizeM;
            info.fx = K->fx; info.fy = K->fy; info.cx = K->cx; info.cy = K->cy;
            apriltag_pose_t pose;
            d.poseErr = estimate_tag_pose(&info, &pose);
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c) d.R(r, c) = MATD_EL(pose.R, r, c);
            for (int r = 0; r < 3; ++r) d.t[r] = MATD_EL(pose.t, r, 0);
            matd_destroy(pose.R);
            matd_destroy(pose.t);
            d.hasPose = true;
        }
        out.push_back(d);
    }
    apriltag_detections_destroy(dets);
    lastMs_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return out;
}

cv::Mat TagDetector::renderTag(int id) const {
    if (!tf_ || id < 0 || id >= (int)tf_->ncodes) return cv::Mat();
    image_u8_t* im = apriltag_to_image(tf_, id);
    if (!im) return cv::Mat();
    cv::Mat m(im->height, im->width, CV_8UC1);
    for (int y = 0; y < im->height; ++y)
        for (int x = 0; x < im->width; ++x) m.at<uint8_t>(y, x) = im->buf[y * im->stride + x];
    image_u8_destroy(im);
    return m;
}

double TagDetector::blackSquareFraction() const {
    if (!tf_) return 0.8;
    // total_width includes the 1-module white border on each side for the classic families
    return (double)(tf_->width_at_border) / (double)tf_->total_width;
}
