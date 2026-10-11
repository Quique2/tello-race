#pragma once
#include <cmath>
#include <opencv2/core.hpp>

struct Intrinsics {
    double fx = 920, fy = 920, cx = 480, cy = 360;
    int width = 960, height = 720;
    Intrinsics scaled(double s) const {
        Intrinsics k = *this;
        k.fx *= s; k.fy *= s; k.cx *= s; k.cy *= s;
        k.width = (int)std::lround(width * s); k.height = (int)std::lround(height * s);
        return k;
    }
    double hfovDeg() const { return 2.0 * std::atan(0.5 * width / fx) * 180.0 / 3.14159265358979323846; }
    double vfovDeg() const { return 2.0 * std::atan(0.5 * height / fy) * 180.0 / 3.14159265358979323846; }
};

// Maps between full-resolution image pixels and the ground plane, expressed in the drone body frame:
//   X forward, Y left, Z up (meters), origin at the camera.
// Camera frame follows OpenCV: x right, y down, z forward. The camera is mounted pitched down by
// camPitch and the airframe attitude (IMU pitch/roll) is added so the mapping stays correct while the
// drone leans into its motion.
class GroundProjector {
public:
    GroundProjector();
    void setIntrinsics(const Intrinsics& k);
    void setCameraMount(double pitchDownDeg, double rollDeg);
    void setBodyAttitude(double pitchNoseUpDeg, double rollRightDownDeg);
    void setHeight(double meters);

    const Intrinsics& K() const { return K_; }
    double height() const { return H_; }
    double cameraPitchDeg() const { return camPitchDeg_; }
    double cameraRollDeg() const { return camRollDeg_; }
    double totalPitchDeg() const { return camPitchDeg_ - bodyPitchDeg_; }
    double bodyPitchDeg() const { return bodyPitchDeg_; }

    bool pixelToGround(double u, double v, double& X, double& Y) const;       // points ahead of the drone only
    bool pixelToGroundAny(double u, double v, double& X, double& Y) const;    // also under and behind it (steep mirror view)
    bool groundToPixel(double X, double Y, double& u, double& v) const;
    cv::Vec3d pixelRayBody(double u, double v) const;      // unit ray, body frame
    cv::Vec3d camToBody(const cv::Vec3d& p) const { return R_ * p; }
    cv::Vec3d bodyToCam(const cv::Vec3d& p) const { return Rt_ * p; }
    double horizonRow(double u) const;                     // row where the view ray becomes horizontal
    double blindZoneX() const;                              // nearest visible ground distance (bottom-center pixel)
    double metersPerPixel(double u, double v) const;        // ground sampling at a pixel (lateral)

    // Camera tilt (down) and roll and height measured from a tag lying flat on the ground:
    // R_ct = tag-to-camera rotation, t_ct = tag center in camera frame. Returns false if degenerate.
    static bool groundFromTagPose(const cv::Matx33d& R_ct, const cv::Vec3d& t_ct,
                                  double& pitchDownDeg, double& rollDeg, double& heightM);

private:
    void rebuild();
    Intrinsics K_;
    double camPitchDeg_ = 10, camRollDeg_ = 0, bodyPitchDeg_ = 0, bodyRollDeg_ = 0, H_ = 0.5;
    cv::Matx33d R_;     // body_from_cam
    cv::Matx33d Rt_;    // cam_from_body
};
