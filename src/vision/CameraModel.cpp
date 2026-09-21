#include "vision/CameraModel.h"

#include "util/MathUtil.h"

GroundProjector::GroundProjector() { rebuild(); }

void GroundProjector::setIntrinsics(const Intrinsics& k) { K_ = k; }
void GroundProjector::setCameraMount(double pitchDownDeg, double rollDeg) {
    camPitchDeg_ = pitchDownDeg; camRollDeg_ = rollDeg; rebuild();
}
void GroundProjector::setBodyAttitude(double pitchNoseUpDeg, double rollRightDownDeg) {
    bodyPitchDeg_ = pitchNoseUpDeg; bodyRollDeg_ = rollRightDownDeg; rebuild();
}
void GroundProjector::setHeight(double meters) { H_ = std::max(0.05, meters); }

// R = Rx(roll) * Ry(pitchDown) * P,  P: camera axes -> body axes (x_b = z_c, y_b = -x_c, z_b = -y_c)
void GroundProjector::rebuild() {
    double th = mu::deg2rad(camPitchDeg_ - bodyPitchDeg_);
    double ph = mu::deg2rad(camRollDeg_ + bodyRollDeg_);
    double c = std::cos(th), s = std::sin(th), cp = std::cos(ph), sp = std::sin(ph);
    cv::Matx33d Ry(c, 0, s, 0, 1, 0, -s, 0, c);
    cv::Matx33d Rx(1, 0, 0, 0, cp, -sp, 0, sp, cp);
    cv::Matx33d P(0, 0, 1, -1, 0, 0, 0, -1, 0);
    R_ = Rx * Ry * P;
    Rt_ = R_.t();
}

cv::Vec3d GroundProjector::pixelRayBody(double u, double v) const {
    cv::Vec3d dc((u - K_.cx) / K_.fx, (v - K_.cy) / K_.fy, 1.0);
    cv::Vec3d db = R_ * dc;
    double n = std::sqrt(db.dot(db));
    return n > 0 ? db * (1.0 / n) : db;
}

bool GroundProjector::pixelToGround(double u, double v, double& X, double& Y) const {
    cv::Vec3d d = pixelRayBody(u, v);
    if (d[2] > -1e-3) return false;           // ray does not go down
    double t = -H_ / d[2];
    X = t * d[0];
    Y = t * d[1];
    return X > 0.01 && X < 50.0;
}

bool GroundProjector::groundToPixel(double X, double Y, double& u, double& v) const {
    cv::Vec3d pb(X, Y, -H_);
    cv::Vec3d pc = Rt_ * pb;
    if (pc[2] < 1e-4) return false;
    u = K_.fx * pc[0] / pc[2] + K_.cx;
    v = K_.fy * pc[1] / pc[2] + K_.cy;
    return true;
}

double GroundProjector::horizonRow(double u) const {
    // d_b.z = R20*a + R21*b + R22 = 0 with a=(u-cx)/fx, b=(v-cy)/fy
    double a = (u - K_.cx) / K_.fx;
    double r20 = R_(2, 0), r21 = R_(2, 1), r22 = R_(2, 2);
    if (std::fabs(r21) < 1e-6) return -1e9;
    // a camera looking past vertical (steep mirror) has no sky in the image: the root below is the horizon behind
    if (pixelRayBody(u, 0.0)[2] < 0) return -1e9;
    double b = -(r20 * a + r22) / r21;
    return K_.cy + K_.fy * b;
}

double GroundProjector::blindZoneX() const {
    // distance ahead where the visible floor starts; 0 when the bottom of the image already looks at the floor
    // under or behind the drone (steep mirror view), 1e9 when it does not reach the floor at all
    cv::Vec3d d = pixelRayBody(K_.cx, K_.height - 1);
    if (d[2] > -1e-3) return 1e9;
    double X = -H_ / d[2] * d[0];
    return X > 0.0 ? std::min(X, 1e9) : 0.0;
}

double GroundProjector::metersPerPixel(double u, double v) const {
    double x0, y0, x1, y1;
    if (!pixelToGround(u, v, x0, y0) || !pixelToGround(u + 1, v, x1, y1)) return 1e9;
    return std::sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
}

bool GroundProjector::groundFromTagPose(const cv::Matx33d& R_ct, const cv::Vec3d& t_ct,
                                        double& pitchDownDeg, double& rollDeg, double& heightM) {
    // AprilTag frame: z points into the tag => ground "up" in camera coordinates is -R.col(2).
    cv::Vec3d n(-R_ct(0, 2), -R_ct(1, 2), -R_ct(2, 2));
    double len = std::sqrt(n.dot(n));
    if (len < 1e-6) return false;
    n *= 1.0 / len;
    // With R = Rx(phi)*Ry(theta)*P the body up axis expressed in camera coords is
    // (-sin(phi), -cos(phi)cos(theta), -cos(phi)sin(theta)).
    double phi = std::asin(mu::clamp(-n[0], -1.0, 1.0));
    double theta = std::atan2(-n[2], -n[1]);
    pitchDownDeg = mu::rad2deg(theta);
    rollDeg = mu::rad2deg(phi);
    // Height: distance from the camera to the tag plane along the normal
    heightM = -(t_ct.dot(n));
    return std::isfinite(pitchDownDeg) && std::isfinite(heightM);
}
