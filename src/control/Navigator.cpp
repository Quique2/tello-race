#include "control/Navigator.h"

#include <cmath>
#include <cstdio>

#include "util/Log.h"

void NavParams::load(const Config& c) {
    altitudeCm = c.getDouble("mission.altitude_cm", altitudeCm);
    takeoffSettleS = c.getDouble("mission.takeoff_settle_s", takeoffSettleS);
    maxFlightS = c.getDouble("mission.max_flight_time_s", maxFlightS);
    minBattery = c.getInt("mission.min_battery", minBattery);
    vMax = c.getDouble("follow.v_max", vMax); vMin = c.getDouble("follow.v_min", vMin);
    lookaheadM = c.getDouble("follow.lookahead_m", lookaheadM);
    kYaw = c.getDouble("follow.k_yaw", kYaw); kdYaw = c.getDouble("follow.kd_yaw", kdYaw);
    kTan = c.getDouble("follow.k_tangent", kTan);
    kLat = c.getDouble("follow.k_lat", kLat); kdLat = c.getDouble("follow.kd_lat", kdLat);
    maxYaw = c.getDouble("follow.max_yaw", maxYaw); maxLat = c.getDouble("follow.max_lat", maxLat);
    slowAngleDeg = c.getDouble("follow.slow_angle_deg", slowAngleDeg);
    curvSlow = c.getDouble("follow.curvature_slow", curvSlow);
    rcSlew = c.getDouble("follow.rc_slew", rcSlew);
    lostCoastS = c.getDouble("follow.lost_coast_s", lostCoastS);
    lostHoverS = c.getDouble("follow.lost_hover_s", lostHoverS);
    kAlt = c.getDouble("follow.k_alt", kAlt); maxUd = c.getDouble("follow.max_ud", maxUd);
    altDeadCm = c.getDouble("follow.alt_deadband_cm", altDeadCm);
    tagSlowDistM = c.getDouble("follow.tag_slow_dist_m", tagSlowDistM);
    confirmFrames = c.getInt("tag.confirm_frames", confirmFrames);
    approachDistM = c.getDouble("tag.approach_dist_m", approachDistM);
    settleS = c.getDouble("tag.settle_s", settleS); settleMaxS = c.getDouble("tag.settle_max_s", settleMaxS);
    gapM = c.getDouble("tag.gap_m", gapM);
    lineEndAssocM = c.getDouble("tag.line_end_assoc_m", lineEndAssocM);
    stopOvershootM = c.getDouble("tag.stop_overshoot_m", stopOvershootM);
    curvedApproachDeg = c.getDouble("tag.curved_approach_deg", curvedApproachDeg);
    goSpeed = c.getInt("tag.go_speed", goSpeed);
    goYLeft = c.getBool("tello.go_y_left", goYLeft);
    ignoreVisited = c.getBool("tag.ignore_visited", ignoreVisited);
    turnMode = toLower(c.getString("tag.turn_mode", turnMode));
    rcTurnRate = c.getDouble("tag.rc_turn_rate", rcTurnRate);
    rcTurnTolDeg = c.getDouble("tag.rc_turn_tol_deg", rcTurnTolDeg);
    rcTurnK = c.getDouble("tag.rc_turn_k", rcTurnK);
    alignYawTolDeg = c.getDouble("acquire.align_yaw_tol_deg", alignYawTolDeg);
    alignLatTolCm = c.getDouble("acquire.align_lat_tol_cm", alignLatTolCm);
    vAcquire = c.getDouble("acquire.v_forward", vAcquire);
    acquireTimeoutS = c.getDouble("acquire.timeout_s", acquireTimeoutS);
    rotateInPlaceDeg = c.getDouble("acquire.rotate_in_place_deg", rotateInPlaceDeg);
    followRotateDeg = c.getDouble("follow.rotate_in_place_deg", followRotateDeg);
    coastRc = c.getDouble("gap.coast_rc", coastRc); gapTimeoutS = c.getDouble("gap.timeout_s", gapTimeoutS);
    kYawHold = c.getDouble("gap.k_yaw_hold", kYawHold);
    sweepDeg = c.getDouble("search.sweep_deg", sweepDeg); sweepRate = c.getDouble("search.sweep_rate", sweepRate);
    maxSearchS = c.getDouble("search.max_time_s", maxSearchS);
    backCm = c.getInt("search.back_cm", backCm); riseCm = c.getInt("search.rise_cm", riseCm);
}

Navigator::Navigator(IDrone& drone, IClock& clock, const Config& cfg) : drone_(drone), clock_(clock) {
    p_.load(cfg);
    slewLr_.rate = slewFb_.rate = slewUd_.rate = slewYaw_.rate = p_.rcSlew;
}

const char* Navigator::phaseName(Phase p) {
    switch (p) {
        case Phase::Idle: return "IDLE";
        case Phase::Takeoff: return "TAKEOFF";
        case Phase::Acquire: return "ACQUIRE";
        case Phase::Follow: return "FOLLOW";
        case Phase::TagApproach: return "TAG_APPROACH";
        case Phase::TagManeuver: return "TAG_MANEUVER";
        case Phase::GapCoast: return "GAP_COAST";
        case Phase::Search: return "SEARCH";
        case Phase::Lost: return "LOST";
        case Phase::Finish: return "FINISH";
        case Phase::Landed: return "LANDED";
        case Phase::Aborted: return "ABORTED";
    }
    return "?";
}
const char* Navigator::phaseName() const { return phaseName(phase_); }

void Navigator::setPhase(Phase ph, const char* why) {
    char buf[200];
    snprintf(buf, sizeof(buf), "t=%6.2f  %s -> %s  (%s)", missionTime(), phaseName(phase_), phaseName(ph), why);
    LOG_INFO("NAV %s", buf);
    events.push_back(buf);
    if (events.size() > 12) events.erase(events.begin());
    phase_ = ph;
    phaseStart_ = now_;
    haveTarget = false;
}

double Navigator::missionTime() const { return missionStart_ > 0 ? now_ - missionStart_ : 0; }

std::string Navigator::status() const {
    char buf[256];
    snprintf(buf, sizeof(buf), "%s  seg=%d  t=%.1fs  tag=%s%d(%d) X=%.2f  rc=[%d %d %d %d]", phaseName(), segmentsDone_,
             missionTime(), tag_.confirmed ? "*" : "", tag_.id, tag_.count, tag_.X, rc_.lr, rc_.fb, rc_.ud, rc_.yaw);
    return buf;
}

void Navigator::start(int startAtTag) {
    now_ = clock_.now();
    startAtTag_ = startAtTag;
    segmentsDone_ = 0;
    visited_.clear();
    if (startAtTag > 0) visited_.insert(startAtTag);
    forwardTagAhead_ = false;
    tag_ = TagTrack();
    landRequested_ = false;
    missionStart_ = now_;
    setPhase(Phase::Takeoff, startAtTag ? "resume at checkpoint" : "start");
}

void Navigator::requestLand() { landRequested_ = true; }

void Navigator::emergencyStop() {
    LOG_ERROR("NAV emergency stop!");
    drone_.emergency();
    setPhase(Phase::Aborted, "emergency");
}

void Navigator::sendRc(double lr, double fb, double ud, double yaw, bool instant) {
    double dt = 0.033;
    if (instant) { slewLr_.reset(lr); slewFb_.reset(fb); slewUd_.reset(ud); slewYaw_.reset(yaw); }
    else { lr = slewLr_.push(lr, dt); fb = slewFb_.push(fb, dt); ud = slewUd_.push(ud, dt); yaw = slewYaw_.push(yaw, dt); }
    rc_.lr = mu::clamp(mu::roundInt(lr), -100, 100);
    rc_.fb = mu::clamp(mu::roundInt(fb), -100, 100);
    rc_.ud = mu::clamp(mu::roundInt(ud), -100, 100);
    rc_.yaw = mu::clamp(mu::roundInt(yaw), -100, 100);
    drone_.rc(rc_.lr, rc_.fb, rc_.ud, rc_.yaw);
}

void Navigator::hover(bool instant) { sendRc(0, 0, 0, 0, instant); }

double Navigator::altitudeUd(const TelloState& st) {
    double h = st.heightCm();
    if (h < 0) return 0;
    double err = p_.altitudeCm - h;
    if (std::fabs(err) < p_.altDeadCm) return 0;
    return mu::clamp(p_.kAlt * err, -p_.maxUd, p_.maxUd);
}

// ---------------------------------------------------------------------------
// Tag tracking: keep one candidate (the tag at the end of the line being followed) and confirm it over frames.
// ---------------------------------------------------------------------------
void Navigator::updateTagTrack(const Observation& obs) {
    const TagObservation* best = nullptr;
    for (const auto& t : obs.tags) {
        if (t.id < 1 || t.id > 4) continue;
        if (p_.ignoreVisited && visited_.count(t.id)) continue;
        // must be the tag at the end of the current line: reject tags far behind the visible line end
        if (obs.line.valid && obs.line.mode == 1 && obs.line.farX > t.X + p_.lineEndAssocM) continue;
        if (!best || t.margin > best->margin) best = &t;
    }
    if (best) {
        if (best->id == tag_.id && now_ - tag_.lastSeen < 1.0) {
            tag_.count++;
            tag_.X += 0.5 * (best->X - tag_.X);
            tag_.Y += 0.5 * (best->Y - tag_.Y);
        } else {
            tag_ = TagTrack();
            tag_.id = best->id; tag_.count = 1; tag_.X = best->X; tag_.Y = best->Y;
        }
        tag_.lastSeen = now_;
        if (tag_.id == 1 && tag_.confirmed) lastFwdTagX_ = tag_.X;
        if (!tag_.confirmed && tag_.count >= p_.confirmFrames) {
            tag_.confirmed = true;
            LOG_INFO("NAV tag ID%d confirmed at X=%.2f Y=%.2f (%s)", tag_.id, tag_.X, tag_.Y,
                     tag_.id == 1 ? "FORWARD" : tag_.id == 2 ? "LEFT" : tag_.id == 3 ? "RIGHT" : "FINISH");
        }
    } else if (tag_.id > 0 && now_ - tag_.lastSeen > 0.6) {
        // Track lost. A confirmed FORWARD tag that disappeared under the nose counts as passed.
        if (tag_.confirmed && tag_.id == 1 && tag_.X < blindZone_ + 0.5 && !visited_.count(1)) {
            visited_.insert(1);
            ++segmentsDone_;
            LOG_INFO("NAV passed FORWARD tag (segments=%d)", segmentsDone_);
            forwardTagAhead_ = false;
        }
        tag_ = TagTrack();
    }
}

// The direction of the line at its visible end, stored as an absolute yaw (telemetry convention, clockwise
// positive). "Forward / left / right" at a tag are relative to this direction, not to the drone heading,
// which can lag it by tens of degrees after a bend hidden in the camera blind zone.
void Navigator::updateEndTangent(const Observation& obs, const TelloState& st) {
    const LineObservation& l = obs.line;
    if (!l.valid || !l.endVisible || !l.endTangentValid || !st.valid || l.endX > 2.5) return;
    if (tag_.id > 0 && now_ - tag_.lastSeen < 0.5 && std::hypot(l.endX - tag_.X, l.endY - tag_.Y) > 0.7) return;
    double abs = mu::wrapDeg(st.yaw - l.endTangentDeg);
    endYawSin_ = 0.7 * endYawSin_ + 0.3 * std::sin(mu::deg2rad(abs));
    endYawCos_ = 0.7 * endYawCos_ + 0.3 * std::cos(mu::deg2rad(abs));
    endAbsYaw_ = mu::rad2deg(std::atan2(endYawSin_, endYawCos_));
    endYawStamp_ = now_;
    endYawValid_ = true;
}

double Navigator::endTangentRelDeg(const TelloState& st) const {   // turn (cw +) needed to face the line end direction
    if (!endYawValid_ || !st.valid || now_ - endYawStamp_ > 20.0) return 0.0;
    return mu::wrapDeg(endAbsYaw_ - st.yaw);
}

bool Navigator::tagConfirmed() const { return tag_.confirmed && now_ - tag_.lastSeen < 0.5; }

// Point on the line polyline at 'lookahead' path length from its first point.
bool Navigator::pursuitTarget(const LineObservation& line, double& tx, double& ty, double* tangentDeg) const {
    if (!line.valid || line.pts.empty()) return false;
    double remaining = lookahead_ > 0 ? lookahead_ : p_.lookaheadM;
    tx = line.pts.front().X; ty = line.pts.front().Y;
    if (tangentDeg) *tangentDeg = line.headingDeg;
    for (size_t i = 1; i < line.pts.size(); ++i) {
        double dx = line.pts[i].X - line.pts[i - 1].X, dy = line.pts[i].Y - line.pts[i - 1].Y;
        double seg = std::hypot(dx, dy);
        if (seg > 1e-6 && tangentDeg) *tangentDeg = mu::rad2deg(std::atan2(dy, dx));
        if (seg >= remaining) {
            double f = seg > 1e-6 ? remaining / seg : 0;
            tx = line.pts[i - 1].X + dx * f; ty = line.pts[i - 1].Y + dy * f;
            return true;
        }
        remaining -= seg;
        tx = line.pts[i].X; ty = line.pts[i].Y;
    }
    return true;
}

// ---------------------------------------------------------------------------
void Navigator::step(const Observation& obs, const TelloState& st) {
    now_ = clock_.now();
    lastStep_ = now_;
    blindZone_ = obs.blindZoneM > 0 && obs.blindZoneM < 5 ? obs.blindZoneM : blindZone_;
    if (obs.line.valid) { lastLine_ = obs.line; lastLineSeen_ = now_; }
    if (phase_ != Phase::Idle && !finished()) updateTagTrack(obs);
    updateEndTangent(obs, st);

    if (landRequested_ && !finished() && phase_ != Phase::Finish) {
        hover(true);
        setPhase(Phase::Aborted, "land requested");
        drone_.land();
        return;
    }
    if (p_.maxFlightS > 0 && flying() && missionTime() > p_.maxFlightS) {
        LOG_WARN("NAV max flight time reached -> landing");
        hover(true);
        setPhase(Phase::Aborted, "max flight time");
        drone_.land();
        return;
    }
    switch (phase_) {
        case Phase::Idle: break;
        case Phase::Takeoff: doTakeoff(st); break;
        case Phase::Acquire: doAcquire(obs, st); break;
        case Phase::Follow: doFollow(obs, st); break;
        case Phase::TagApproach: doTagApproach(obs, st); break;
        case Phase::TagManeuver: doTagManeuver(obs, st); break;
        case Phase::GapCoast: doGapCoast(obs, st); break;
        case Phase::Search: doSearch(obs, st); break;
        case Phase::Lost:
            sendRc(0, 0, altitudeUd(st), 0);
            if (obs.line.valid) beginAcquire("line reappeared");
            break;
        case Phase::Finish: doFinish(); break;
        case Phase::Landed:
        case Phase::Aborted: break;
    }
}

void Navigator::doTakeoff(const TelloState& st) {
    if (st.valid && st.bat >= 0 && st.bat < p_.minBattery) {
        LOG_ERROR("NAV battery %d%% below minimum %d%% -> not taking off", st.bat, p_.minBattery);
        setPhase(Phase::Aborted, "low battery");
        return;
    }
    if (!drone_.takeoff()) {
        setPhase(Phase::Aborted, "takeoff failed");
        return;
    }
    missionStart_ = clock_.now();
    now_ = missionStart_;
    clock_.sleep(0.5);
    TelloState s = drone_.state();
    double h = s.heightCm();
    double diff = p_.altitudeCm - h;
    if (h > 0 && std::fabs(diff) >= 20) {
        LOG_INFO("NAV adjusting altitude %.0f -> %.0f cm", h, p_.altitudeCm);
        drone_.move(diff > 0 ? "up" : "down", std::min(200, (int)std::fabs(diff)));
    }
    hover(true);
    clock_.sleep(p_.takeoffSettleS);
    now_ = clock_.now();
    if (startAtTag_ == 2 || startAtTag_ == 3) {
        pendingDir_ = startAtTag_;
        beginTurn(startAtTag_ == 2 ? -90 : 90, drone_.state());
        if (!turning_) beginAcquire("resume: turned at checkpoint");
    } else if (startAtTag_ == 1) {
        beginGap("resume: forward checkpoint", drone_.state());
    } else {
        beginAcquire("airborne");
    }
}

// ---------------------------------------------------------------------------
// ACQUIRE: a line is expected ahead (start line after takeoff, or the next segment after a tag turn).
// Rotate/translate until the line runs straight ahead, then hand over to FOLLOW.
// ---------------------------------------------------------------------------
void Navigator::beginAcquire(const char* why) {
    setPhase(Phase::Acquire, why);
    expectX_ = p_.gapM; expectY_ = 0;
    prevAngle_ = prevLat_ = 0;
    slewLr_.reset(); slewFb_.reset(); slewYaw_.reset();
}

void Navigator::doAcquire(const Observation& obs, const TelloState& st) {
    double ud = altitudeUd(st);
    if (tagConfirmed() && tag_.id != 1 && tag_.X < p_.approachDistM) { setPhase(Phase::TagApproach, "tag ahead during acquire"); return; }
    if (!obs.line.valid) {
        if (now_ - phaseStart_ > p_.acquireTimeoutS) { beginSearch("no line to acquire", st); return; }
        sendRc(0, 0, ud, 0);
        return;
    }
    LineObservation line = obs.line;
    if (line.mode == 2) {
        // choose the endpoint closest to where the segment should start as the entry point
        double dA = std::hypot(line.AX - expectX_, line.AY - expectY_);
        double dB = std::hypot(line.BX - expectX_, line.BY - expectY_);
        if (dB < dA) { std::reverse(line.pts.begin(), line.pts.end()); std::swap(line.AX, line.BX); std::swap(line.AY, line.BY); }
    }
    double tx, ty;
    if (!pursuitTarget(line, tx, ty)) { sendRc(0, 0, ud, 0); return; }
    targetX = tx; targetY = ty; haveTarget = true;
    double angle = mu::rad2deg(std::atan2(ty, tx));           // + = target to the left
    double lat = line.pts.front().Y;
    double lr, fb, yaw;
    if (std::fabs(angle) > p_.rotateInPlaceDeg) {
        yaw = -mu::sign(angle) * mu::clamp(p_.kYaw * std::fabs(angle), 20.0, p_.maxYaw);
        lr = 0; fb = 0;
    } else {
        yaw = mu::clamp(-p_.kYaw * angle, -p_.maxYaw, p_.maxYaw);
        lr = mu::clamp(-p_.kLat * lat * 100.0, -p_.maxLat, p_.maxLat);
        fb = p_.vAcquire * mu::clamp(1.0 - std::fabs(angle) / p_.rotateInPlaceDeg, 0.2, 1.0);
        if (line.pts.front().X < blindZone_ + 0.1 && std::fabs(lat) > 0.15) fb *= 0.5;
    }
    sendRc(lr, fb, ud, yaw);
    double lineDir = line.mode == 1 ? line.headingDeg : mu::rad2deg(std::atan2(line.BY - line.AY, line.BX - line.AX));
    bool aligned = std::fabs(angle) < p_.alignYawTolDeg && std::fabs(lat) * 100.0 < p_.alignLatTolCm && std::fabs(lineDir) < 30 &&
                   line.mode == 1 && line.pts.size() >= 2 && line.pts.front().X < blindZone_ + 0.6;
    if (aligned) { setPhase(Phase::Follow, "aligned with line"); prevAngle_ = angle; prevLat_ = lat; }
}

// ---------------------------------------------------------------------------
// FOLLOW: pure-pursuit steering on the projected line, lateral correction, speed adapted to geometry.
// ---------------------------------------------------------------------------
void Navigator::doFollow(const Observation& obs, const TelloState& st) {
    double ud = altitudeUd(st);
    double vScale = 1.0;
    if (tagConfirmed()) {
        if (tag_.id == 1) {
            forwardTagAhead_ = true;
            lastFwdTagX_ = tag_.X;
            if (tag_.X < p_.approachDistM && std::fabs(endTangentRelDeg(st)) > p_.curvedApproachDeg) {
                pendingDir_ = 1;
                setPhase(Phase::TagApproach, "FORWARD tag after a bend: precise maneuver");
                return;
            }
        } else {
            if (tag_.X < p_.approachDistM) { pendingDir_ = tag_.id; setPhase(Phase::TagApproach, "tag within approach distance"); return; }
            double span = std::max(0.2, p_.tagSlowDistM - p_.approachDistM);
            vScale = mu::clamp((tag_.X - p_.approachDistM) / span, 0.35, 1.0);
        }
    }
    if (obs.line.valid) {
        double tx, ty;
        double curv = std::fabs(obs.line.curvature);
        lookahead_ = p_.lookaheadM * mu::clamp(1.0 - curv / 2.5, 0.4, 1.0);   // shorter look-ahead in bends
        double tangent = 0;
        pursuitTarget(obs.line, tx, ty, &tangent);
        targetX = tx; targetY = ty; haveTarget = true;
        double angle = mu::rad2deg(std::atan2(ty, tx));
        double lat = obs.line.pts.front().Y;
        double dt = 0.033;
        double dAngle = (angle - prevAngle_) / dt, dLat = (lat - prevLat_) / dt;
        prevAngle_ = angle; prevLat_ = lat;
        // bearing to the pursuit point brings the drone onto the line, the tangent term aligns it with the
        // line direction so the heading at a segment end matches where the next segment will be
        double yaw = mu::clamp(-(p_.kYaw * angle + p_.kTan * tangent + p_.kdYaw * dAngle), -p_.maxYaw, p_.maxYaw);
        double lr = mu::clamp(-(p_.kLat * lat * 100.0 + p_.kdLat * dLat * 100.0), -p_.maxLat, p_.maxLat);
        double f = mu::clamp(1.0 - std::max(std::fabs(angle), 0.5 * std::fabs(tangent)) / p_.slowAngleDeg, 0.0, 1.0);
        double cf = mu::clamp(1.0 - curv / p_.curvSlow, 0.3, 1.0);
        double v = p_.vMin + (p_.vMax - p_.vMin) * f * cf;
        if (std::fabs(angle) > p_.followRotateDeg) {   // sharp bend: turn first, creep only
            v = 0;
            yaw = -mu::sign(angle) * mu::clamp(p_.kYaw * std::fabs(angle), 20.0, p_.maxYaw);
        }
        if (obs.line.endVisible && !forwardTagAhead_) v *= mu::clamp((obs.line.endX - blindZone_) / 1.0 + 0.4, 0.4, 1.0);
        if (obs.line.mode == 2 || obs.line.pts.front().X > blindZone_ + 0.9) v = std::min(v, p_.vAcquire);   // line not right ahead: approach carefully
        sendRc(lr, v * vScale, ud, yaw);
        return;
    }
    // line lost
    double lost = now_ - lastLineSeen_;
    // The segment really ends only if the last chain was heading straight; a chain that ended while bending
    // most likely curved out of the field of view -> turn toward it (search) instead of coasting.
    bool straightEnd = std::fabs(lastLine_.headingDeg) < 30 && std::fabs(prevAngle_) < 20 && std::fabs(lastLine_.curvature) < 1.2;
    bool atLineEnd = lastLine_.endVisible && lastLine_.endX < blindZone_ + 0.7 && straightEnd;
    bool tagUnderNose = forwardTagAhead_ && lastFwdTagX_ < blindZone_ + 0.7 && std::fabs(prevAngle_) < 25;
    if (lost < p_.lostCoastS) {
        sendRc(rc_.lr * 0.5, rc_.fb * 0.6, ud, rc_.yaw * 0.5);
    } else if (lost < p_.lostHoverS && !atLineEnd && !tagUnderNose) {
        sendRc(0, 0, ud, 0);
    } else if (atLineEnd || tagUnderNose) {
        beginGap(tagUnderNose ? "line ended at FORWARD tag" : "line ended", st);
    } else {
        beginSearch("line lost (bend?)", st);
    }
}

// ---------------------------------------------------------------------------
// TAG APPROACH: stop, let the drone settle, measure the tag position precisely, then maneuver.
// ---------------------------------------------------------------------------
void Navigator::doTagApproach(const Observation& obs, const TelloState& st) {
    if (approachStart_ < phaseStart_) {
        approachStart_ = now_;
        apX_.clear(); apY_.clear(); apT_.clear();
        hover(true);
    }
    sendRc(0, 0, altitudeUd(st), 0, true);
    if (const TagObservation* t = obs.tag(tag_.id)) { apX_.push_back(t->X); apY_.push_back(t->Y); apT_.push_back(now_); }
    double elapsed = now_ - approachStart_;
    std::vector<double> xs, ys;
    for (size_t i = 0; i < apT_.size(); ++i)
        if (apT_[i] >= approachStart_ + 0.5 * p_.settleS) { xs.push_back(apX_[i]); ys.push_back(apY_[i]); }
    bool ready = (elapsed >= p_.settleS && xs.size() >= 3) || elapsed >= p_.settleMaxS;
    if (!ready) return;
    if (!xs.empty()) { maneuverX_ = mu::median(xs); maneuverY_ = mu::median(ys); }
    else if (!apX_.empty()) { maneuverX_ = apX_.back() - p_.stopOvershootM; maneuverY_ = apY_.back(); }
    else { maneuverX_ = tag_.X - p_.stopOvershootM; maneuverY_ = tag_.Y; }
    pendingDir_ = tag_.id;
    LOG_INFO("NAV tag ID%d measured at X=%.2f Y=%.2f (%zu settled samples)", tag_.id, maneuverX_, maneuverY_, xs.size());
    setPhase(Phase::TagManeuver, "tag measured");
}

// ---------------------------------------------------------------------------
// TAG MANEUVER: fly over the tag (VPS-based "go"), then turn according to the tag, or land on FINISH.
// ---------------------------------------------------------------------------
void Navigator::beginTurn(int deg, const TelloState& st) {
    turning_ = false;
    if (deg == 0) return;
    if (p_.turnMode == "rc" && st.valid) {
        turning_ = true;
        turnTargetYaw_ = (int)mu::wrapDeg(st.yaw + deg);
        turnStart_ = now_;
        turnStableSince_ = -1;
        LOG_INFO("NAV rc turn %d deg: yaw %d -> %d", deg, st.yaw, turnTargetYaw_);
        return;
    }
    LOG_INFO("NAV sdk turn %d deg", deg);
    hover(true);
    drone_.rotateCw(deg);
    hover(true);
    now_ = clock_.now();
}

void Navigator::doTagManeuver(const Observation& obs, const TelloState& st) {
    if (turning_) {
        double err = mu::wrapDeg(turnTargetYaw_ - st.yaw);          // state yaw: + = clockwise
        double yaw = mu::clamp(p_.rcTurnK * err, -p_.rcTurnRate, p_.rcTurnRate);
        if (std::fabs(err) > p_.rcTurnTolDeg && std::fabs(yaw) < 15) yaw = 15 * mu::sign(err);
        if (std::fabs(err) <= p_.rcTurnTolDeg) {
            yaw = 0;
            if (turnStableSince_ < 0) turnStableSince_ = now_;
        } else {
            turnStableSince_ = -1;
        }
        sendRc(0, 0, altitudeUd(st), yaw, true);
        if ((turnStableSince_ > 0 && now_ - turnStableSince_ > 0.3) || now_ - turnStart_ > 6.0) {
            turning_ = false;
            hover(true);
            beginAcquire(now_ - turnStart_ > 6.0 ? "rc turn timeout" : "rc turn complete");
        }
        return;
    }
    int dir = pendingDir_;
    int dx = mu::roundInt(maneuverX_ * 100.0), dy = mu::roundInt(maneuverY_ * 100.0);
    int goY = p_.goYLeft ? dy : -dy;
    if (std::abs(dx) >= 20 || std::abs(dy) >= 20) {
        dx = mu::clamp(dx, -500, 500); goY = mu::clamp(goY, -500, 500);
        LOG_INFO("NAV go over tag: go %d %d 0 %d", dx, goY, p_.goSpeed);
        hover(true);
        if (!drone_.go(dx, goY, 0, p_.goSpeed)) {
            LOG_WARN("NAV go failed (%s) -> fallback to axis moves", drone_.lastReply.c_str());
            if (std::abs(dx) >= 20) drone_.move(dx > 0 ? "forward" : "back", std::min(500, std::abs(dx)));
            if (std::abs(dy) >= 20) drone_.move(dy > 0 ? "left" : "right", std::min(500, std::abs(dy)));
        }
        now_ = clock_.now();
    }
    visited_.insert(dir);
    ++segmentsDone_;
    tag_ = TagTrack();
    forwardTagAhead_ = false;
    LOG_INFO("NAV reached tag ID%d (segments=%d, t=%.1fs)", dir, segmentsDone_, missionTime());
    if (dir == 4) { setPhase(Phase::Finish, "FINISH tag reached"); return; }
    TelloState sNow = drone_.state();
    double rel = endTangentRelDeg(sNow);
    int deg = mu::roundInt(mu::wrapDeg(rel + (dir == 2 ? -90 : (dir == 3 ? 90 : 0))));
    LOG_INFO("NAV turn for tag ID%d: line end direction %+.0f deg relative -> turn %d deg", dir, rel, deg);
    endYawValid_ = false;
    if (std::abs(deg) < 4) { beginAcquire("already facing the next segment"); return; }
    beginTurn(deg, sNow);
    if (!turning_) beginAcquire("turned toward next segment");
}

// ---------------------------------------------------------------------------
// GAP COAST: fly straight holding the heading until the next line shows up (after a FORWARD tag or a line end).
// ---------------------------------------------------------------------------
void Navigator::beginGap(const char* why, const TelloState& st) {
    setPhase(Phase::GapCoast, why);
    gapYawValid_ = st.valid;
    gapYaw0_ = (endYawValid_ && now_ - endYawStamp_ < 20.0) ? mu::roundInt(endAbsYaw_) : st.yaw;
    if (endYawValid_) LOG_INFO("NAV coasting on the line end heading (%+.0f deg from current)", endTangentRelDeg(st));
    slewFb_.reset(rc_.fb);
}

void Navigator::doGapCoast(const Observation& obs, const TelloState& st) {
    if (tagConfirmed() && tag_.id != 1 && tag_.X < p_.approachDistM) { pendingDir_ = tag_.id; setPhase(Phase::TagApproach, "tag ahead during gap"); return; }
    if (obs.line.valid) {
        if (forwardTagAhead_ && !visited_.count(1)) {   // the FORWARD tag segment is done once the next line shows up
            visited_.insert(1);
            ++segmentsDone_;
            forwardTagAhead_ = false;
            LOG_INFO("NAV passed FORWARD tag (segments=%d)", segmentsDone_);
        }
        if (obs.line.mode == 1 && std::fabs(obs.line.pts.front().Y) < 0.25 && std::fabs(obs.line.headingDeg) < 25) {
            setPhase(Phase::Follow, "next line ahead");
            prevAngle_ = prevLat_ = 0;
        } else {
            beginAcquire("next line found");
        }
        return;
    }
    double yaw = 0, lr = 0;
    if (gapYawValid_ && st.valid) yaw = mu::clamp(p_.kYawHold * mu::wrapDeg(gapYaw0_ - st.yaw), -20.0, 20.0);
    if (tag_.id == 1 && tag_.confirmed && now_ - tag_.lastSeen < 0.4 && tag_.X > 0.2) {
        // keep the FORWARD tag straight ahead: the next segment starts 80 cm beyond it
        double bearing = mu::rad2deg(std::atan2(tag_.Y, tag_.X));
        yaw = mu::clamp(-0.6 * p_.kYaw * bearing, -25.0, 25.0);
        lr = mu::clamp(-0.5 * p_.kLat * tag_.Y * 100.0, -15.0, 15.0);
        if (gapYawValid_ && st.valid) gapYaw0_ = st.yaw;
    }
    sendRc(lr, p_.coastRc, altitudeUd(st), yaw);
    if (now_ - phaseStart_ > p_.gapTimeoutS) beginSearch("gap timeout", st);
}

// ---------------------------------------------------------------------------
// SEARCH: bounded recovery pattern. Never flies away blindly: step back, sweep yaw, rise, repeat, then hold.
// ---------------------------------------------------------------------------
void Navigator::beginSearch(const char* why, const TelloState& st) {
    searchFromAcquire_ = (phase_ == Phase::Acquire);
    setPhase(Phase::Search, why);
    hover(true);
    searchStart_ = now_;
    searchStep_ = 0;
    searchStepStart_ = now_;
    searchSweeps_ = 0;
    sweepDir_ = 1;
    searchDidBack_ = false;
    searchDidRise_ = false;
    searchYawValid_ = st.valid;
    searchYaw0_ = st.yaw;
    // a line that bent away is found by turning toward it first (pursuit angle > 0 = it went left = yaw ccw = negative state yaw)
    searchBentAway_ = std::fabs(prevAngle_) > 15 && now_ - lastLineSeen_ < 2.0;
    sweepFirst_ = prevAngle_ > 0 ? -1 : 1;
    sweepDir_ = sweepFirst_;
}

void Navigator::doSearch(const Observation& obs, const TelloState& st) {
    if (obs.line.valid) { hover(true); beginAcquire("line found while searching"); return; }
    if (tagConfirmed() && tag_.id != 1 && tag_.X < p_.approachDistM) { pendingDir_ = tag_.id; setPhase(Phase::TagApproach, "tag found while searching"); return; }
    if (now_ - searchStart_ > p_.maxSearchS) { hover(true); setPhase(Phase::Lost, "search exhausted, holding position"); return; }
    double ud = altitudeUd(st);
    double sinceStep = now_ - searchStepStart_;
    switch (searchStep_) {
        case 0: {   // step back so the expected start point enters the visible ground area
            bool needBack = (blindZone_ > p_.gapM - 0.1 || !searchFromAcquire_) && !searchBentAway_;
            if (needBack && !searchDidBack_ && p_.backCm >= 20) {
                LOG_INFO("NAV search: back %d cm", p_.backCm);
                drone_.move("back", p_.backCm);
                searchDidBack_ = true;
                now_ = clock_.now();
            }
            searchStep_ = 1; searchStepStart_ = now_; sweepDir_ = sweepFirst_;
            sendRc(0, 0, ud, 0, true);
            return;
        }
        case 1: {   // sweep first toward the likely side, then the other side, then back to the centre (wider every cycle)
            double sweep = std::min(100.0, p_.sweepDeg * (1.0 + 0.5 * searchSweeps_));
            double target = searchYaw0_ + sweepDir_ * sweep;
            bool reached = false;
            double yaw = 0;
            if (searchYawValid_ && st.valid) {
                double err = mu::wrapDeg(target - st.yaw);
                yaw = mu::clamp(1.5 * err, -p_.sweepRate, p_.sweepRate);
                if (std::fabs(err) > 3 && std::fabs(yaw) < 15) yaw = 15 * mu::sign(err);
                reached = std::fabs(err) <= 3;
            } else {
                double legS = sweep / std::max(10.0, p_.sweepRate);
                int dir = sweepDir_ == 0 ? sweepFirst_ : sweepDir_;
                yaw = dir * p_.sweepRate;
                reached = sinceStep > (sweepDir_ == sweepFirst_ ? legS : (sweepDir_ == 0 ? legS : 2 * legS));
            }
            sendRc(0, 0, ud, reached ? 0 : yaw, true);
            if (reached) {
                searchStepStart_ = now_;
                if (sweepDir_ == 0) { searchSweeps_++; searchStep_ = 2; sweepDir_ = sweepFirst_; }
                else if (sweepDir_ == sweepFirst_) sweepDir_ = -sweepFirst_;
                else sweepDir_ = 0;
            }
            return;
        }
        case 2: {   // rise once to see farther, later alternate stepping back and sweeping
            if (!searchDidRise_ && p_.riseCm >= 20) {
                LOG_INFO("NAV search: up %d cm", p_.riseCm);
                drone_.move("up", p_.riseCm);
                searchDidRise_ = true;
                now_ = clock_.now();
            } else if (p_.backCm >= 20) {
                LOG_INFO("NAV search: back %d cm", p_.backCm);
                drone_.move("back", p_.backCm);
                now_ = clock_.now();
            }
            searchStep_ = 1; searchStepStart_ = now_; sweepDir_ = sweepFirst_;
            sendRc(0, 0, ud, 0, true);
            return;
        }
    }
}

void Navigator::doFinish() {
    hover(true);
    LOG_INFO("NAV landing on FINISH tag, mission time %.1fs", missionTime());
    drone_.land();
    now_ = clock_.now();
    LOG_INFO("NAV landed. Total time %.1fs, segments %d", missionTime(), segmentsDone_);
    setPhase(Phase::Landed, "landed");
}
