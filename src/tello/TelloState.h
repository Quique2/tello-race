#pragma once
#include <string>

// Telemetry pushed by the Tello on UDP port 8890 (about 10 Hz), e.g.
// "pitch:0;roll:0;yaw:12;vgx:0;vgy:0;vgz:0;templ:60;temph:63;tof:47;h:40;bat:83;baro:25.3;time:12;agx:-4.0;agy:2.0;agz:-999.0;"
struct TelloState {
    double stamp = 0;         // clock seconds when received
    bool valid = false;
    int pitch = 0, roll = 0, yaw = 0;      // degrees
    int vgx = 0, vgy = 0, vgz = 0;         // speed (drone units, see config tello.speed_scale)
    int templ = 0, temph = 0;              // temperature range, C
    int tof = 0;                           // time-of-flight range to ground, cm
    int h = 0;                             // relative height, cm
    int bat = -1;                          // battery, %
    double baro = 0;                       // barometer, m
    int time = 0;                          // motor-on time, s
    double agx = 0, agy = 0, agz = 0;      // acceleration

    // Best available height in cm (ToF preferred when in a plausible range)
    double heightCm() const;
    static bool parse(const std::string& s, TelloState& out);
    std::string summary() const;
};
