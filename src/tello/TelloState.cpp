#include "tello/TelloState.h"

#include <cstdio>
#include <cstdlib>

double TelloState::heightCm() const {
    if (!valid) return -1;
    if (tof >= 10 && tof <= 400) return tof;
    if (h >= 0 && h <= 400) return h;
    return -1;
}

bool TelloState::parse(const std::string& s, TelloState& out) {
    size_t pos = 0;
    int fields = 0;
    while (pos < s.size()) {
        size_t semi = s.find(';', pos);
        if (semi == std::string::npos) semi = s.size();
        size_t colon = s.find(':', pos);
        if (colon != std::string::npos && colon < semi) {
            std::string key = s.substr(pos, colon - pos);
            const char* val = s.c_str() + colon + 1;
            ++fields;
            if (key == "pitch") out.pitch = atoi(val);
            else if (key == "roll") out.roll = atoi(val);
            else if (key == "yaw") out.yaw = atoi(val);
            else if (key == "vgx") out.vgx = atoi(val);
            else if (key == "vgy") out.vgy = atoi(val);
            else if (key == "vgz") out.vgz = atoi(val);
            else if (key == "templ") out.templ = atoi(val);
            else if (key == "temph") out.temph = atoi(val);
            else if (key == "tof") out.tof = atoi(val);
            else if (key == "h") out.h = atoi(val);
            else if (key == "bat") out.bat = atoi(val);
            else if (key == "baro") out.baro = atof(val);
            else if (key == "time") out.time = atoi(val);
            else if (key == "agx") out.agx = atof(val);
            else if (key == "agy") out.agy = atof(val);
            else if (key == "agz") out.agz = atof(val);
            else --fields;
        }
        pos = semi + 1;
    }
    out.valid = fields >= 5;
    return out.valid;
}

std::string TelloState::summary() const {
    char buf[256];
    snprintf(buf, sizeof(buf), "bat=%d%% tof=%dcm h=%dcm yaw=%d pitch=%d roll=%d v=(%d,%d,%d) temp=%d-%dC t=%ds",
             bat, tof, h, yaw, pitch, roll, vgx, vgy, vgz, templ, temph, time);
    return buf;
}
