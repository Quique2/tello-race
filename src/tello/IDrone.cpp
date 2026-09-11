#include "tello/IDrone.h"

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <algorithm>

#include "util/Log.h"

bool IDrone::takeoff() { return command("takeoff", &lastReply, 20.0); }
bool IDrone::land() { return command("land", &lastReply, 20.0); }
bool IDrone::emergency() { return command("emergency", &lastReply, 3.0); }

bool IDrone::go(int x, int y, int z, int speed) {
    char buf[64];
    snprintf(buf, sizeof(buf), "go %d %d %d %d", x, y, z, speed);
    double dist = std::sqrt((double)x * x + (double)y * y + (double)z * z);
    return command(buf, &lastReply, 5.0 + dist / std::max(10, speed) * 2.0);
}

bool IDrone::rotateCw(int deg) {
    char buf[64];
    if (deg == 0) return true;
    snprintf(buf, sizeof(buf), "%s %d", deg > 0 ? "cw" : "ccw", std::abs(deg));
    return command(buf, &lastReply, 8.0 + std::abs(deg) / 30.0);
}

bool IDrone::move(const std::string& dir, int cm) {
    char buf[64];
    snprintf(buf, sizeof(buf), "%s %d", dir.c_str(), cm);
    return command(buf, &lastReply, 5.0 + cm / 20.0);
}

int IDrone::queryInt(const std::string& q) {
    std::string reply;
    if (!command(q, &reply, 3.0)) return -1;
    char* end = nullptr;
    long v = strtol(reply.c_str(), &end, 10);
    return (end && end != reply.c_str()) ? (int)v : -1;
}

bool NullDrone::command(const std::string& cmd, std::string* reply, double) {
    LOG_INFO("[null-drone] command: %s", cmd.c_str());
    if (cmd == "takeoff") flying_ = true;
    if (cmd == "land" || cmd == "emergency") flying_ = false;
    if (reply) *reply = "ok";
    return true;
}
