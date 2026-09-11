#pragma once
#include <string>

#include "tello/TelloState.h"

// Abstract drone: implemented by TelloDrone (real UDP SDK), Simulator, and NullDrone (replay/dry-run).
class IDrone {
public:
    virtual ~IDrone() = default;
    virtual std::string name() const = 0;
    virtual bool connect(std::string* err) = 0;
    virtual void disconnect() = 0;
    // Blocking SDK text command ("takeoff", "go 50 0 0 60", ...). Returns true when the drone answered "ok".
    virtual bool command(const std::string& cmd, std::string* reply, double timeoutS) = 0;
    // Non-blocking stick command, all channels -100..100: lr(+right) fb(+forward) ud(+up) yaw(+clockwise)
    virtual void rc(int lr, int fb, int ud, int yaw) = 0;
    virtual TelloState state() const = 0;
    virtual bool streamOn() = 0;
    virtual bool isFlying() const = 0;

    // Convenience wrappers over command()
    bool takeoff();
    bool land();
    bool emergency();
    bool go(int xCm, int yCm, int zCm, int speedCmS);   // body frame: x forward, y left (SDK convention), z up
    bool rotateCw(int deg);                             // negative = counter-clockwise
    bool move(const std::string& dir, int cm);          // "forward" "back" "left" "right" "up" "down"
    int queryInt(const std::string& q);                 // "battery?" -> 83, -1 on failure

    // last blocking-command reply text for diagnostics
    std::string lastReply;
};

// Drone that accepts everything and does nothing (video replay / dry-run).
class NullDrone : public IDrone {
public:
    std::string name() const override { return "null"; }
    bool connect(std::string*) override { return true; }
    void disconnect() override {}
    bool command(const std::string& cmd, std::string* reply, double) override;
    void rc(int, int, int, int) override {}
    TelloState state() const override { return state_; }
    bool streamOn() override { return true; }
    bool isFlying() const override { return flying_; }
    void setState(const TelloState& s) { state_ = s; }
private:
    TelloState state_;
    bool flying_ = false;
};
