#pragma once
#include <chrono>
#include <thread>

// Time source abstraction so the same navigator code runs against wall-clock time (real drone)
// or simulated time (simulator, possibly faster than real time).
class IClock {
public:
    virtual ~IClock() = default;
    virtual double now() const = 0;          // seconds, monotonic
    virtual void sleep(double seconds) = 0;
};

class SteadyClock : public IClock {
public:
    double now() const override {
        return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    void sleep(double s) override {
        if (s > 0) std::this_thread::sleep_for(std::chrono::duration<double>(s));
    }
};
