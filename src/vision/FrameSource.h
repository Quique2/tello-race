#pragma once
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

struct Frame {
    cv::Mat image;        // BGR
    uint64_t seq = 0;
    double stamp = 0;     // wall/sim seconds when grabbed
};

class IFrameSource {
public:
    virtual ~IFrameSource() = default;
    virtual bool start(std::string* err) = 0;
    virtual void stop() = 0;
    // Waits up to timeoutS for a frame newer than afterSeq. Returns false on timeout / end of stream.
    virtual bool latest(Frame& out, uint64_t afterSeq, double timeoutS) = 0;
    virtual bool isLive() const = 0;
    virtual bool ended() const = 0;
    virtual double fps() const = 0;
};

// cv::VideoCapture based source: "udp://0.0.0.0:11111" (Tello), "file:path.avi", "camera:0".
// Live sources are read on a dedicated thread that always keeps only the newest frame (no backlog latency).
class VideoCaptureSource : public IFrameSource {
public:
    explicit VideoCaptureSource(const std::string& spec, bool realtimeReplay = true);
    ~VideoCaptureSource() override;
    bool start(std::string* err) override;
    void stop() override;
    bool latest(Frame& out, uint64_t afterSeq, double timeoutS) override;
    bool isLive() const override { return live_; }
    bool ended() const override { return ended_; }
    double fps() const override { return fps_; }

private:
    void grabLoop();
    std::string spec_;
    bool live_ = true;
    bool realtime_ = true;
    cv::VideoCapture cap_;
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> ended_{false};
    std::mutex mtx_;
    std::condition_variable cv_;
    Frame latest_;
    double fps_ = 0;
    double fileFps_ = 30;
    double lastFileFrameTime_ = 0;
};
