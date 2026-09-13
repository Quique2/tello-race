#include "vision/FrameSource.h"

#include <chrono>
#include <cstdlib>

#include "util/Log.h"

static double wallNow() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

static void setEnv(const char* k, const char* v) {
#ifdef _WIN32
    _putenv_s(k, v);
#else
    setenv(k, v, 1);
#endif
}

VideoCaptureSource::VideoCaptureSource(const std::string& spec, bool realtimeReplay)
    : spec_(spec), realtime_(realtimeReplay) {}

VideoCaptureSource::~VideoCaptureSource() { stop(); }

bool VideoCaptureSource::start(std::string* err) {
    ended_ = false;   // start() is also used to re-open a live stream (camera switch, recovery)
    fps_ = 0;
    { std::lock_guard<std::mutex> lk(mtx_); latest_ = Frame(); }   // never serve the frame from before the restart
    if (spec_.rfind("file:", 0) == 0) {
        live_ = false;
        std::string path = spec_.substr(5);
        if (!cap_.open(path)) { if (err) *err = "cannot open video file " + path; return false; }
        fileFps_ = cap_.get(cv::CAP_PROP_FPS);
        if (fileFps_ <= 1 || fileFps_ > 240) fileFps_ = 30;
        fps_ = fileFps_;
        LOG_INFO("Video file %s opened (%.0fx%.0f @ %.1f fps)", path.c_str(), cap_.get(cv::CAP_PROP_FRAME_WIDTH),
                 cap_.get(cv::CAP_PROP_FRAME_HEIGHT), fileFps_);
        return true;
    }
    if (spec_.rfind("camera:", 0) == 0) {
        int idx = atoi(spec_.c_str() + 7);
        if (!cap_.open(idx)) { if (err) *err = "cannot open camera " + std::to_string(idx); return false; }
        cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
    } else {
        // Tello H.264 UDP stream through FFmpeg: ask for the lowest-latency demuxer/decoder settings.
        setEnv("OPENCV_FFMPEG_CAPTURE_OPTIONS", "fflags;nobuffer|flags;low_delay|probesize;32|analyzeduration;0|max_delay;0");
        setEnv("OPENCV_FFMPEG_LOGLEVEL", "-8");
        std::string url = spec_ == "udp" ? "udp://0.0.0.0:11111" : spec_;
        LOG_INFO("Opening video stream %s ...", url.c_str());
        if (!cap_.open(url, cv::CAP_FFMPEG)) {
            LOG_WARN("Low-latency open failed, retrying with FFmpeg defaults");
            setEnv("OPENCV_FFMPEG_CAPTURE_OPTIONS", "");
            if (!cap_.open(url, cv::CAP_FFMPEG)) { if (err) *err = "cannot open stream " + url; return false; }
        }
        cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
    }
    live_ = true;
    running_ = true;
    thread_ = std::thread(&VideoCaptureSource::grabLoop, this);
    return true;
}

void VideoCaptureSource::stop() {
    running_ = false;
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
    if (cap_.isOpened()) cap_.release();
}

void VideoCaptureSource::grabLoop() {
    uint64_t seq = 0;
    int fails = 0;
    double fpsT0 = wallNow();
    int fpsN = 0;
    cv::Mat img;
    while (running_) {
        if (!cap_.read(img) || img.empty()) {
            if (++fails > 300) { LOG_ERROR("video: no frames for a long time, stream ended?"); ended_ = true; break; }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        fails = 0;
        ++fpsN;
        double t = wallNow();
        if (t - fpsT0 >= 1.0) { fps_ = fpsN / (t - fpsT0); fpsN = 0; fpsT0 = t; }
        {
            std::lock_guard<std::mutex> lk(mtx_);
            latest_.image = img.clone();
            latest_.seq = ++seq;
            latest_.stamp = t;
        }
        cv_.notify_all();
    }
}

bool VideoCaptureSource::latest(Frame& out, uint64_t afterSeq, double timeoutS) {
    if (!live_) {
        // file replay: sequential read, optionally paced at the file's frame rate
        if (realtime_) {
            double target = lastFileFrameTime_ + 1.0 / fileFps_;
            double now = wallNow();
            if (now < target) std::this_thread::sleep_for(std::chrono::duration<double>(target - now));
        }
        cv::Mat img;
        if (!cap_.read(img) || img.empty()) { ended_ = true; return false; }
        out.image = img;
        out.seq = afterSeq + 1;
        out.stamp = lastFileFrameTime_ = wallNow();
        return true;
    }
    std::unique_lock<std::mutex> lk(mtx_);
    bool ok = cv_.wait_for(lk, std::chrono::duration<double>(timeoutS), [&] { return latest_.seq > afterSeq || !running_; });
    if (!ok || latest_.seq <= afterSeq) return false;
    out = latest_;
    return true;
}
