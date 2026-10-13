#pragma once
#include <string>
#include <vector>

#include <opencv2/videoio.hpp>

#include "tello/IDrone.h"
#include "util/Clock.h"
#include "util/Config.h"
#include "vision/FrameSource.h"

// A recorded flight (the .avi and the .csv of the ground station's RECORD) played back as the drone, the camera and
// the clock at once, the way the Simulator stands in for all three: perception and the navigator run unmodified on
// the frames and the telemetry of the real flight, each at its own time. It is open-loop: the video shows what the
// real drone did, whatever is commanded here. Once the navigator decides differently from the flight, what follows
// applies only loosely; where it first does is the finding.
class FlightReplay : public IDrone, public IFrameSource, public IClock {
public:
    FlightReplay(const Config& cfg, const std::string& videoPath, const std::string& csvPath, bool realtime);

    // IClock: the station clock of the recording (the t column), set by the frame being delivered
    double now() const override { return now_; }
    void sleep(double s) override { if (s > 0) now_ += s; }   // the frames of that time are passed over, as in flight
    // IDrone: take-off, landing and every other command answer ok at once
    std::string name() const override { return "replay"; }
    bool connect(std::string*) override { return true; }
    void disconnect() override {}
    bool command(const std::string& cmd, std::string* reply, double timeoutS) override;
    void rc(int lr, int fb, int ud, int yaw) override { sent_[0] = lr; sent_[1] = fb; sent_[2] = ud; sent_[3] = yaw; }
    TelloState state() const override { return rows_.empty() ? TelloState() : rows_[telem_].st; }
    bool streamOn() override { return true; }
    bool isFlying() const override { return flying_; }
    // IFrameSource: one recorded frame per call, with the clock and the telemetry of its csv row
    bool start(std::string* err) override;
    void stop() override;
    bool latest(Frame& out, uint64_t afterSeq, double timeoutS) override;
    bool isLive() const override { return false; }
    bool ended() const override { return ended_; }
    double fps() const override { return fps_; }

    // True once, on the frame the real loop held when START was pressed: the mission is started on the same one.
    bool startMissionNow();

private:
    struct Row {
        double t = 0;               // time of the frame on the station clock (s), rebuilt from seq and the t column
        uint64_t seq = 0;
        bool ground = false;        // recorded phase: not flying (IDLE, LANDED, ABORTED)
        bool tag = false;           // a tag was seen in this frame
        TelloState st;
        int rc[4] = {0, 0, 0, 0};   // the sticks the real flight sent on this frame
    };
    bool loadCsv(std::string* err);
    void compareSticks();
    void finish(const char* why);

    std::string videoPath_, csvPath_;
    bool realtime_ = false;
    int tagEveryN_ = 1;
    std::vector<Row> rows_;
    cv::VideoCapture cap_;
    size_t first_ = 0, takeoff_ = 0, last_ = 0;   // rows: first one played, the take-off, the last one of the flight
    size_t next_ = 0, cur_ = 0, framesRead_ = 0;  // rows: the next one to play and the one being played; frames taken from the video
    size_t telem_ = 0;                            // the row state() answers with (the one being played, except right after the take-off)
    double fps_ = 24, now_ = 0, airborneT_ = 0, landS_ = 0;
    bool flying_ = false, started_ = false, ended_ = false, delivered_ = false;
    int sent_[4] = {0, 0, 0, 0};
    double diffSq_[4] = {0, 0, 0, 0};
    int diffN_ = 0, apartRun_ = 0;
    double apartT_ = -1;            // mission time from which the sticks stayed apart from the recorded ones (-1: never)
    double lastWall_ = 0, lastRowT_ = 0;
};
