#pragma once
#include <string>
#include <vector>

#include <opencv2/videoio.hpp>

#include "tello/IDrone.h"
#include "util/Clock.h"
#include "util/Config.h"
#include "vision/FrameSource.h"
#include "vision/Perception.h"

// A recorded flight (the .avi and the .csv of the ground station's RECORD) played back as the drone, the camera and
// the clock at once, the way the Simulator stands in for all three: perception and the navigator run unmodified on
// the frames and the telemetry of the real flight, each at its own time. It is open-loop: the video shows what the
// real drone did, whatever is commanded here. Once the navigator decides differently from the flight, what follows
// applies only loosely; where it first does is the finding.
//
// What it cannot give back: the frames went through the recorder's MJPG before they reached the file, so a pixel that
// sat on a colour limit in flight may fall on the other side here (replay.jpeg_quality = 85..95 plays them through
// one more generation: a decision that changes with it hung on such pixels). And a station that had flown before
// the recording began went into it with a camera angle measured in those flights, not the one given here: where the
// tags are seen against where the flight saw them shows that, and is reported at the end.
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
    // What perception made of the frame just played: its tag is compared with where the flight saw it.
    void observed(const Observation& obs);

private:
    struct Row {
        double t = 0;               // time of the frame on the station clock (s), rebuilt from seq and the t column
        uint64_t seq = 0;
        bool ground = false;        // recorded phase: not flying (IDLE, LANDED, ABORTED)
        int tagId = -1;             // the tag the flight saw in this frame (-1: none) and where, on the floor (m)
        double tagX = 0, tagY = 0;
        TelloState st;
        int rc[4] = {0, 0, 0, 0};   // the sticks the real flight sent on this frame
    };
    bool loadCsv(std::string* err);
    void compareSticks();
    void finish(const char* why);

    std::string videoPath_, csvPath_;
    bool realtime_ = false;
    int tagEveryN_ = 1;
    int jpegQuality_ = 0;                         // replay.jpeg_quality: > 0 plays every frame through one more JPEG generation
    double givenPitch_ = 0, givenRoll_ = 0;       // the camera angle the replay was started with
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
    // the tags here against the flight's: frames with both, the sums of the difference (m) and of the height, the rest
    int tagN_ = 0, tagOnlyFlight_ = 0, tagOnlyHere_ = 0;
    double tagSum_[2] = {0, 0}, tagSq_ = 0, tagHeight_ = 0;
    double tagFirst_[3] = {0, 0, 0};              // the first of them: the difference and the height
    double lastWall_ = 0, lastRowT_ = 0;
};
