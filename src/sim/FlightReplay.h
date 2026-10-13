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
// Three things the recording does not hold, and what is done about each:
//  - The camera angle. A station that had flown before went into the recording with an angle measured in those
//    flights, and measures it again on the way from sightings a replay never gets. But the csv says where the flight
//    saw its tags and the video has the same pixels, so the angle the flight had in force is read off every sighting
//    and the replay is given it, step by step (replay.follow_camera). Off, the angle is the configured one and is
//    fitted again here from the same sightings, as on a station just started: that is the default for a flight that
//    began with ground.mount_verified = 0, which measured its mount itself with the manoeuvre around the first tag.
//  - The pixels. The frames went through the recorder's MJPG before they reached the file, and that leaves its
//    block grid in them: in the three recordings of 2026-10-12 the two colour channels jump from one 16 px block to
//    the next by 6 to 30 times the step between neighbours inside a block (the flight's frames had no such grid).
//    Where a colour sits on a limit of the tape mask (the dark cells of a tag, a tape end) the mask follows the
//    blocks. replay.jpeg_shift = 1..15 plays every frame through one more JPEG generation whose blocks lie that
//    many px further right and down: the recorder's grid goes (0.3 to 0.9 times), a far weaker one comes,
//    elsewhere. None of these shows the flight's own pixels; a decision most of them make alike does not hang on
//    the blocks (tools/replay_check.py runs several and counts).
//  - When a telemetry packet arrived (recordings without the st_t column), and when in each loop pass the navigator
//    looked at the clock. The csv holds the telemetry that was current at every frame, so a packet is placed half a
//    frame before the row that first shows it, and the navigator's clock is the frame's own time. The lean taken
//    out of the floor model is interpolated between packets: while it changes it comes out up to a degree off the
//    flight's. In the frames with a tag that is put right together with the camera angle; in the others it stays
//    (a centimetre or two on the floor while the drone tips, nothing while it is level).
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
    // The camera angle the flight took off with, written into the configuration the replay is built from (and the
    // fit that would move it again switched off: aim() does that from here on). Nothing when it is not followed.
    void applyTo(Config& cfg) const;
    // Before the frame just delivered is looked at: the camera angle the flight had in force on it, and what its
    // lean differed by from the one worked out here.
    void aim(Perception& perception);
    // What perception made of the frame just played: its tag is compared with where the flight saw it.
    void observed(const Observation& obs);
    // False when the video or the csv stopped short of the recorded flight: what was replayed is not all of it.
    bool complete() const { return complete_; }

private:
    struct Row {
        double t = 0;               // time of the frame on the station clock (s), rebuilt from seq and the t column
        uint64_t seq = 0;
        bool ground = false;        // recorded phase: not flying (IDLE, LANDED, ABORTED)
        int tagId = -1;             // the tag the flight saw in this frame (-1: none) and where, on the floor (m)
        double tagX = 0, tagY = 0;
        TelloState st;
        int rc[4] = {0, 0, 0, 0};   // the sticks the real flight sent on this frame
        int paths = -1;             // pieces of tape the flight traced in this frame (-1: the recording does not say)
    };
    struct Mount { size_t row; double pitch, roll; };   // from this row on the flight had this camera angle in force
    bool loadCsv(std::string* err);
    bool readFrame(size_t row, cv::Mat& img);
    bool followCamera(std::string* err);
    void compareSticks();
    void finish(const char* why);

    Config cfg_;
    std::string videoPath_, csvPath_;
    bool realtime_ = false;
    int tagEveryN_ = 1;
    int jpegShift_ = 0, jpegQuality_ = 0;         // replay.jpeg_shift > 0: one more JPEG generation on a grid moved by that many px (quality: 90)
    bool follow_ = true;                          // replay.follow_camera (default: ground.mount_verified)
    double givenPitch_ = 0, givenRoll_ = 0;       // the camera angle the replay was started with
    std::vector<Row> rows_;
    std::vector<Mount> mount_;                    // the camera angle of the flight over time; [0] is the one it took off with
    std::vector<cv::Point2d> leanFix_;            // per row: what the flight's lean differed by from the one known here (deg: pitch, roll)
    size_t mountNext_ = 1;
    cv::VideoCapture cap_;
    size_t first_ = 0, takeoff_ = 0, last_ = 0;   // rows: first one played, the take-off, the last one of the flight
    size_t next_ = 0, cur_ = 0, framesRead_ = 0;  // rows: the next one to play and the one being played; frames taken from the video
    size_t telem_ = 0;                            // the row state() answers with (the one being played, except right after the take-off)
    double fps_ = 24, now_ = 0, airborneT_ = 0, landS_ = 0;
    bool flying_ = false, started_ = false, ended_ = false, delivered_ = false, complete_ = true;
    int sent_[4] = {0, 0, 0, 0};
    double diffSq_[4] = {0, 0, 0, 0};
    int diffN_ = 0, apartRun_ = 0;
    double apartT_ = -1;            // mission time from which the sticks stayed apart from the recorded ones (-1: never)
    // the tags here against the flight's: frames with both, the sums of the difference (m) and of the height, the rest
    int tagN_ = 0, tagOnlyFlight_ = 0, tagOnlyHere_ = 0;
    double tagSum_[2] = {0, 0}, tagSq_ = 0, tagHeight_ = 0;
    double tagFirst_[3] = {0, 0, 0};              // the first of them: the difference and the height
    int pathsN_ = 0, pathsSame_ = 0;              // frames whose number of traced tape pieces is known from the flight; those with the same here
    double pathsFirstT_ = -1;                     // mission time of the first frame with another number (-1: none)
    double paceWall_ = 0, paceRowT_ = 0;          // real-time pacing: when the last frame was due, and its time in the recording
};
