#pragma once

#include <thread>
#include <gst/gstpad.h>
#include <image_transport/camera_subscriber.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core/mat.hpp>
#include <peel/Gst/Bin.h>
#include <peel/Gst/Pad.h>
#include <peel/Gst/PadProbeReturn.h>
#include <peel/GstApp/AppSink.h>
#include <SDL3/SDL_opengl.h>

#include "foc2-gui/overlayable.hpp"
#include "foc2-gui/widget.hpp"
#include "foc2-gui/overlays/video_stats_overlay.hpp"

class ArucoVideoOverlay;
class NavPathVideoOverlay;

struct _GstAppSink;
typedef _GstAppSink GstAppSink;

enum class VideoFlipMethod {
    NONE = 0,
    CLOCKWISE = 1,
    ROTATE_180 = 2,
    COUNTERCLOCKWISE = 3,
    HORIZONTAL_FLIP = 4,
    VERTICAL_FLIP = 5,
    UPPER_LEFT_DIAGONAL = 6,
    UPPER_RIGHT_DIAGONAL = 7,
    AUTOMATIC = 8,
};

// TODO 2026-05-24 (Will Free): consider converting to the pimpl pattern to improve compile times?
class VideoWidget : public UiWidget, public UiOverlayable {
    using CameraInfo = sensor_msgs::msg::CameraInfo;
    template <typename T>
    using FloatPtr = peel::FloatPtr<T>;
    template <typename T>
    using RefPtr = peel::RefPtr<T>;

public:
    RCLCPP_SMART_PTR_ALIASES_ONLY(VideoWidget);

    explicit VideoWidget(
        ImApplication& application,
        const std::string& source_url,
        const std::string& camera_topic,
        bool minimap
    );

    void onInit() override;
    void onShutdown() override;

    [[nodiscard]] ImVec2 expectedSize(ImVec2 available) const;

protected:
    void draw() override;

private:
    void drawContextMenu();

    void drawFiltersMenu();

    void drawConfigWindow();

public:
    struct StreamConfig {
        std::string source_url;
        std::string camera_topic;
        bool minimap = false;
        int rtspsrc_latency = 100;
        int jitterbuffer_latency = 50;
    };

    struct FilterState {
        bool enabled = true;
        double gamma = 1.0;
        double brightness = 0.0;
        double contrast = 1.0;
        double saturation = 1.0;
        VideoFlipMethod rotation = VideoFlipMethod::NONE;
        double sharpness = 0.0;
    };

    enum class ConnectionState {
        STOPPED,
        RUNNING,
        DISCONNECTED,
    };

private:
    bool config_window_open = false;
    StreamConfig stream_config;
    StreamConfig stream_config_next;

    rclcpp::TimerBase::SharedPtr stats_timer;
    SubscriptionGroup<CameraInfo>::SharedPtr camera_info_subscription;

    std::thread gst_thread;
    std::atomic<bool> running = true;
    std::atomic<bool> reconnect = false;
    cv::Mat next_frame;
    cv::Mat current_frame;
    std::mutex frame_mutex;

    FloatPtr<peel::Gst::Bin> pipeline = nullptr;
    RefPtr<peel::Gst::Element> rtspsrc = nullptr;
    RefPtr<peel::Gst::Element> jitterbuffer = nullptr;
    RefPtr<peel::GstApp::AppSink> appsink = nullptr;

    RefPtr<peel::Gst::Element> balance = nullptr;
    RefPtr<peel::Gst::Element> gamma = nullptr;
    RefPtr<peel::Gst::Element> sharpness = nullptr;
    RefPtr<peel::Gst::Element> videoflip = nullptr;

    std::mutex stats_mutex;
    std::chrono::time_point<std::chrono::steady_clock> last_probe;
    std::chrono::time_point<std::chrono::steady_clock> last_frame;

    VideoStatsOverlay::VideoStats video_stats;
    std::shared_ptr<VideoStatsOverlay> stats_overlay;

    bool new_frame_available = false;

    GLuint texture_id = 0;
    int texture_width = 0;
    int texture_height = 0;

    FilterState filters = {};

    void videoThread();

    FloatPtr<peel::Gst::Bin> createPipeline() const;

    void configureAppsink();

    void configureJitterbuffer();

    peel::Gst::Pad::ProbeReturn onJitterbufferProbe(peel::Gst::Pad*, const peel::Gst::Pad::ProbeInfo* info);

    void runPipelineLoop();

    GstFlowReturn onNewSample(GstAppSink* sink);

    void updateTexture();

    void applyRosCameraTopic();

    void applyStreamConfig();

    void applyFilters(const FilterState& filters, bool update_flip = true) const;

    void applyFlip(VideoFlipMethod rotation) const;

    std::string getUniqueId(const char* prefix) {
        // we're using the memory address of the current object to ensure every object has a unique id
        return fmt::format("{}_{}", prefix, reinterpret_cast<uintptr_t>(this));
    }
};
