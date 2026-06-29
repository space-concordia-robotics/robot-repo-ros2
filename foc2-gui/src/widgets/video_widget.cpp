#include "foc2-gui/widgets/video_widget.hpp"

#include <IconsFontAwesome7.h>
#include <imgui.h>
#include <cstddef>
#include <image_transport/camera_common.hpp>
#include <magic_enum/magic_enum.hpp>
#include <misc/cpp/imgui_stdlib.h>
#include <peel/Gst/Bin.h>
#include <peel/Gst/MapFlags.h>
#include <peel/Gst/MapInfo.h>
#include <peel/Gst/MessageType.h>
#include <peel/Gst/PadProbeInfo.h>
#include <peel/Gst/PadProbeReturn.h>
#include <peel/Gst/PadProbeType.h>
#include <peel/Gst/State.h>
#include <peel/Gst/StateChangeReturn.h>
#include <peel/Gst/functions.h>
#include <peel/GstApp/AppSink.h>
#include <sensor_msgs/msg/camera_info.hpp>

#include "foc2-gui/overlays/aruco_video_overlay.hpp"
#include "foc2-gui/overlays/crosshair_overlay.hpp"
#include "foc2-gui/overlays/minimap.hpp"
#include "foc2-gui/overlays/nav_path_video_overlay.hpp"
#include "foc2-gui/overlays/video_stats_overlay.hpp"


using namespace std::chrono;
using namespace std::chrono_literals;

using namespace peel;

template <>
struct fmt::formatter<String> : formatter<const char*> {
    template <typename FormatContext>
    auto format(const String& string, FormatContext& ctx) const {
        return formatter<const char*>::format(string.c_str(), ctx);
    }
};

namespace magic_enum::customize {
    template <>
    constexpr customize_t enum_name(const VideoFlipMethod method) noexcept {
        switch (method) {
        case VideoFlipMethod::NONE:
            return "None";
            break;
        case VideoFlipMethod::CLOCKWISE:
            return "Clockwise";
            break;
        case VideoFlipMethod::ROTATE_180:
            return "Rotate 180";
            break;
        case VideoFlipMethod::COUNTERCLOCKWISE:
            return "Counterclockwise";
            break;
        case VideoFlipMethod::HORIZONTAL_FLIP:
            return "Horizontal Flip";
            break;
        case VideoFlipMethod::VERTICAL_FLIP:
            return "Vertical Flip";
            break;
        case VideoFlipMethod::UPPER_LEFT_DIAGONAL:
            return "Upper Left Diagonal";
            break;
        case VideoFlipMethod::UPPER_RIGHT_DIAGONAL:
            return "Upper Right Diagonal";
            break;
        case VideoFlipMethod::AUTOMATIC:
            return "Automatic";
            break;
        }
        return invalid_tag;
    }
}

VideoWidget::VideoWidget(
    ImApplication& application,
    std::string source_url,
    std::string camera_topic,
    const bool minimap
)
    : UiWidget(application) {
    stream_config.source_url = std::move(source_url);
    stream_config.camera_topic = std::move(camera_topic);
    stream_config.minimap = minimap;

    stats_overlay = std::make_shared<VideoStatsOverlay>(application);
    const auto aruco_overlay = std::make_shared<ArucoVideoOverlay>(application);
    const auto global_nav_overlay = std::make_shared<NavPathVideoOverlay>(application, "/plan", ImVec4(0.1, 1, 0, 1));
    const auto local_nav_overlay = std::make_shared<NavPathVideoOverlay>(application, "/local_plan", ImVec4(1, 0.3, 1, 1));

    addOverlay(stats_overlay);
    addOverlay(aruco_overlay);
    addOverlay(local_nav_overlay);
    addOverlay(global_nav_overlay);

    camera_info_subscription = SubscriptionGroup<CameraInfo>::make_shared(application);

    camera_info_subscription->addCallback(
        aruco_overlay,
        [overlay = aruco_overlay](const CameraInfo::SharedPtr& camera_info) {
            overlay->onCameraInfo(camera_info);
        }
    );

    camera_info_subscription->addCallback(
        local_nav_overlay,
        [overlay = local_nav_overlay](const CameraInfo::SharedPtr& camera_info) mutable {
            overlay->onCameraInfo(camera_info);
        }
    );

    camera_info_subscription->addCallback(
        global_nav_overlay,
        [overlay = global_nav_overlay](const CameraInfo::SharedPtr& camera_info) mutable {
            overlay->onCameraInfo(camera_info);
        }
    );

    addOverlay(std::make_shared<CrosshairOverlay>(application));

    applyMinimap();
}

void VideoWidget::onInit() {
    UiWidget::onInit();
    UiOverlayable::onInit();

    gst_thread = std::thread([this] {
        videoThread();
    });

    applyRosCameraTopic();

    // if this 50ms value is changed, also update the value in the video stats overlay
    stats_timer = this->application.create_timer(50ms, [this] {
        if (this->stats_overlay) {
            std::scoped_lock lock(this->stats_mutex);

            this->stats_overlay->updateStats(this->video_stats);
        }
    });
}

void VideoWidget::onShutdown() {
    UiWidget::onShutdown();
    UiOverlayable::onShutdown();

    running = false;
    gst_thread.join();

    if (texture_id != 0u) {
        glDeleteTextures(1, &texture_id);
        texture_id = 0;
        texture_width = 0;
        texture_height = 0;
    }
}

ImVec2 VideoWidget::expectedSize(const ImVec2 available) const {
    const float video_aspect = static_cast<float>(current_frame.cols) / static_cast<float>(current_frame.rows);

    ImVec2 size = available;

    if (const float window_aspect = available.x / available.y; window_aspect > video_aspect) {
        size.x = available.y * video_aspect;
    } else {
        size.y = available.x / video_aspect;
    }

    return size;
}

void VideoWidget::draw() {
    const auto now = steady_clock::now();

    if (last_frame != time_point<steady_clock>()) {
        if (const auto frame_delta = duration_cast<duration<double>>(now - last_frame).count(); frame_delta > 0.0) {
            video_stats.fps = 1.0 / frame_delta;
        }
    }

    updateTexture();

    ImGui::BeginChild("VideoRegion", ImVec2(0, 0), 0, ImGuiWindowFlags_NoScrollbar);

    // TODO 2026-05-07 (Will Free): tint entire frame red or gray or something when no frame has been received for like 1-2s

    const auto avail = ImGui::GetContentRegionAvail();

    if (texture_id != 0u) {
        const float video_aspect = static_cast<float>(current_frame.cols) / static_cast<float>(current_frame.rows);

        ImVec2 size = avail;

        if (const float window_aspect = avail.x / avail.y; window_aspect > video_aspect) {
            size.x = avail.y * video_aspect;
        } else {
            size.y = avail.x / video_aspect;
        }

        ImGui::SetCursorPosX((avail.x - size.x) * 0.5f);
        ImGui::SetCursorPosY((avail.y - size.y) * 0.5f);

        ImGui::Image(texture_id, size);

        drawContextMenu(); // TODO 2026-05-24 (Will Free): should this be put aftr the draw overlays? idk if draw overlays will mess with it at all...

        drawOverlays(ImGui::GetWindowDrawList(), ImRect(ImGui::GetItemRectMin(), ImGui::GetItemRectMax()));

        drawFiltersMenu();
    } else {
        // TODO 2026-05-07 (Will Free): show message if we got disconnected from the stream & are reconnecting

        static constexpr auto AWAITING_FRAME_TEXT = "Waiting for video frame...";

        const auto draw_list = ImGui::GetWindowDrawList();

        const auto top_left = ImGui::GetCursorScreenPos();

        const auto text_size = ImGui::CalcTextSize(AWAITING_FRAME_TEXT);

        draw_list->AddText(top_left + avail * 0.5 - text_size * 0.5, ImGui::ImColor(255, 255, 255, 255), AWAITING_FRAME_TEXT);

        ImGui::Dummy(avail);

        drawContextMenu();
    }

    ImGui::EndChild();

    //

    drawConfigWindow();
}

void VideoWidget::drawContextMenu() {
    if (ImGui::BeginPopupContextItem(getUniqueId("stream_menu").data())) {
        if (ImGui::MenuItem(ICON_FA_GEARS " Configure Video")) {
            stream_config_next = stream_config; // reset config_next to current config
            config_window_open = true;
        }

        if (ImGui::MenuItem(ICON_FA_ROTATE " Reload Stream")) {
            reconnect = true;
        }

        ImGui::EndPopup();
    }
}

void VideoWidget::drawFiltersMenu() {
    const auto item_max = ImGui::GetItemRectMax();
    constexpr auto cog_size = ImVec2(20.0f, 20.0f);
    const auto cog_pos = item_max - cog_size - ImVec2(6.0, 6.0);
    const auto icon_col = ImGui::GetColorU32(ImGuiCol_Text);

    const auto draw_list = ImGui::GetWindowDrawList();

    ImGui::PushFont(nullptr, 24.0);
    const ImVec2 text_size = ImGui::CalcTextSize(ICON_FA_GEAR);
    const ImVec2 text_pos = cog_pos + (cog_size - text_size) * 0.5f;
    draw_list->AddText(text_pos, icon_col, ICON_FA_GEAR);
    ImGui::PopFont();

    ImGui::SetCursorScreenPos(cog_pos);

    const auto popup_id = getUniqueId("filters_popup");

    if (ImGui::InvisibleButton("filters", cog_size)) {
        ImGui::OpenPopup(popup_id.data());
    }

    if (ImGui::BeginPopup(popup_id.data())) {
        auto changed = false;

        ImGui::TextUnformatted("Filters");
        ImGui::Separator();

        changed |= ImGui::Checkbox("Enable filters", &filters.enabled);

        ImGui::BeginDisabled(!filters.enabled);

        ImGui::Spacing();

        changed |= ImGui::SliderDoubleSnapping("Gamma", &filters.gamma, 0.1, 3.0, 0.1);
        changed |= ImGui::SliderDoubleSnapping("Brightness", &filters.brightness, -1.0, 1.0, 0.1);
        changed |= ImGui::SliderDoubleSnapping("Contrast", &filters.contrast, 0.0, 3.0, 0.1);
        changed |= ImGui::SliderDoubleSnapping("Saturation", &filters.saturation, 0.0, 3.0, 0.1);

        ImGui::Spacing();

        static constexpr auto VIDEO_FLIP_ENTRIES = magic_enum::enum_entries<VideoFlipMethod>();
        const auto current_index = magic_enum::enum_index(filters.rotation).value_or(0);
        const auto preview_value = magic_enum::enum_name(filters.rotation);

        // NOLINTNEXTLINE(*-suspicious-stringview-data-usage): preview_value is null terminated
        if (ImGui::BeginCombo("Rotation", preview_value.data(), ImGuiComboFlags_None)) {
            for (auto i = 0u; i < VIDEO_FLIP_ENTRIES.size(); ++i) {
                const auto [enum_val, name] = VIDEO_FLIP_ENTRIES.at(i);

                ImGui::PushID(static_cast<int>(i));

                const auto selected = i == current_index;

                // NOLINTNEXTLINE(*-suspicious-stringview-data-usage): name is null terminated
                if (ImGui::Selectable(name.data(), selected)) {
                    filters.rotation = enum_val;
                    changed = true;
                }

                if (selected)
                    ImGui::SetItemDefaultFocus();

                ImGui::PopID();
            }
            ImGui::EndCombo();
        }

        ImGui::Spacing();

        changed |= ImGui::SliderDoubleSnapping("Sharpness", &filters.sharpness, 0.0, 5.0, 0.1);

        ImGui::EndDisabled();

        ImGui::Separator();

        static constexpr auto DEFAULT_FILTER_STATE = FilterState();

        if (ImGui::Button("Reset")) {
            filters = DEFAULT_FILTER_STATE;
            changed = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Close")) {
            ImGui::CloseCurrentPopup();
        }

        if (filters.enabled) {
            if (changed)
                applyFilters(filters);
        } else {
            if (changed) {
                applyFilters(DEFAULT_FILTER_STATE, false);
                applyFlip(filters.rotation);
            }
        }

        ImGui::EndPopup();
    }
}

void VideoWidget::drawConfigWindow() {
    if (config_window_open) {
        if (ImGui::Begin("Video configuration", &config_window_open)) {
            ImGui::InputText("RTSP URI", &stream_config_next.source_url);
            ImGui::InputText("ROS Camera Topic", &stream_config_next.camera_topic);

            ImGui::Spacing();

            ImGui::SliderInt("RTSP latency (ms)", &stream_config_next.rtspsrc_latency, 0, 1000);
            ImGui::SliderInt("Jitterbuffer latency (ms)", &stream_config.jitterbuffer_latency, 0, 1000);

            ImGui::Spacing();

            ImGui::Checkbox("Minimap", &stream_config_next.minimap);

            ImGui::Spacing();

            if (ImGui::Button("Apply")) {
                stream_config = stream_config_next; // apply next config
                applyStreamConfig();
                applyMinimap();
                applyRosCameraTopic();
            }

            ImGui::SameLine();

            if (ImGui::Button("Close")) {
                // TODO 2026-05-26 (Will Free): show message for unsaved changes.
                stream_config_next = stream_config; // reset config_next to current config
                config_window_open = false;
            }

            if (ImGui::Button("Reset")) {
                stream_config_next = stream_config; // reset config_next to current config
            }
        }

        ImGui::End();
    }
}

void VideoWidget::videoThread() {
    while (running) {
        pipeline = createPipeline();

        if (pipeline == nullptr) {
            std::this_thread::sleep_for(1s);
            continue;
        }

        const auto pipeline_bin = pipeline->cast<Gst::Bin>();

        rtspsrc = pipeline_bin->get_by_name("src");
        jitterbuffer = pipeline_bin->get_by_name("jitter");

        appsink = std::move(pipeline_bin->get_by_name("appsink")).cast<GstApp::AppSink>();

        balance = pipeline_bin->get_by_name("balance");
        gamma = pipeline_bin->get_by_name("gamma");
        sharpness = pipeline_bin->get_by_name("sharpness");
        // TODO 2026-05-20 (Will Free): determine initial videoflip from camera frame
        videoflip = pipeline_bin->get_by_name("videoflip");

        configureAppsink();
        applyStreamConfig();
        applyFilters(filters);

        // TODO 2026-05-07 (Will Free): we are currently not recording the latency of the RTSP stream.
        //  this can be done by listening to RTCP packets from the RTSP stream,
        //  and then getting the difference between the current time and the time in the packet.

        configureJitterbuffer();

        runPipelineLoop();

        if (running) {
            // TODO 2026-05-23 (Will Free): config value for this or something?
            std::this_thread::sleep_for(250ms);
        }
    }
}

FloatPtr<Gst::Bin> VideoWidget::createPipeline() const {
    // TODO 2026-05-07 (Will Free): replace this pipeline string with creating each of the elements individually
    //  via gst_element_factory_make(), then linking them together.

    // TODO 2026-05-22 (Will Free): when switching to creating the elements individually,
    //  maybe consider something that checks if rtspsrc exists, and if yes using it.

    // TODO 2026-05-23 (Will Free): maybe consider replacing rtspsrc with playbin and using an rtsp:// URI? idk if playbin lets you set the latency

    static constexpr auto GSTREAMER_PIPELINE = "rtspsrc name=src ! "
        "rtpjitterbuffer name=jitter ! " // TODO 2026-05-23 (Will Free): I think this additional jitterbuffer is just adding unecessary latency?
        "rtph264depay ! "
        "decodebin ! "
        "videoconvert ! "
        "videobalance name=balance ! "
        "gamma name=gamma ! "
        "frei0r-filter-sharpness name=sharpness ! "
        "videoflip name=videoflip ! "
        "video/x-raw,format=RGBA ! "
        "appsink name=appsink sync=true";

    peel::UniquePtr<GLib::Error> error = nullptr;

    auto pipeline = Gst::parse_launch(GSTREAMER_PIPELINE, &error);

    if (pipeline == nullptr || error != nullptr) {
        logger.error("Failed to create GStreamer pipeline: {}", error ? error->message : "unknown");
        return nullptr;
    }

    auto pipeline_bin = std::move(pipeline).cast<Gst::Bin>();

    const auto rtspsrc = pipeline_bin->get_by_name("src");
    rtspsrc->set_property<String>("location", stream_config.source_url.data());

    return pipeline_bin;
}

void VideoWidget::configureAppsink() {
    appsink->set_emit_signals(true);
    appsink->set_drop(true);
    appsink->set_max_buffers(1);
    // appsink->set_sync(true);  // TODO 2026-05-06 (Will Free): is this needed?

    auto appsink_callbacks = GstAppSinkCallbacks{};
    appsink_callbacks.new_sample = [](GstAppSink* appsink, void* widget) -> GstFlowReturn {
        return static_cast<VideoWidget*>(widget)->onNewSample(reinterpret_cast<GstApp::AppSink*>(appsink));
    };

    appsink->set_callbacks(reinterpret_cast<GstApp::AppSinkCallbacks*>(&appsink_callbacks), this, nullptr);
}

void VideoWidget::configureJitterbuffer() {
    if (const auto jitterbuffer_sink_pad = jitterbuffer->get_static_pad("sink")) {
        jitterbuffer_sink_pad->add_probe(
            Gst::Pad::ProbeType::BUFFER,
            [this](Gst::Pad* pad, const Gst::Pad::ProbeInfo* info) -> Gst::Pad::ProbeReturn {
                return onJitterbufferProbe(pad, info);
            }
        );
    }
}

Gst::Pad::ProbeReturn VideoWidget::onJitterbufferProbe(Gst::Pad* /*pad*/, const Gst::Pad::ProbeInfo* info) {
    if (info == nullptr)
        return Gst::Pad::ProbeReturn::HANDLED;

    const auto buf = GST_PAD_PROBE_INFO_BUFFER(info);

    if (buf == nullptr)
        return Gst::Pad::ProbeReturn::OK;

    auto avg_jitter = 0uL;
    auto num_pushed = 0uL;
    auto num_lost = 0uL;
    auto rtx_rtt = 0uL;
    auto loss = 0.0;

    if (jitterbuffer) {
        // TODO 2026-05-21 (Will Free): figure out how Gst::Structure works in peel
        GstStructure* stats = nullptr;
        g_object_get(jitterbuffer, "stats", &stats, nullptr);

        gst_structure_get_uint64(stats, "avg-jitter", &avg_jitter);
        gst_structure_get_uint64(stats, "num-pushed", &num_pushed);
        gst_structure_get_uint64(stats, "num-lost", &num_lost);
        gst_structure_get_uint64(stats, "rtx-rtt", &rtx_rtt);

        // TODO 2026-05-06 (Will Free): num-lost seems to periodically reset here, whereas num-pushed doesn't. deal with that.

        gst_structure_free(stats);

        if (num_pushed + num_lost > 0)
            loss = static_cast<double>(num_lost) / static_cast<double>(num_pushed + num_lost);
    }

    {
        std::scoped_lock lock(stats_mutex);

        const auto now = steady_clock::now();

        // TODO 2026-05-07 (Will Free): I think these bitrate numbers might be wrong, but I'm not sure.
        if (last_probe != time_point<steady_clock>()) {
            if (const auto probe_delta = duration_cast<duration<double>>(now - last_probe).count(); probe_delta > 0.0) {
                static constexpr auto ALPHA = 0.05;

                const auto size = gst_buffer_get_size(buf);

                // size is the size of the array, so times 8 to get bits, over 1000 to get kbits, over probe_delta to get kbits/s
                const auto current_bitrate = static_cast<double>(size) * 8 / 1000 / probe_delta;
                const double old_bitrate = video_stats.bitrate;
                video_stats.bitrate = ALPHA * current_bitrate + (1.0 - ALPHA) * old_bitrate;
            }
        }

        last_probe = now;

        video_stats.average_jitter = duration_cast<milliseconds>(nanoseconds(avg_jitter));
        video_stats.packet_loss = loss;
        video_stats.rtt = nanoseconds(rtx_rtt);
    }

    return Gst::Pad::ProbeReturn::OK;
}

// ReSharper disable once CppDFAUnreachableFunctionCall
void VideoWidget::runPipelineLoop() {
    if (const auto ret = pipeline->set_state(Gst::State::PLAYING); ret == Gst::StateChangeReturn::FAILURE) {
        // TODO 2026-05-21 (Will Free): this does not properly clean up things
        logger.error("Failed to set pipeline to PLAYING");
        pipeline->set_state(Gst::State::NULL_);
    }

    logger.info("RTSP pipeline started");

    const auto bus = pipeline->get_bus();
    reconnect = false;

    // TODO 2026-05-23 (Will Free): I assume it's fine to create multiple main loops?
    // NOTE: if this ever gets uncommented, this block should go above the pipeline->set_state(Gst::State::PLAYING) line.
    // in the header, add: RefPtr<peel::GLib::MainLoop> loop;
    //
    // loop = GLib::MainLoop::create(nullptr, false);
    //
    // bus->add_watch_full(
    //     G_PRIORITY_DEFAULT,
    //     [&](Gst::Bus*, Gst::Message* msg) {
    //         switch (msg->type) {
    //         case Gst::Message::Type::ERROR_: {
    //             peel::UniquePtr<GLib::Error> err = nullptr;
    //             String debug = nullptr;
    //
    //             msg->parse_error(&err, &debug);
    //
    //             logger.error("Error received from element {}: {} ({})", msg->src->get_name(), err->message, debug ? debug : "no debug");
    //
    //             loop->quit();
    //         }
    //         break;
    //         case Gst::Message::Type::EOS:
    //             logger.warn("Got end-of-stream, reconnecting...");
    //
    //             loop->quit();
    //             break;
    //         // TODO 2026-05-23 (Will Free): handle latency message type?
    //         default:
    //             break;
    //         }
    //
    //         return true;
    //     }
    // );

    while (running && !reconnect) {
        const auto msg = bus->timed_pop_filtered(500ms / 1ns, Gst::Message::Type::ERROR_ | Gst::Message::Type::EOS | Gst::Message::Type::LATENCY);

        if (!msg)
            continue;

        switch (msg->type) {
        case Gst::Message::Type::ERROR_: {
            peel::UniquePtr<GLib::Error> err = nullptr;
            String debug = nullptr;

            msg->parse_error(&err, &debug);

            logger.error("Error received from element {}: {} ({})", msg->src->get_name(), err->message, debug ? debug : "no debug");

            reconnect = true;
            break;
        }
        case Gst::Message::Type::EOS:
            logger.warn("Got end-of-stream, reconnecting...");
            reconnect = true;
            break;
        default:
            break;
        }
    }

    pipeline->set_state(Gst::State::NULL_);
}

GstFlowReturn VideoWidget::onNewSample(GstApp::AppSink* sink) {
    const auto sample = sink->pull_sample();

    if (sample == nullptr)
        return GST_FLOW_FLUSHING;

    const auto caps = sample->get_caps();
    const auto caps_structure = caps->get_structure(0);

    int width = 0;
    int height = 0;
    caps_structure->get_int("width", &width);
    caps_structure->get_int("height", &height);

    const auto buffer = sample->get_buffer();
    Gst::MapInfo map; // NOLINT(*-pro-type-member-init)
    if (buffer->map(&map, Gst::MapFlags::READ)) {
        {
            std::scoped_lock lock(frame_mutex);

            if (next_frame.cols != width || next_frame.rows != height || next_frame.type() != CV_8UC4) {
                next_frame = cv::Mat(height, width, CV_8UC4);
            }

            {
                std::scoped_lock stats_lock(stats_mutex);
                video_stats.width = current_frame.cols;
                video_stats.height = current_frame.rows;
            }

            // note: this currently assumes that the matrix type is always a CV_8U/CV_8S
            // and does not account for scenarios where the matrix type uses more than 1 byte per element.
            // this is probably fine, as we only ever use CV_8UC4.
            // so long as that doesn't change, this should be equivalent to just width * height * 4.
            buffer->extract(0, ArrayRef(next_frame.data, static_cast<size_t>(next_frame.rows * next_frame.cols * next_frame.channels())));
            new_frame_available = true;
        }

        buffer->unmap(&map);
    }

    {
        std::scoped_lock lock(stats_mutex);

        const auto now = steady_clock::now();

        last_frame = now;
    }

    return GST_FLOW_OK;
}

void VideoWidget::updateTexture() {
    {
        std::scoped_lock lock(frame_mutex);

        if (!new_frame_available)
            return;

        if (next_frame.empty())
            return;

        std::swap(current_frame, next_frame);
        new_frame_available = false;
    }

    // TODO 2026-05-23 (Will Free): consider using something like glupload in the gstreamer pipeline,
    //  see: https://gstreamer.freedesktop.org/documentation/additional/design/opengl.html

    if (texture_id == 0) {
        glGenTextures(1, &texture_id);
        glBindTexture(GL_TEXTURE_2D, texture_id);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }

    glBindTexture(GL_TEXTURE_2D, texture_id);

    // we do this because glTexImage2D() is more expensive than glTexSubImage2D().
    // if the texture resolution hasn't changed recently, then we can just call
    // glTexSubImage2D().
    if (current_frame.cols != texture_width || current_frame.rows != texture_height) {
        texture_width = current_frame.cols;
        texture_height = current_frame.rows;

        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            GL_RGBA,
            texture_width,
            texture_height,
            0,
            GL_RGBA,
            GL_UNSIGNED_BYTE,
            current_frame.data
        );
    } else {
        glTexSubImage2D(
            GL_TEXTURE_2D,
            0,
            0, 0,
            texture_width,
            texture_height,
            GL_RGBA,
            GL_UNSIGNED_BYTE,
            current_frame.data
        );
    }
}

void VideoWidget::applyRosCameraTopic() const {
    const auto subscription = camera_info_subscription->getSubscription();
    if (subscription && subscription->get_topic_name() == stream_config.camera_topic)
        return;

    camera_info_subscription->subscribe(image_transport::getCameraInfoTopic(stream_config.camera_topic), 10);
}

void VideoWidget::applyMinimap() {
    if (stream_config.minimap)
        addOverlay("minimap", std::make_shared<MiniMapOverlay>(application));
}

void VideoWidget::applyStreamConfig() {
    if (pipeline == nullptr)
        return;

    rtspsrc->set_property<String>("location", stream_config.source_url.data());
    rtspsrc->set_property<gint>("latency", stream_config.rtspsrc_latency);

    jitterbuffer->set_property<gint>("latency", stream_config.jitterbuffer_latency);
}

void VideoWidget::applyFilters(const FilterState& filters, const bool update_flip) const {
    if (pipeline == nullptr)
        return;

    if (balance) {
        balance->set_property<gdouble>("brightness", filters.brightness);
        balance->set_property<gdouble>("contrast", filters.contrast);
        balance->set_property<gdouble>("saturation", filters.saturation);
    }

    if (gamma) {
        gamma->set_property<gdouble>("gamma", filters.gamma);
    }

    if (sharpness) {
        sharpness->set_property<gdouble>("amount", filters.sharpness);
    }

    if (update_flip) {
        applyFlip(filters.rotation);
    }
}

void VideoWidget::applyFlip(VideoFlipMethod rotation) const {
    if (videoflip) {
        videoflip->set_property<gint>("method", static_cast<gint>(rotation));
    }
}
