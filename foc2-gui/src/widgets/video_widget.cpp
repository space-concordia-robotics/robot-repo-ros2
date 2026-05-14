#include "foc2-gui/widgets/video_widget.hpp"

#include <imgui.h>
#include <gst/gstbin.h>
#include <gst/app/gstappsink.h>

#include "foc2-gui/overlays/aruco_video_overlay.hpp"
#include "foc2-gui/overlays/crosshair_overlay.hpp"
#include "foc2-gui/overlays/minimap.hpp"
#include "foc2-gui/overlays/video_stats_overlay.hpp"


using namespace std::chrono;
using namespace std::chrono_literals;

VideoWidget::VideoWidget(ImApplication& application, const std::string& source_url, const bool minimap, const std::string& videoflip)
    : UiWidget(application),
      source_url(source_url), minimap(minimap), videoflip(videoflip) {
    addOverlay(std::make_shared<CrosshairOverlay>(application));
    video_stats_overlay = std::make_shared<VideoStatsOverlay>(application);
    addOverlay(std::make_shared<ArucoVideoOverlay>(application, "/rover/ffc/front/image_raw", "ffc_front_camera"));

    addOverlay(video_stats_overlay);

    if (minimap)
        addOverlay(std::make_shared<MiniMapOverlay>(application));

    // if this 50ms value is changed, also update the value in the video stats overlay
    stats_timer = this->application.create_timer(50ms, [this] {
        if (this->video_stats_overlay) {
            std::lock_guard lock(this->stats_mutex);

            this->video_stats_overlay->updateStats(this->video_stats);
        }
    });
}

void VideoWidget::onInit() {
    UiWidget::onInit();
    UiOverlayable::onInit();

    gst_thread = std::thread(std::bind(&VideoWidget::videoThread, this));
}

void VideoWidget::onShutdown() {
    UiWidget::onShutdown();
    UiOverlayable::onShutdown();

    running = false;
    gst_thread.join();
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
    updateTexture();

    ImGui::BeginChild("VideoRegion", ImVec2(0, 0), 0, ImGuiWindowFlags_NoScrollbar);

    // TODO 2026-05-07 (Will Free): tint entire frame red or gray or something when no frame has been received for like 1-2s

    const auto avail = ImGui::GetContentRegionAvail();

    if (texture_id) {
        const float video_aspect = static_cast<float>(current_frame.cols) / static_cast<float>(current_frame.rows);

        ImVec2 size = avail;

        if (const float window_aspect = avail.x / avail.y; window_aspect > video_aspect) {
            size.x = avail.y * video_aspect;
        } else {
            size.y = avail.x / video_aspect;
        }

        ImGui::SetCursorPosX((avail.x - size.x) * 0.5f);
        // ImGui::SetCursorPosY((avail.y - size.y) * 0.5f);

        ImGui::Image(texture_id, size);

        drawOverlays(ImGui::GetWindowDrawList(), ImRect(ImGui::GetItemRectMin(), ImGui::GetItemRectMax()));
    } else {
        // TODO 2026-05-07 (Will Free): show message if we got disconnected from the stream & are reconnecting

        static constexpr auto AWAITING_FRAME_TEXT = "Waiting for video frame...";

        const auto draw_list = ImGui::GetWindowDrawList();

        const auto top_left = ImGui::GetCursorScreenPos();

        const auto text_size = ImGui::CalcTextSize(AWAITING_FRAME_TEXT);

        draw_list->AddText(top_left + avail * 0.5 - text_size * 0.5, ImGui::ImColor(255, 255, 255, 255), AWAITING_FRAME_TEXT);

        ImGui::Dummy(avail);
    }

    ImGui::EndChild();
}

void VideoWidget::videoThread() {
    // TODO 2026-05-07 (Will Free): replace this pipeline string with creating each of the elements individually
    //  via gst_element_factory_make(), then linking them together.

    const auto pipeline_desc = fmt::format(
        "rtspsrc name=src location={} latency=100 ! "
        "rtpjitterbuffer name=jitter latency=50 ! "
        "rtph264depay ! "
        "decodebin ! "
        "videoconvert ! "
        "videoflip method={} ! "
        "video/x-raw,format=RGBA ! "
        "appsink name=appsink sync=true", // is sync needed here?
        source_url, videoflip
    );

    while (running) {
        GError* error = nullptr;
        pipeline = gst_parse_launch(pipeline_desc.c_str(), &error);

        if (!pipeline || error) {
            logger.error("Failed to create GStreamer pipeline: {}", error ? error->message : "unknown");
            if (error) g_error_free(error);
            std::this_thread::sleep_for(1s);
            continue;
        }

        jitterbuffer = gst_bin_get_by_name(GST_BIN(pipeline), "jitter");

        appsink = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline), "appsink"));
        if (!appsink) {
            logger.error("Failed to get appsink from pipeline");
            gst_object_unref(pipeline);
            pipeline = nullptr;
            std::this_thread::sleep_for(1s);
            continue;
        }

        gst_app_sink_set_emit_signals(appsink, true);
        gst_app_sink_set_drop(appsink, true);
        gst_app_sink_set_max_buffers(appsink, 1);
        // gst_base_sink_set_sync(GST_BASE_SINK(appsink), true); // TODO 2026-05-06 (Will Free): is this needed?

        auto appsink_callbacks = GstAppSinkCallbacks{};
        appsink_callbacks.new_sample = [](GstAppSink* appsink, void* widget) -> GstFlowReturn {
            return static_cast<VideoWidget*>(widget)->onNewSample(appsink);
        };

        gst_app_sink_set_callbacks(appsink, &appsink_callbacks, this, nullptr);

        // TODO 2026-05-07 (Will Free): we are currently not recording the latency of the RTSP stream.
        //  this can be done by listening to RTCP packets from the RTSP stream,
        //  and then getting the difference between the current time and the time in the packet.
        video_stats.rtsp_latency = milliseconds(0);

        if (const auto jitterbuffer_sink_pad = gst_element_get_static_pad(jitterbuffer, "sink")) {
            gst_pad_add_probe(
                jitterbuffer_sink_pad,
                GST_PAD_PROBE_TYPE_BUFFER,
                // ReSharper disable once CppParameterMayBeConstPtrOrRef
                [](GstPad*, GstPadProbeInfo* info, const gpointer user_data) -> GstPadProbeReturn {
                    const auto self = static_cast<VideoWidget*>(user_data);

                    if (!info)
                        return GST_PAD_PROBE_OK;

                    const auto buf = GST_PAD_PROBE_INFO_BUFFER(info);

                    if (!buf)
                        return GST_PAD_PROBE_OK;

                    auto avg_jitter = 0uL;
                    auto num_pushed = 0uL;
                    auto num_lost = 0uL;
                    auto rtx_rtt = 0uL;
                    auto loss = 0.0;

                    if (self->jitterbuffer) {
                        GstStructure* stats = nullptr;
                        g_object_get(self->jitterbuffer, "stats", &stats, nullptr);

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
                        std::lock_guard lock(self->stats_mutex);

                        const auto now = steady_clock::now();

                        // TODO 2026-05-07 (Will Free): I think these bitrate numbers might be wrong, but I'm not sure.
                        if (self->last_probe != time_point<steady_clock>()) {
                            if (const auto probe_delta = duration_cast<duration<double>>(now - self->last_probe).count(); probe_delta > 0.0) {
                                static constexpr auto ALPHA = 0.05;

                                const auto size = gst_buffer_get_size(buf);

                                // size is the size of the array, so times 8 to get bits, over 1000 to get kbits, over probe_delta to get kbits/s
                                const auto current_bitrate = static_cast<double>(size) * 8 / 1000 / probe_delta;
                                const double old_bitrate = self->video_stats.bitrate;
                                self->video_stats.bitrate = ALPHA * current_bitrate + (1.0 - ALPHA) * old_bitrate;
                            }
                        }

                        self->last_probe = now;

                        self->video_stats.average_jitter = duration_cast<milliseconds>(nanoseconds(avg_jitter));
                        self->video_stats.packet_loss = loss;
                        self->video_stats.rtt = nanoseconds(rtx_rtt);
                    }

                    return GST_PAD_PROBE_OK;
                },
                this,
                nullptr
            );

            gst_object_unref(jitterbuffer_sink_pad);
        }

        if (const auto ret = gst_element_set_state(pipeline, GST_STATE_PLAYING); ret == GST_STATE_CHANGE_FAILURE) {
            logger.error("Failed to set pipeline to PLAYING");
            gst_element_set_state(pipeline, GST_STATE_NULL);
            gst_object_unref(appsink);
            gst_object_unref(pipeline);
            appsink = nullptr;
            pipeline = nullptr;
            std::this_thread::sleep_for(seconds(1));
            continue;
        }

        logger.info("RTSP pipeline started");

        GstBus* bus = gst_element_get_bus(pipeline);
        bool reconnect = false;

        while (running && !reconnect) {
            using namespace std::chrono_literals;

            const auto msg = gst_bus_timed_pop_filtered(
                bus,
                500ms / 1ns,
                static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS | GST_MESSAGE_LATENCY)
            );

            if (!msg)
                continue;

            switch (msg->type) {
            case GST_MESSAGE_ERROR: {
                GError* err = nullptr;
                gchar* debug = nullptr;

                gst_message_parse_error(msg, &err, &debug);

                logger.error("Error received from element {}: {} ({})", msg->src->name, err->message, debug ? debug : "no debug");

                g_clear_error(&err);
                g_free(debug);

                reconnect = true;
                break;
            }
            case GST_MESSAGE_EOS:
                logger.warn("Got end-of-stream, reconnecting...");
                reconnect = true;
                break;
            default:
                break;
            }

            gst_message_unref(msg);
        }

        gst_object_unref(bus);

        gst_element_set_state(pipeline, GST_STATE_NULL);
        if (appsink) gst_object_unref(appsink);
        if (jitterbuffer) gst_object_unref(jitterbuffer);
        if (pipeline) gst_object_unref(pipeline);

        appsink = nullptr;
        jitterbuffer = nullptr;
        pipeline = nullptr;

        if (running) {
            std::this_thread::sleep_for(seconds(1));
        }
    }
}

GstFlowReturn VideoWidget::onNewSample(GstAppSink* sink) {
    const auto sample = gst_app_sink_pull_sample(sink);

    if (!sample)
        return GST_FLOW_FLUSHING;

    const GstCaps* caps = gst_sample_get_caps(sample);
    const GstStructure* s = gst_caps_get_structure(caps, 0);

    int width = 0, height = 0;
    gst_structure_get_int(s, "width", &width);
    gst_structure_get_int(s, "height", &height);

    GstBuffer* buffer = gst_sample_get_buffer(sample);
    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        {
            std::lock_guard lock(frame_mutex);

            if (next_frame.cols != width || next_frame.rows != height || next_frame.type() != CV_8UC4) {
                next_frame = cv::Mat(height, width, CV_8UC4);
            }

            {
                std::lock_guard stats_lock(stats_mutex);
                video_stats.width = current_frame.cols;
                video_stats.height = current_frame.rows;
            }

            std::memcpy(next_frame.data, map.data, width * height * 4);
            new_frame_available = true;
        }

        gst_buffer_unmap(buffer, &map);
    }

    gst_sample_unref(sample);

    {
        std::lock_guard lock(stats_mutex);

        const auto now = steady_clock::now();

        if (last_frame != time_point<steady_clock>()) {
            if (const auto frame_delta = duration_cast<duration<double>>(now - last_frame).count(); frame_delta > 0.0) {
                video_stats.fps = 1.0 / frame_delta;
            }
        }

        last_frame = now;
    }

    return GST_FLOW_OK;
}

void VideoWidget::updateTexture() {
    {
        auto lock = std::lock_guard(frame_mutex);
        if (!new_frame_available)
            return;

        if (next_frame.empty())
            return;

        std::swap(current_frame, next_frame);
        new_frame_available = false;
    }

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
