#include <map>
#include <string>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/rtsp-server/rtsp-server-object.h>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <magic_enum/magic_enum.hpp>

using namespace std;
using namespace rclcpp;

enum RTSPCameraType {
    CAMERA,
    TOPIC,
    TEST,
};

struct RTSPStreamEncoder {
    string encoder;
    string bitrate_param;
    string pipeline;
    string rtph_pay;
};

struct RTSPStreamConfig {
    RTSPCameraType type;
    string source;
    string mountpoint;
    RTSPStreamEncoder encoder;
    long int bitrate;
    long int fps;
    long int width;
    long int height;
};

class RTSPCameraNode final : public Node {
public:
    RTSPCameraNode(): Node("rtsp_camera_node") {
        // Parameters
        this->declare_parameter<int>("port", 8445);
        this->declare_parameter<string>("templates.v4l2", "v4l2src do-timestamp=1");
        this->declare_parameter<string>("templates.appsrc.pipeline",
                                        "appsrc name=imagesrc do-timestamp=true min-latency=0 max-latency=0 max-bytes=1000 is-live=true");
        this->declare_parameter<string>("templates.appsrc.name", "imagesrc");
        this->declare_parameter<string>("templates.test", "videotestsrc");

        appsrc_name = this->get_parameter("templates.appsrc.name").as_string();

        map<string, RTSPStreamEncoder> encoders;
        const auto encoder_names = this->declare_parameter<vector<string>>("templates.encode.encoders");

        for (const auto &encoder_name: encoder_names) {
            RCLCPP_INFO(this->get_logger(), "Adding encoder: %s", encoder_name.c_str());

            encoders[encoder_name] = RTSPStreamEncoder{
                .encoder = this->declare_parameter<string>(fmt::format("templates.encode.{}.encoder", encoder_name)),
                .bitrate_param = this->declare_parameter<string>(fmt::format("templates.encode.{}.bitrate_param", encoder_name)),
                .pipeline = this->declare_parameter<string>(fmt::format("templates.encode.{}.pipeline", encoder_name)),
                .rtph_pay = this->declare_parameter<string>(fmt::format("templates.encode.{}.rtph_pay", encoder_name)),
            };
        }

        // ReSharper disable once CppTooWideScopeInitStatement
        const auto enabled_streams = this->declare_parameter<vector<string>>("enabled_streams");

        for (const auto &stream_name: enabled_streams) {
            RCLCPP_INFO(this->get_logger(), "Adding stream: %s", stream_name.c_str());

            const auto type_str = this->declare_parameter<string>(fmt::format("streams.{}.type", stream_name));

            auto type = magic_enum::enum_cast<RTSPCameraType>(type_str, magic_enum::case_insensitive);

            if (!type.has_value()) {
                constexpr auto rtsp_camera_type_names = magic_enum::enum_names<RTSPCameraType>();
                const auto message = fmt::format("Undefined stream type '{}', available types are: {}. Check your config file.",
                                                 type_str, fmt::join(rtsp_camera_type_names, ", "));
                RCLCPP_FATAL(this->get_logger(), message.c_str());
                throw runtime_error("Undefined stream type");
            }

            const auto encoder_str = this->declare_parameter<string>(fmt::format("streams.{}.encoder", stream_name));

            if (encoders.count(encoder_str) != 1) {
                const auto message = fmt::format("Undefined encoder type '{}', available types are: {}. Check your config file.", encoder_str,
                                                 fmt::join(encoder_names, ", "));
                RCLCPP_FATAL(this->get_logger(), message.c_str());
                throw runtime_error("Undefined encoder type");
            }

            RTSPStreamConfig stream = {
                .type = type.value(),
                .source = this->declare_parameter<string>(fmt::format("streams.{}.source", stream_name)),
                .mountpoint = this->declare_parameter<string>(fmt::format("streams.{}.mountpoint", stream_name)),
                .encoder = encoders[encoder_str],
                .bitrate = this->declare_parameter<int>(fmt::format("streams.{}.bitrate", stream_name)),
                .fps = this->declare_parameter<int>(fmt::format("streams.{}.fps", stream_name), 30),
                .width = this->declare_parameter<int>(fmt::format("streams.{}.width", stream_name), 0),
                .height = this->declare_parameter<int>(fmt::format("streams.{}.height", stream_name), 0),
            };
            streams.push_back(stream);
        }

        // TODO 12/04/25: Add support for updating params at runtime (via add_on_set_parameters_callback)

        // Initialize GStreamer
        gst_init(nullptr, nullptr);

        const auto port = this->get_parameter("port").as_int();

        // Initialize gstreamer
        gstreamer_thread = thread(&mainloop, this);
        // pthread_t tloop;

        // pthread_create(&tloop, nullptr, &mainloop, nullptr);

        rtsp_server = rtsp_server_create(static_cast<int>(port));

        // Go through and parse each stream
        // ReSharper disable once CppUseStructuredBinding
        for (const auto &stream: streams) {
            RCLCPP_DEBUG(this->get_logger(), "Found stream %s ==> %s", stream.source.c_str(), stream.mountpoint.c_str());

            const auto pipeline = create_pipeline(&stream);

            RCLCPP_INFO(this->get_logger(), "Stream available at rtsp://%s:%ld%s", gst_rtsp_server_get_address(rtsp_server), port, stream.mountpoint.c_str());
            RCLCPP_INFO(this->get_logger(), "Pipeline: %s", pipeline.c_str());
        }
    }

    ~RTSPCameraNode() override {
        gstreamer_thread.join();
    }

private:
    vector<RTSPStreamConfig> streams;

    // ROS publisher
    Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    GstRTSPServer *rtsp_server;
    // GStreamer stuff
    map<string, Subscription<sensor_msgs::msg::Image>::SharedPtr> subs;
    map<string, GstAppSrc *> appsrc;
    map<string, int> num_of_clients;
    string appsrc_name;

    thread gstreamer_thread;

    string create_pipeline(const RTSPStreamConfig *stream_config) {
        const auto v4l2_pipeline_template = this->get_parameter("templates.v4l2").as_string();
        const auto appsrc_pipeline_template = this->get_parameter("templates.appsrc.pipeline").as_string();
        const auto test_pipeline_template = this->get_parameter("templates.test").as_string();

        const auto encoder = &stream_config->encoder.encoder;
        const auto encoder_pipeline = &stream_config->encoder.pipeline;
        const auto bitrate_param = &stream_config->encoder.bitrate_param;
        const auto rtph_pay = &stream_config->encoder.rtph_pay;

        string src;
        GstAppSrc **appsrc = nullptr;

        if (stream_config->type == CAMERA) {
            src = fmt::format("{} device={}", v4l2_pipeline_template, stream_config->source);
        } else if (stream_config->type == TOPIC) {
            src = appsrc_pipeline_template;
            /* Keep record of number of clients connected to each topic.
             * so we know to stop subscribing when no-one is connected. */
            num_of_clients[stream_config->mountpoint] = 0;
            this->appsrc[stream_config->mountpoint] = nullptr;
            appsrc = &this->appsrc[stream_config->mountpoint];
        } else if (stream_config->type == TEST) {
            src = test_pipeline_template;
        }

        const auto full_encoder_pipeline = fmt::format("{} {}={} ! {}", *encoder, *bitrate_param, stream_config->bitrate, *encoder_pipeline);
        auto convert = fmt::format("videoconvert ! videoscale ! videorate ! video/x-raw,framerate={}/1", stream_config->fps);
        if (stream_config->width > 0)
            convert = fmt::format("{},width={}", convert, stream_config->width);
        if (stream_config->height > 0)
            convert = fmt::format("{},height={}", convert, stream_config->height);

        const auto pipeline = fmt::format("{} ! {} ! {} ! {}", src, convert, full_encoder_pipeline, *rtph_pay);
        rtsp_server_add_url(stream_config->mountpoint, pipeline, appsrc);

        return pipeline;
    }

    GstRTSPServer *rtsp_server_create(const int port) {
        /* create a server instance */
        const auto server = gst_rtsp_server_new();

        gst_rtsp_server_set_service(server, fmt::format_int(port).c_str());

        /* attach the server to the default maincontext */
        gst_rtsp_server_attach(server, nullptr);

        g_signal_connect(server, "client-connected", G_CALLBACK(new_client), this);

        /* add a timeout for the session cleanup */
        g_timeout_add_seconds(4, reinterpret_cast<GSourceFunc>(session_cleanup), this);

        return server;
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg, const string &topic) {
        // g_print("Image encoding: %s\n", msg->encoding.c_str());
        if (appsrc[topic] != nullptr) {
            // Set caps from message
            const auto caps = gst_caps_new_from_ros_image(msg);
            gst_app_src_set_caps(appsrc[topic], caps);

            const auto buf = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
            gst_buffer_fill(buf, 0, msg->data.data(), msg->data.size());
            GST_BUFFER_FLAG_SET(buf, GST_BUFFER_FLAG_LIVE);

            gst_app_src_push_buffer(appsrc[topic], buf);
        }
    }

    static void client_options(GstRTSPClient *, const GstRTSPContext *state, RTSPCameraNode *node) {
        if (state->uri) {
            node->url_connected(state->uri->abspath);
        }
    }


    static void client_teardown(GstRTSPClient *, const GstRTSPContext *state, RTSPCameraNode *node) {
        if (state->uri) {
            node->url_disconnected(state->uri->abspath);
        }
    }

    static void new_client(GstRTSPServer *, GstRTSPClient *client, RTSPCameraNode *node) {
        const auto client_connection = gst_rtsp_client_get_connection(client);
        const auto client_url = gst_rtsp_url_get_request_uri(gst_rtsp_connection_get_url(client_connection));
        const auto client_ip = gst_rtsp_connection_get_ip(client_connection);
        RCLCPP_INFO(node->get_logger(), "New RTSP client connecting from %s (%s)", client_url, client_ip);
        g_signal_connect(client, "options-request", G_CALLBACK(client_options), node);
        g_signal_connect(client, "teardown-request", G_CALLBACK(client_teardown), node);
    }

    static gboolean session_cleanup(const RTSPCameraNode *node, gboolean) {
        GstRTSPServer *server = node->rtsp_server;

        auto *pool = gst_rtsp_server_get_session_pool(server);
        const auto num = gst_rtsp_session_pool_cleanup(pool);
        g_object_unref(pool);

        if (num > 0) {
            RCLCPP_INFO(node->get_logger(), "Cleaning up session %d", num);
        }

        return TRUE;
    }

    void url_connected(const string &url) {
        RCLCPP_INFO(this->get_logger(), "Client connected: %s", url.c_str());

        for (const auto &stream: streams) {
            if (stream.type == TOPIC && url == stream.mountpoint) {
                if (num_of_clients[url] == 0) {
                    auto imageCallback = [this, mountpoint = stream.mountpoint](const sensor_msgs::msg::Image::ConstSharedPtr &image) {
                        this->imageCallback(image, mountpoint);
                    };
                    const auto subscription = this->create_subscription<sensor_msgs::msg::Image>(stream.source, 1, imageCallback);
                    // Subscribe to the ROS topic
                    subs[url] = subscription;
                }
                num_of_clients[url]++;
            }
        }
    }

    void url_disconnected(const string &url) {
        RCLCPP_INFO(this->get_logger(), "Client disconnected: %s", url.c_str());

        for (const auto &stream: streams) {
            if (url == stream.mountpoint) {
                if (num_of_clients[url] > 0) num_of_clients[url]--;
                if (num_of_clients[url] == 0) {
                    // No-one else is connected, stop the subscription by letting it go out of scope
                    subs[url] = nullptr;
                    appsrc[url] = nullptr;
                }
            }
        }
    }

    GstCaps *gst_caps_new_from_ros_image(const sensor_msgs::msg::Image::ConstSharedPtr &msg) const {
        // https://gstreamer.freedesktop.org/documentation/additional/design/mediatype-video-raw.html?gi-language=c#formats
        static const map<string, string> known_formats = {
            {
                {sensor_msgs::image_encodings::RGB8, "RGB"},
                {sensor_msgs::image_encodings::RGBA8, "RGBA"},
                // {sensor_msgs::image_encodings::RGB16, "RGB16"}, // RGB16 does not seem to be supported
                {sensor_msgs::image_encodings::RGBA16, "RGBA64_LE"},
                {sensor_msgs::image_encodings::BGR8, "BGR"},
                // {sensor_msgs::image_encodings::BGR16, "BGRx64_LE"}, // BGR16 does not seem to be supported
                {sensor_msgs::image_encodings::BGRA8, "BGRA"},
                {sensor_msgs::image_encodings::BGRA16, "BGRA64_LE"},
                {sensor_msgs::image_encodings::MONO8, "GRAY8"},
                {sensor_msgs::image_encodings::MONO16, "GRAY16_LE"},
            }
        };

        if (msg->is_bigendian) {
            RCLCPP_INFO(this->get_logger(), "GST: big endian image format is not supported");
            return nullptr;
        }

        const auto format = known_formats.find(msg->encoding);
        if (format == known_formats.end()) {
            RCLCPP_INFO(this->get_logger(), "GST: image format '%s' unknown", msg->encoding.c_str());
            return nullptr;
        }

        return gst_caps_new_simple("video/x-raw",
                                   "format", G_TYPE_STRING, format->second.c_str(),
                                   "width", G_TYPE_INT, msg->width,
                                   "height", G_TYPE_INT, msg->height,
                                   "framerate", GST_TYPE_FRACTION, 10, 1,
                                   nullptr);
    }


    void rtsp_server_add_url(const string &url, const string &pipeline, GstAppSrc **appsrc) const {
        /* get the mount points for this server, every server has a default object
        * that be used to map uri mount points to media factories */
        const auto mounts = gst_rtsp_server_get_mount_points(rtsp_server);

        /* make a media factory for a test stream. The default media factory can use
         * gst-launch syntax to create pipelines.
         * any launch line works as long as it contains elements named pay%d. Each
         * element with pay%d names will be a stream */
        const auto factory = gst_rtsp_media_factory_new();
        gst_rtsp_media_factory_set_launch(factory, pipeline.c_str());

        /* notify when our media is ready, This is called whenever someone asks for
         * the media and a new pipeline is created */
        g_signal_connect(factory, "media-configure", reinterpret_cast<GCallback>(media_configure), appsrc);

        gst_rtsp_media_factory_set_shared(factory, true);

        gst_rtsp_media_factory_set_enable_rtcp(factory, true);

        /* attach the factory to the url */
        gst_rtsp_mount_points_add_factory(mounts, url.c_str(), factory);

        /* don't need the ref to the mounts anymore */
        g_object_unref(mounts);
    }


    /* called when a new media pipeline is constructed. We can query the
     * pipeline and configure our appsrc */
    static void media_configure(GstRTSPMediaFactory *, GstRTSPMedia *media, GstElement **appsrc) {
        // What is this?
        if (appsrc) {
            const auto pipeline = gst_rtsp_media_get_element(media);

            *appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "imagesrc");

            /* this instructs appsrc that we will be dealing with timed buffer */
            gst_util_set_object_arg(G_OBJECT(*appsrc), "format", "time");

            gst_object_unref(pipeline);
        } else {
            fmt::println("Initializing media factory...");

            const auto n_streams = gst_rtsp_media_n_streams(media);

            for (uint i = 0; i < n_streams; i++) {
                const auto stream = gst_rtsp_media_get_stream(media, i);

                /* make a new address pool */
                const auto pool = gst_rtsp_address_pool_new();

                gchar *min = g_strdup_printf("224.3.0.%d", 2 * i + 1);
                gchar *max = g_strdup_printf("224.3.0.%d", 2 * i + 2);
                gst_rtsp_address_pool_add_range(pool, min, max,
                                                5000 + 10 * i, 5010 + 10 * i, 1);
                g_free(min);
                g_free(max);

                gst_rtsp_stream_set_address_pool(stream, pool);
                g_object_unref(pool);
            }

            fmt::println("Initialized media factory with {} streams", 1);
        }
    }


    static void *mainloop(void *) {
        GMainLoop *loop = g_main_loop_new(nullptr, false);

        g_main_loop_run(loop);

        g_main_loop_unref(loop);
        return nullptr;
    }
};


int main(const int argc, const char *const argv[]) {
    init(argc, argv);
    spin(make_shared<RTSPCameraNode>());
    shutdown();

    return 0;
}
