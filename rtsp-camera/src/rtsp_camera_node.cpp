#include <boost/numeric/conversion/cast.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <gst/gst.h>
#include <gstreamer-1.0/gst/gstclock.h>
#include <magic_enum/magic_enum.hpp>
#include <opencv2/opencv.hpp>
#include <rtsp_camera/rtsp_camera.hpp>


RTSPCameraNode::RTSPCameraNode()
    : Node("rtsp_camera_node"),
      logger(this->get_logger()), camera_info_manager(this) {
    // Parameters
    this->declare_parameter<int>("port", 8445);
    this->declare_parameter<string>("templates.v4l2", "v4l2src do-timestamp=1");
    this->declare_parameter<string>("templates.appsink.pipeline", "appsink");
    this->declare_parameter<string>("templates.appsink.name", "imagesink");
    this->declare_parameter<string>("templates.appsrc.pipeline",
                                    "appsrc name=imagesrc do-timestamp=true min-latency=0 max-latency=0 max-bytes=1000 is-live=true");
    this->declare_parameter<string>("templates.appsrc.name", "imagesrc");
    this->declare_parameter<string>("templates.test", "videotestsrc");
    this->declare_parameter<string>("templates.intervideosink", "intervideosink");
    this->declare_parameter<string>("templates.intervideosrc", "intervideosrc");

    appsrc_name = this->get_parameter("templates.appsrc.name").as_string();
    appsink_name = this->get_parameter("templates.appsink.name").as_string();

    map<string, RTSPStreamEncoder> encoders;
    const auto encoder_names = this->declare_parameter<vector<string>>("templates.encode.encoders");

    for (const auto& encoder_name : encoder_names) {
        logger.info("Adding encoder: {}", encoder_name);

        encoders[encoder_name] = RTSPStreamEncoder{
            .encoder       = this->declare_parameter<string>(fmt::format("templates.encode.{}.encoder", encoder_name)),
            .bitrate_param = this->declare_parameter<string>(fmt::format("templates.encode.{}.bitrate_param", encoder_name)),
            .pipeline      = this->declare_parameter<string>(fmt::format("templates.encode.{}.pipeline", encoder_name)),
            .rtp_pay       = this->declare_parameter<string>(fmt::format("templates.encode.{}.rtph_pay", encoder_name)),
        };
    }

    // ReSharper disable once CppTooWideScopeInitStatement
    const auto enabled_streams = this->declare_parameter<vector<string>>("enabled_streams");

    for (const auto& stream_name : enabled_streams) {
        logger.info("Adding stream: {}", stream_name);

        const auto type_str = this->declare_parameter<string>(fmt::format("streams.{}.type", stream_name));

        auto type = magic_enum::enum_cast<RTSPCameraType>(type_str, magic_enum::case_insensitive);

        if (!type.has_value()) {
            constexpr auto rtsp_camera_type_names = magic_enum::enum_names<RTSPCameraType>();
            logger.fatal("Undefined stream type '{}', available types are: {}. Check your config file.", type_str, fmt::join(rtsp_camera_type_names, ", "));
            throw runtime_error("Undefined stream type");
        }

        const auto encoder_str = this->declare_parameter<string>(fmt::format("streams.{}.encoder", stream_name));

        if (encoders.count(encoder_str) != 1) {
            const auto message = fmt::format("Undefined encoder type '{}', available types are: {}. Check your config file.", encoder_str,
                                             fmt::join(encoder_names, ", "));
            logger.fatal("Undefined encoder type '{}', available types are: {}. Check your config file.", encoder_str, fmt::join(encoder_names, ", "));
            throw runtime_error("Undefined encoder type");
        }

        const auto image_topic_enabled = this->declare_parameter<bool>(fmt::format("streams.{}.image_topic_enabled", stream_name));
        ROSImageTopicConfig publishing;

        const auto width = this->declare_parameter<int>(fmt::format("streams.{}.width", stream_name), 0);
        const auto height = this->declare_parameter<int>(fmt::format("streams.{}.height", stream_name), 0);

        if (image_topic_enabled) {
            const auto scale = this->declare_parameter<double>(fmt::format("streams.{}.image_topic.scale", stream_name));

            publishing = {
                .topic_name   = this->declare_parameter<string>(fmt::format("streams.{}.image_topic.name", stream_name)),
                .uncompressed = this->declare_parameter<bool>(fmt::format("streams.{}.image_topic.uncompressed", stream_name)),
                .compressed   = this->declare_parameter<bool>(fmt::format("streams.{}.image_topic.compressed", stream_name)),
                // .ffmpeg = this->declare_parameter<bool>(fmt::format("streams.{}.image_topic.ffmpeg", stream_name)),
                .fps             = this->declare_parameter<int>(fmt::format("streams.{}.image_topic.fps", stream_name)),
                .width           = lround(width * scale),
                .height          = lround(height * scale),
                .format          = "RGB", // for now we're only supporting RGB
                .sensor_data_qos = this->declare_parameter<bool>("use_sensor_data_qos", false),
            };
        }

        RTSPStreamConfig stream = {
            .name       = stream_name,
            .type       = type.value(),
            .source     = this->declare_parameter<string>(fmt::format("streams.{}.source", stream_name)),
            .mountpoint = this->declare_parameter<string>(fmt::format("streams.{}.mountpoint", stream_name)),
            .encoder    = encoders[encoder_str],
            .bitrate    = this->declare_parameter<int>(fmt::format("streams.{}.bitrate", stream_name)),
            .fps        = this->declare_parameter<int>(fmt::format("streams.{}.fps", stream_name), 30),
            .width      = width,
            .height     = height,
            .publishing = publishing,
        };
        streams.push_back(stream);
    }

    // TODO 2025-04-12: Add support for updating params at runtime (via add_on_set_parameters_callback)

    // Initialize GStreamer
    gst_init(nullptr, nullptr);

    const auto port = this->get_parameter("port").as_int();

    // Initialize gstreamer
    gstreamer_thread = thread(&mainloop, this);
    // pthread_t tloop;

    // pthread_create(&tloop, nullptr, &mainloop, nullptr);

    rtsp_server = rtsp_server_create(static_cast<int>(port));

    GstClock* clock = gst_system_clock_obtain();
    GstClockTime ct = gst_clock_get_time(clock);
    gst_object_unref(clock);
    time_offset = now().nanoseconds() - GST_TIME_AS_NSECONDS(ct);

    // Go through and parse each stream
    // ReSharper disable once CppUseStructuredBinding
    for (const auto& stream : streams) {
        logger.debug("Found stream {} ==> {}", stream.source, stream.mountpoint);

        create_pipeline(stream);

        logger.info("Stream available at rtsp://{}:{}{}", gst_rtsp_server_get_address(rtsp_server), port, stream.mountpoint);
    }

    // const auto pipeline_thread = thread([this] {
    //
    // });
    //
    // this->pipeline_threads.push_back(pipeline_thread);
}

RTSPCameraNode::~RTSPCameraNode() {
    gst_deinit();
    g_main_loop_quit(main_loop);
    gstreamer_thread.join();
}

void RTSPCameraNode::create_pipeline(const RTSPStreamConfig& stream_config) {
    // TODO 2025-10-18: Stop templating the pipeline and instead build the pipeline up using gst_element_factory_make()
    // const auto v4l2_pipeline_template = this->get_parameter("templates.v4l2").as_string();
    // const auto appsink_pipeline_template = this->get_parameter("templates.appsink.pipeline").as_string();
    // const auto appsrc_pipeline_template = this->get_parameter("templates.appsrc.pipeline").as_string();
    // const auto test_pipeline_template = this->get_parameter("templates.test").as_string();
    // const auto intersink_pipeline_template = this->get_parameter("templates.intersink").as_string();
    // const auto intersrc_pipeline_template = this->get_parameter("templates.intersrc").as_string();
    //
    // const auto appsink_template = fmt::format("{} name={}", appsink_pipeline_template, appsink_name);
    // const auto appsrc_template = fmt::format("{} name={}", appsrc_pipeline_template, appsrc_name);
    //
    // const auto encoder = &stream_config.encoder.encoder;
    // const auto encoder_pipeline = &stream_config.encoder.pipeline;
    // const auto bitrate_param = &stream_config.encoder.bitrate_param;
    // const auto rtph_pay = &stream_config.encoder.rtp_pay;

    // const auto full_encoder_pipeline = fmt::format("{} {}={} ! {}", *encoder, *bitrate_param, stream_config.bitrate, *encoder_pipeline);

    string pipeline;
    GstAppSrc* appsrc = nullptr;
    GstAppSink* appsink = nullptr;

    const auto source_pipeline = create_source_pipeline(stream_config);
    const auto rtsp_pipeline = create_rtsp_pipeline(stream_config);
    GstElement* topic_pipeline = nullptr;

    if (stream_config.type == TOPIC) {
        appsrc = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(source_pipeline), appsrc_name.c_str()));
    }

    if (stream_config.publishing) {
        topic_pipeline = create_topic_pipeline(stream_config);
        appsink = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(topic_pipeline), appsink_name.c_str()));
    }

    auto stream_pipeline = RTSPStreamPipeline{
        .source_pipeline = source_pipeline,
        // .rtsp_pipeline = rtsp_pipeline,
        .topic_pipeline = topic_pipeline,
        .appsrc         = appsrc,
        .appsink        = appsink,
    };


    const auto topic_config = stream_config.publishing;
    rtsp_server_add_url(stream_config.mountpoint, rtsp_pipeline, stream_pipeline, topic_config);

    start_pipeline(stream_config, stream_pipeline);
}

void RTSPCameraNode::start_pipeline(const RTSPStreamConfig& stream_config, RTSPStreamPipeline& stream_pipeline) {
    if (const auto appsink = stream_pipeline.appsink; appsink != nullptr) {
        // gst_app_sink_set_emit_signals(appsink, true);

        logger.info("creating image publisher...");

        const auto publishing = *stream_config.publishing;

        const auto qos = publishing.sensor_data_qos ? SensorDataQoS() : QoS{1};

        stream_pipeline.publisher = this->create_publisher<sensor_msgs::msg::Image>(fmt::format("/rtsp_camera/{}/image_raw", publishing.topic_name), qos);

        struct CallbackData {
            RTSPCameraNode* node;
            const RTSPStreamConfig* stream_config;
            const Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
            const ROSImageTopicConfig publishing;
            GstClockTime base_time;
        };

        logger.info("does publishing have value? {}", stream_config.publishing.has_value());
        logger.info("resolution: {}x{}", publishing.width, publishing.height);

        auto new_sample_callback = [](GstAppSink* app_sink, void* user_data)-> GstFlowReturn {
            const auto data = static_cast<CallbackData*>(user_data);

            /* Retrieve the buffer */
            if (const auto sample = gst_app_sink_pull_sample(app_sink)) {
                // const auto time = data->base_time;

                const auto msg = data->node->img_msg_from_gst_sample(sample, &data->publishing);

                // const auto buffer = gst_sample_get_buffer(sample);
                // const auto caps = gst_sample_get_caps(sample);
                // const auto structure = gst_caps_get_structure(caps, 0);
                //
                // // Get width and height
                // int width, height;
                //
                // gst_structure_get_int(structure, "width", &width);
                // gst_structure_get_int(structure, "height", &height);
                //
                // GstMapInfo map;
                // sensor_msgs::msg::Image::SharedPtr msg;
                // if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
                //     // Convert to OpenCV Mat
                //     auto frame = cv::Mat(cv::Size(width, height), CV_8UC3, map.data, cv::Mat::AUTO_STEP);
                //
                //     // Resize the frame if necessary
                //     // if (width != config->width || height != config->height) {
                //     //     const auto resized = cv::Mat(cv::Size(config->width, config->height), CV_8UC3);
                //     //     cv::resize(frame, resized, resized.size());
                //     //     frame = resized;
                //     // }
                //
                //     // Convert to ROS Image message
                //     msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", frame).toImageMsg();
                //
                //     gst_buffer_unmap(buffer, &map);
                // }
                //
                // gst_sample_unref(sample);

                msg->header.stamp = data->node->now();
                // msg->header.stamp = Time(GST_TIME_AS_NSECONDS(buffer->pts + data->base_time) + data->node->time_offset);
                msg->header.frame_id = data->stream_config->name;

                // Publish the image
                data->publisher->publish(*msg);

                gst_sample_unref(sample);
                return GST_FLOW_OK;
            }

            return GST_FLOW_FLUSHING;
        };

        auto callbacks = GstAppSinkCallbacks{
            .new_sample = new_sample_callback
        };

        auto data = CallbackData{
            .node          = this,
            .stream_config = &stream_config,
            .publisher     = stream_pipeline.publisher,
            .publishing    = publishing,
            .base_time     = gst_element_get_base_time(stream_pipeline.topic_pipeline),
        };

        gst_app_sink_set_callbacks(appsink, &callbacks, &data, nullptr);

        gst_element_set_state(stream_pipeline.topic_pipeline, GST_STATE_PLAYING);
    }

    if (const auto appsrc = stream_pipeline.appsrc; appsrc != nullptr) {
        auto image_callback = [this, appsrc](const sensor_msgs::msg::Image::ConstSharedPtr& image) {
            // Set caps from message
            const auto caps = gst_caps_new_from_ros_image(image);
            gst_app_src_set_caps(appsrc, caps);

            const auto buf = gst_buffer_new_allocate(nullptr, image->data.size(), nullptr);
            gst_buffer_fill(buf, 0, image->data.data(), image->data.size());
            GST_BUFFER_FLAG_SET(buf, GST_BUFFER_FLAG_LIVE);

            gst_app_src_push_buffer(appsrc, buf);
            // this->imageCallback(image, stream_pipeline.appsrc);
        };

        stream_pipeline.subscription = this->create_subscription<sensor_msgs::msg::Image>(stream_config.source, 1, image_callback);
    }

    gst_element_set_state(stream_pipeline.source_pipeline, GST_STATE_PLAYING);
}

sensor_msgs::msg::Image::SharedPtr RTSPCameraNode::img_msg_from_gst_sample(GstSample* sample, const ROSImageTopicConfig* /*config*/) {
    const auto buffer = gst_sample_get_buffer(sample);
    const auto caps = gst_sample_get_caps(sample);
    const auto structure = gst_caps_get_structure(caps, 0);

    // Get width and height
    int width, height;

    gst_structure_get_int(structure, "width", &width);
    gst_structure_get_int(structure, "height", &height);

    GstMapInfo map;
    sensor_msgs::msg::Image::SharedPtr msg;
    if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        // Convert to OpenCV Mat
        const auto frame = cv::Mat(cv::Size(width, height), CV_8UC3, map.data, cv::Mat::AUTO_STEP);

        // Resize the frame if necessary
        // if (width != config->width || height != config->height) {
        //     const auto resized = cv::Mat(cv::Size(config->width, config->height), CV_8UC3);
        //     cv::resize(frame, resized, resized.size());
        //     frame = resized;
        // }

        // Convert to ROS Image message
        msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", frame).toImageMsg();

        gst_buffer_unmap(buffer, &map);
    }

    gst_sample_unref(sample);

    return msg;
}

GstElement* RTSPCameraNode::create_source_pipeline(const RTSPStreamConfig& stream_config) {
    const auto v4l2_pipeline_template = this->get_parameter("templates.v4l2").as_string();
    const auto appsrc_pipeline_template = this->get_parameter("templates.appsrc.pipeline").as_string();
    const auto test_pipeline_template = this->get_parameter("templates.test").as_string();
    const auto intervideosink_pipeline_template = this->get_parameter("templates.intervideosink").as_string();

    const auto width = stream_config.width;
    const auto height = stream_config.height;
    const auto fps = stream_config.fps;

    string src;

    if (stream_config.type == CAMERA) {
        src = fmt::format("{} device={}", v4l2_pipeline_template, stream_config.source);
    } else if (stream_config.type == TOPIC) {
        src = fmt::format("{} name={}", appsrc_pipeline_template, appsrc_name);
        // /* Keep record of number of clients connected to each topic.
        //  * so we know to stop subscribing when no-one is connected. */
        // num_of_clients[stream_config.mountpoint] = 0;
        // this->appsrcs[stream_config.mountpoint] = nullptr;
        // appsrc = &this->appsrcs[stream_config.mountpoint];
    } else if (stream_config.type == TEST) {
        src = test_pipeline_template;
    }

    auto convert = fmt::format("videoconvert ! videoscale ! videorate ! video/x-raw,framerate={}/1", fps);

    if (width > 0)
        convert = fmt::format("{},width={}", convert, stream_config.width);
    if (height > 0)
        convert = fmt::format("{},height={}", convert, stream_config.height);

    const auto source_pipeline_str = fmt::format("{} ! {} ! {} channel={}", src, convert, intervideosink_pipeline_template, stream_config.name);

    logger.info("Source pipeline: {}", source_pipeline_str);

    GError* error = nullptr;

    const auto source_pipeline = gst_parse_launch(source_pipeline_str.c_str(), &error);

    if (error) {
        gst_object_unref(source_pipeline);
        g_clear_error(&error);
        logger.fatal("Error parsing pipeline ({}): {}", error->code, error->message);
        throw runtime_error("Error parsing pipeline");
    }

    return source_pipeline;
}

string RTSPCameraNode::create_rtsp_pipeline(const RTSPStreamConfig& stream_config) const {
    const auto intervideosrc_pipeline_template = this->get_parameter("templates.intervideosrc").as_string();

    const auto bitrate = stream_config.bitrate;
    const auto encoder = stream_config.encoder.encoder;
    const auto encoder_pipeline = stream_config.encoder.pipeline;
    const auto bitrate_param = stream_config.encoder.bitrate_param;
    const auto rtp_pay = stream_config.encoder.rtp_pay;

    const auto full_encoder_pipeline = fmt::format("{} {}={} ! {}", encoder, bitrate_param, bitrate, encoder_pipeline);

    // v4l2src do-timestamp=1 device=/dev/video0 ! videoconvert ! videoscale ! videorate ! video/x-raw,framerate={}/1,width=1920,height=1080 ! intervideosink channel=camera0
    // intervideosrc channel=camera0 ! x264enc tune=zerolatency bitrate=8000 ! video/x-h264, profile=baseline ! rtph264pay name=pay0 pt=96

    const auto rtsp_pipeline_str = fmt::format("{} channel={} ! {} ! {}", intervideosrc_pipeline_template, stream_config.name, full_encoder_pipeline, rtp_pay);

    logger.info("RTSP pipeline: {}", rtsp_pipeline_str);

    return rtsp_pipeline_str;
    // GError* error = nullptr;
    //
    // const auto rtsp_pipeline = gst_parse_launch(rtsp_pipeline_str.c_str(), &error);
    //
    // if (error) {
    //     gst_object_unref(rtsp_pipeline);
    //     g_clear_error(&error);
    //     logger.fatal("Error parsing pipeline ({}): {}", error->code, error->message);
    //     throw runtime_error("Error parsing pipeline");
    // }
    //
    // gst_pipeline_set_auto_flush_bus(GST_PIPELINE(rtsp_pipeline), FALSE);
    //
    // return rtsp_pipeline;
}

GstElement* RTSPCameraNode::create_topic_pipeline(const RTSPStreamConfig& stream_config) {
    const auto intervideosrc_pipeline_template = this->get_parameter("templates.intervideosrc").as_string();
    const auto appsink_pipeline_template = this->get_parameter("templates.appsink.pipeline").as_string();

    const auto format = stream_config.publishing->format;
    const auto fps = stream_config.publishing->fps;
    if (format != "RGB") {
        logger.fatal("format must be 'RGB', but was '{}'", format);
        throw runtime_error("Bad format");
    }


    auto convert_appsink = fmt::format("videoconvert ! videoscale ! video/x-raw,format={},framerate={}/1,width={},height={}", format, fps,
                                       stream_config.publishing->width, stream_config.publishing->height);
    const auto topic_pipeline_str = fmt::format("{} channel={} ! {} ! {} name={}", intervideosrc_pipeline_template, stream_config.name, convert_appsink,
                                                appsink_pipeline_template, appsink_name);

    logger.info("Topic pipeline: {}", topic_pipeline_str);


    GError* error = nullptr;

    const auto rtsp_pipeline = gst_parse_launch(topic_pipeline_str.c_str(), &error);

    if (error) {
        gst_object_unref(rtsp_pipeline);
        g_clear_error(&error);
        logger.fatal("Error parsing pipeline ({}): {}", error->code, error->message);
        throw runtime_error("Error parsing pipeline");
    }

    // gst_pipeline_set_auto_flush_bus(GST_PIPELINE(rtsp_pipeline), FALSE); // what is this for? is this needed?

    return rtsp_pipeline;
}


GstRTSPServer* RTSPCameraNode::rtsp_server_create(const int port) {
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

void RTSPCameraNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg, GstAppSrc* appsrc) const {
    // Set caps from message
    const auto caps = gst_caps_new_from_ros_image(msg);
    gst_app_src_set_caps(appsrc, caps);

    const auto buf = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
    gst_buffer_fill(buf, 0, msg->data.data(), msg->data.size());
    GST_BUFFER_FLAG_SET(buf, GST_BUFFER_FLAG_LIVE);

    gst_app_src_push_buffer(appsrc, buf);
}

void RTSPCameraNode::client_options(GstRTSPClient*, const GstRTSPContext* state, RTSPCameraNode* node) {
    if (state->uri) {
        node->url_connected(state->uri->abspath);
    }
}

void RTSPCameraNode::client_teardown(GstRTSPClient*, const GstRTSPContext* state, RTSPCameraNode* node) {
    if (state->uri) {
        node->url_disconnected(state->uri->abspath);
    }
}

void RTSPCameraNode::new_client(GstRTSPServer*, GstRTSPClient* client, RTSPCameraNode* node) {
    const auto client_connection = gst_rtsp_client_get_connection(client);
    const auto client_url = gst_rtsp_url_get_request_uri(gst_rtsp_connection_get_url(client_connection));
    const auto client_ip = gst_rtsp_connection_get_ip(client_connection);
    node->logger.info("New RTSP client connecting from {} ({})", client_url, client_ip);
    g_signal_connect(client, "options-request", G_CALLBACK(client_options), node);
    g_signal_connect(client, "teardown-request", G_CALLBACK(client_teardown), node);
}

gboolean RTSPCameraNode::session_cleanup(const RTSPCameraNode* node, gboolean) {
    GstRTSPServer* server = node->rtsp_server;

    auto* pool = gst_rtsp_server_get_session_pool(server);
    const auto num = gst_rtsp_session_pool_cleanup(pool);
    g_object_unref(pool);

    if (num > 0) {
        node->logger.info("Cleaning up session {}", num);
    }

    return TRUE;
}

void RTSPCameraNode::url_connected(const string& url) {
    logger.info("Client connected: {}", url);

    // for (const auto& stream : streams) {
    //     if (stream.type == TOPIC && url == stream.mountpoint) {
    //         if (num_of_clients[url] == 0) {
    //             auto imageCallback = [this, mountpoint = stream.mountpoint](const sensor_msgs::msg::Image::ConstSharedPtr& image) {
    //                 this->imageCallback(image, TODO);
    //             };
    //             const auto subscription = this->create_subscription<sensor_msgs::msg::Image>(stream.source, 1, imageCallback);
    //             // Subscribe to the ROS topic
    //             subscribers[url] = subscription;
    //         }
    //         num_of_clients[url]++;
    //     }
    // }
}

void RTSPCameraNode::url_disconnected(const string& url) {
    logger.info("Client disconnected: {}", url);

    // for (const auto& stream : streams) {
    //     if (url == stream.mountpoint) {
    //         if (num_of_clients[url] > 0) num_of_clients[url]--;
    //         if (num_of_clients[url] == 0) {
    //             // No-one else is connected, stop the subscription by letting it go out of scope
    //             subscribers[url] = nullptr;
    //             appsrcs[url] = nullptr;
    //         }
    //     }
    // }
}

GstCaps* RTSPCameraNode::gst_caps_new_from_ros_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg) const {
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
        logger.info("GST: big endian image format is not supported");
        return nullptr;
    }

    const auto format = known_formats.find(msg->encoding);
    if (format == known_formats.end()) {
        logger.info("GST: image format '%s' unknown", msg->encoding.c_str());
        return nullptr;
    }

    return gst_caps_new_simple(
        "video/x-raw",
        "format", G_TYPE_STRING, format->second.c_str(),
        "width", G_TYPE_INT, msg->width,
        "height", G_TYPE_INT, msg->height,
        "framerate", GST_TYPE_FRACTION, 10, 1,
        nullptr
    );
}

void RTSPCameraNode::rtsp_server_add_url(const string& url, const string& pipeline, RTSPStreamPipeline stream_pipeline,
                                         const optional<ROSImageTopicConfig>& topic_config) {
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
    struct MediaConfigureData {
        RTSPCameraNode* node;
        const string url;
        RTSPStreamPipeline* stream_pipeline;
        const optional<ROSImageTopicConfig>& topic_config;
    };

    const auto media_configure = +[](GstRTSPMediaFactory*, GstRTSPMedia* media, const MediaConfigureData* data) {
        data->node->media_configure(data->url, media, data->stream_pipeline, data->topic_config);
    };

    auto data = MediaConfigureData{
        .node            = this,
        .url             = url,
        .stream_pipeline = &stream_pipeline,
        .topic_config    = topic_config,
    };

    g_signal_connect(factory, "media-configure", reinterpret_cast<GCallback>(media_configure), &data);

    gst_rtsp_media_factory_set_shared(factory, true);

    gst_rtsp_media_factory_set_enable_rtcp(factory, true);

    /* attach the factory to the url */
    gst_rtsp_mount_points_add_factory(mounts, url.c_str(), factory);

    /* don't need the ref to the mounts anymore */
    g_object_unref(mounts);
}

/**
 * called when a new media pipeline is constructed.
 * We can query the pipeline and configure our appsrc.
 */
void RTSPCameraNode::media_configure(const string& /*url*/, GstRTSPMedia* media, RTSPStreamPipeline* /*stream_pipeline*/,
                                     const optional<ROSImageTopicConfig>& /*topic_config*/) {
    const auto pipeline = gst_rtsp_media_get_element(media);

    // if (topic_config) {
    //     *appsink = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline), appsink_name.c_str()));
    //     // // TODO 2025-10-18: This needs to be moved to rtsp_server_add_url
    //     // publishers[url] = ;
    // }
    //
    // if (appsrc) {
    //     // *appsrc = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(pipeline), appsrc_name.c_str()));
    //
    //     /* this instructs appsrc that we will be dealing with timed buffer */
    //     // gst_util_set_object_arg(G_OBJECT(*appsrc), "format", "time");
    // } else {
    logger.info("Initializing media factory...");

    const auto n_streams = gst_rtsp_media_n_streams(media);

    // for (auto i = 0u; i < n_streams; i++) {
    //     const auto stream = gst_rtsp_media_get_stream(media, i);
    //
    //     /* make a new address pool */
    //     const auto pool = gst_rtsp_address_pool_new();
    //
    //     gchar* min = g_strdup_printf("224.3.0.%d", 2 * i + 1);
    //     gchar* max = g_strdup_printf("224.3.0.%d", 2 * i + 2);
    //     gst_rtsp_address_pool_add_range(pool, min, max,
    //                                     5000 + 10 * i, 5010 + 10 * i, 1);
    //     g_free(min);
    //     g_free(max);
    //
    //     gst_rtsp_stream_set_address_pool(stream, pool);
    //     g_object_unref(pool);
    // }

    logger.info("Initialized media factory with {} streams", n_streams);
    // }

    gst_object_unref(pipeline);
}

void RTSPCameraNode::process_frames() {
    while (ok()) {
        for (const auto& stream : streams) {
            if (!(stream.publishing.has_value() || publishers.contains(stream.mountpoint)))
                continue;

            const auto config = stream.publishing;
            const auto appsink = appsinks[stream.mountpoint];
            const auto publisher = publishers[stream.mountpoint];

            if (const auto sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink), GST_SECOND / 10)) {
                const auto buffer = gst_sample_get_buffer(sample);
                const auto caps = gst_sample_get_caps(sample);
                const auto structure = gst_caps_get_structure(caps, 0);

                // Get width and height
                int width, height;

                gst_structure_get_int(structure, "width", &width);
                gst_structure_get_int(structure, "height", &height);

                GstMapInfo map;
                if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
                    // Convert to OpenCV Mat
                    const auto frame = cv::Mat(cv::Size(width, height), CV_8UC3, map.data, cv::Mat::AUTO_STEP);

                    // Resize the frame if necessary
                    if (width != config->width || height != config->height) {
                        cv::resize(frame, frame, cv::Size(boost::numeric_cast<int>(config->width), boost::numeric_cast<int>(config->height)));
                    }

                    // Convert to ROS Image message
                    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", frame).toImageMsg();
                    msg->header.stamp = this->get_clock()->now();
                    msg->header.frame_id = stream.name;

                    // Publish the image
                    publisher->publish(*msg);

                    gst_buffer_unmap(buffer, &map);
                }

                gst_sample_unref(sample);
            } else {
                // No sample available, sleep briefly
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            // Check for messages on the bus
            // while (gst_bus_have_pending(bus_)) {
            //     GstMessage* msg = gst_bus_pop(bus_);
            //     if (msg != NULL) {
            //         GError* err;
            //         gchar* debug_info;
            //
            //         switch (GST_MESSAGE_TYPE(msg)) {
            //             case GST_MESSAGE_ERROR:
            //                 gst_message_parse_error(msg, &err, &debug_info);
            //                 logger.error("Error received from element {}: {}",
            //                              GST_OBJECT_NAME(msg->src), err->message);
            //                 logger.error("Debugging information: %s", debug_info ? debug_info : "none");
            //                 g_clear_error(&err);
            //                 g_free(debug_info);
            //                 quit = TRUE;
            //                 break;
            //             case GST_MESSAGE_EOS:
            //                 logger.info("End-Of-Stream reached.");
            //                 quit = TRUE;
            //                 break;
            //             default:
            //                 // For other messages, do nothing
            //                 break;
            //         }
            //         gst_message_unref(msg);
            //     }
            // }
        }
    }

    // logger.info("Stopping pipeline.");
    // gst_element_set_state(pipeline_, GST_STATE_NULL);
}

void* RTSPCameraNode::mainloop(RTSPCameraNode* node) {
    const auto loop = g_main_loop_new(nullptr, false);
    node->main_loop = loop;

    g_main_loop_run(loop);

    g_main_loop_unref(loop);
    return nullptr;
}


int main(const int argc, const char* const argv[]) {
    init(argc, argv);
    spin(make_shared<RTSPCameraNode>());
    shutdown();

    return 0;
}
