#pragma once

#include <map>
#include <string>
#include <camera_info_manager/camera_info_manager.hpp>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/rtsp-server/rtsp-server-object.h>
#include <rclcpp/rclcpp.hpp>
#include <ros2_fmt_logger/ros2_fmt_logger.hpp>

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
    string rtp_pay;
};

struct ROSImageTopicConfig {
    string topic_name;
    bool uncompressed;
    bool compressed;
    // bool ffmpeg;
    long int fps;
    long int width;
    long int height;
    string format;
    bool sensor_data_qos;
    // string compression;
};

struct RTSPStreamConfig {
    string name;
    RTSPCameraType type;
    string source;
    string mountpoint;
    RTSPStreamEncoder encoder;
    long int bitrate;
    long int fps;
    long int width;
    long int height;
    optional<ROSImageTopicConfig> publishing;
};

struct RTSPStreamPipeline {
    GstElement* source_pipeline;
    // GstElement* rtsp_pipeline; // can be null pointer
    GstElement* topic_pipeline;
    GstAppSrc* appsrc;
    GstAppSink* appsink;

    Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;

    Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;

    // GstBus* bus;
    // GMainLoop* main_loop;
};

class RTSPCameraNode final : public Node {
public:
    RTSPCameraNode();

    ~RTSPCameraNode() override;
    void create_pipeline(const RTSPStreamConfig& stream_config);
    void start_pipeline(const RTSPStreamConfig& stream_config, RTSPStreamPipeline& stream_pipeline);
    static sensor_msgs::msg::Image::SharedPtr img_msg_from_gst_sample(GstSample* sample, const ROSImageTopicConfig* config);
    GstElement* create_source_pipeline(const RTSPStreamConfig& stream_config);
    string create_rtsp_pipeline(const RTSPStreamConfig& stream_config) const;
    GstElement* create_topic_pipeline(const RTSPStreamConfig& stream_config);
    GstRTSPServer* rtsp_server_create(int port);
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg, GstAppSrc* appsrc) const;
    static void client_options(GstRTSPClient*, const GstRTSPContext* state, RTSPCameraNode* node);
    static void client_teardown(GstRTSPClient*, const GstRTSPContext* state, RTSPCameraNode* node);
    static void new_client(GstRTSPServer*, GstRTSPClient* client, RTSPCameraNode* node);
    static gboolean session_cleanup(const RTSPCameraNode* node, gboolean);
    void url_connected(const string& url);
    void url_disconnected(const string& url);
    GstCaps* gst_caps_new_from_ros_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg) const;
    void rtsp_server_add_url(const string& url, const string& pipeline, RTSPStreamPipeline stream_pipeline, const optional<ROSImageTopicConfig>& topic_config);
    void media_configure(const string& url, GstRTSPMedia* media, RTSPStreamPipeline* stream_pipeline, const optional<ROSImageTopicConfig>& topic_config);
    void process_frames();
    static void* mainloop(RTSPCameraNode* node);

private:
    vector<thread> pipeline_threads = {};

    ros2_fmt_logger::Logger logger;
    camera_info_manager::CameraInfoManager camera_info_manager;

    string appsink_name;
    string appsrc_name;
    vector<RTSPStreamConfig> streams;

    // ROS publisher
    Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    GstRTSPServer* rtsp_server;
    // GStreamer stuff
    map<string, Subscription<sensor_msgs::msg::Image>::SharedPtr> subscribers;
    map<string, Publisher<sensor_msgs::msg::Image>::SharedPtr> publishers;
    map<string, GstAppSrc*> appsrcs;
    map<string, GstAppSink*> appsinks;
    map<string, int> num_of_clients;

    thread gstreamer_thread;
    GMainLoop* main_loop;

    long int time_offset;
};
