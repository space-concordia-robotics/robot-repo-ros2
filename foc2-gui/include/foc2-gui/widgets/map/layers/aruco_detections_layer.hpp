#pragma once

#include <memory>
#include <mutex>
#include <vector>
#include <mln/style/style.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <ros_aruco_opencv_msgs/msg/aruco_detections.hpp>

#include "layer_manager.hpp"

class ArucoMarkerLayerManager : public MaplibreLayerManager {
    struct DetectedArucoMarker {
        uint32_t id = 0;
        geographic_msgs::msg::GeoPoint position;
    };

    using ArucoDetections = ros_aruco_opencv_msgs::msg::ArucoDetections;

public:
    explicit ArucoMarkerLayerManager(ImApplication& application, MapWidget& widget);

    ~ArucoMarkerLayerManager() override;
    ArucoMarkerLayerManager(ArucoMarkerLayerManager&& other) noexcept;
    ArucoMarkerLayerManager(const ArucoMarkerLayerManager& other) = delete;

    ArucoMarkerLayerManager& operator=(const ArucoMarkerLayerManager& other) = delete;
    ArucoMarkerLayerManager& operator=(ArucoMarkerLayerManager&& other) = delete;

    void install(mln::style::Style& style) override;

    void uninstall(mln::style::Style& style) override;

    void update() override;

    void enable() override;

    void disable() override;

private:
    rclcpp::Subscription<ArucoDetections>::SharedPtr subscription;

    std::vector<DetectedArucoMarker> detected_markers;
    bool has_update = false;
    std::mutex mutex;

    mln::style::GeoJSONSource* marker_source = nullptr;
    mln::style::SymbolLayer* marker_layer = nullptr;

    void onArucoDetections(const ArucoDetections::ConstSharedPtr& msg);
};
