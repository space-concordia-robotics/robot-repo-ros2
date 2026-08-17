#pragma once

#include <ros2_fmt_logger/logger.hpp>
#include <geographic_msgs/msg/geo_point.hpp>

#include "foc2-gui/im_application.hpp"
#include "foc2-gui/osm/osm_irich_item.hpp"

class RichGPSMarker : public ImOsm::Rich::IRichItem {
    using GeoPoint = geographic_msgs::msg::GeoPoint;

public:
    explicit RichGPSMarker(ImApplication& application)
        : application(application),
          logger(application.get_logger()) {}

    ~RichGPSMarker() override = default;

    void onInit();

    void onShutdown();

    bool inBounds(Eigen::AlignedBox2d geo_box) const override;

    void setEnabled(bool /*enabled*/) override {}

    bool isEnabled() const override {
        return true;
    }

    void paint() override;

protected:
    ImApplication& application;
    ros2_fmt_logger::Logger logger;

    rclcpp::Subscription<GeoPoint>::SharedPtr point_subscription;
    GeoPoint::SharedPtr point;
};
