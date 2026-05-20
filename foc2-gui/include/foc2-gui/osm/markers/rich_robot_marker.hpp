#pragma once

#include <ros2_fmt_logger/logger.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "foc2-gui/im_application.hpp"
#include "foc2-gui/osm/osm_irich_item.hpp"

class RichRobotMarker : public ImOsm::Rich::IRichItem {
    using NavSatFix = sensor_msgs::msg::NavSatFix;

public:
    explicit RichRobotMarker(ImApplication& application)
        : application(application),
          logger(application.get_logger()) {}

    ~RichRobotMarker() override = default;

    void onInit();

    void onShutdown() const;

    bool inBounds(Eigen::AlignedBox2d geo_box) const override;

    void setEnabled(bool /*enabled*/) override {}

    bool isEnabled() const override {
        return true;
    }

    void paint() override;

protected:
    ImApplication& application;
    ros2_fmt_logger::Logger logger;

    rclcpp::Subscription<NavSatFix>::SharedPtr fix_subscription;
    NavSatFix::SharedPtr fix;
};
