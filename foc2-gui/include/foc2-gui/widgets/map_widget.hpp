#pragma once

#include "foc2-gui/im_application.hpp"
#include "foc2-gui/overlayable.hpp"
#include "foc2-gui/widget.hpp"
#include "foc2-gui/osm/markers/rich_gps_marker.hpp"
#include "foc2-gui/osm/markers/rich_robot_marker.hpp"

namespace ImOsm::Rich {
    class RichMapPlot;
}

class MapWidget : public UiWidget {
public:
    RCLCPP_SMART_PTR_ALIASES_ONLY(MapWidget)

    explicit MapWidget(ImApplication& application);

    void onInit() override;
    void onShutdown() override;

protected:
    void draw() override;

private:
    std::shared_ptr<ImOsm::Rich::RichMapPlot> map_plot;
    std::shared_ptr<RichRobotMarker> robot_marker;
    std::shared_ptr<RichGPSMarker> gps_marker;
};
