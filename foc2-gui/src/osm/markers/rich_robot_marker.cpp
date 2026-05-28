#include "foc2-gui/osm/markers/rich_robot_marker.hpp"

#include <imgui.h>
#include <implot.h>

#include "foc2-gui/osm/osm_coords.hpp"
#include "foc2-gui/util/imgui_util.hpp"

void RichRobotMarker::onInit() {
    using namespace std::chrono_literals;

    fix_subscription = application.create_subscription<NavSatFix>(
        "/gps/fix",
        10,
        // ReSharper disable once CppPassValueParameterByConstReference
        [&](const NavSatFix::SharedPtr msg) {
            fix = msg;
        }
    );
}

void RichRobotMarker::onShutdown() const {}

bool RichRobotMarker::inBounds(const Eigen::AlignedBox2d /*geo_box*/) const {
    // return geo_box.contains(Eigen::Vector2d(robot_transform.translation.x, robot_transform.translation.z));
    return true;
}

void RichRobotMarker::paint() {
    if (!fix)
        return;

    ImGui::PushID("Robot Marker");

    const auto osm_coords = ImOsm::OsmCoords(ImOsm::lon2x(fix->longitude), ImOsm::lat2y(fix->latitude));

    using namespace std::chrono_literals;
    logger.warn_throttle(5s, "robot at {}, {} (osm {}, {})", fix->longitude, fix->latitude, osm_coords.x, osm_coords.y);

    const auto spec = ImPlotSpec(
        ImPlotProp_Marker, ImPlotMarker_Circle,
        ImPlotProp_MarkerSize, 4.0f,
        ImPlotProp_MarkerFillColor, ImVec4(255, 0, 0, 200),
        ImPlotProp_LineWeight, 1.5f,
        ImPlotProp_LineColor, ImVec4(255, 0, 0, 200)
    );

    ImPlot::PlotScatter("##", &osm_coords.x, &osm_coords.y, 1, spec);

    ImGui::PopID();
}
