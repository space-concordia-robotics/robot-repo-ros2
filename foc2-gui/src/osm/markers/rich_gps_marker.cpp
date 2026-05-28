#include "foc2-gui/osm/markers/rich_gps_marker.hpp"

#include <imgui.h>
#include <implot.h>

#include "foc2-gui/osm/osm_coords.hpp"
#include "foc2-gui/util/imgui_util.hpp"

void RichGPSMarker::onInit() {
    using namespace std::chrono_literals;

    point_subscription = application.create_subscription<GeoPoint>(
        "/coords",
        10,
        // ReSharper disable once CppPassValueParameterByConstReference
        [&](const GeoPoint::SharedPtr msg) {
            point = msg;
        }
    );
}

void RichGPSMarker::onShutdown() const {}

bool RichGPSMarker::inBounds(const Eigen::AlignedBox2d /*geo_box*/) const {
    // return geo_box.contains(Eigen::Vector2d(robot_transform.translation.x, robot_transform.translation.z));
    return true;
}

void RichGPSMarker::paint() {
    if (!point)
        return;

    ImGui::PushID("GPS Marker");

    const auto osm_coords = ImOsm::OsmCoords(ImOsm::lon2x(point->longitude), ImOsm::lat2y(point->latitude));

    using namespace std::chrono_literals;

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
