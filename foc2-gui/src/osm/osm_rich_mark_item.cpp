#include "foc2-gui/osm/osm_rich_mark_item.hpp"

namespace ImOsm::Rich {
    MarkItem::MarkItem() {
        osm_coords = OsmCoords(coords);
        rx.resize(static_cast<int>(360.0 / dphi) + 1);
        ry.resize(static_cast<int>(360.0 / dphi) + 1);
    }

    MarkItem::MarkItem(const GeographicLib::GeoCoords& coords, const std::string& text)
        : coords(coords), text(text) {
        osm_coords = OsmCoords(coords);
        rx.resize(static_cast<int>(360.0 / dphi) + 1);
        ry.resize(static_cast<int>(360.0 / dphi) + 1);
    }

    MarkItem::~MarkItem() = default;

    bool MarkItem::inBounds(const Eigen::AlignedBox2d geo_box) const {
        return geo_box.intersects(bounds);
    }

    void MarkItem::paint() {
        if (style.markerEnabled) {
            ImGui::PushID(&osm_coords);

            const auto spec = ImPlotSpec(
                ImPlotProp_Marker, style.markerType,
                ImPlotProp_MarkerSize, style.markerSize,
                ImPlotProp_MarkerFillColor, style.markerFill,
                ImPlotProp_LineWeight, style.markerWeight,
                ImPlotProp_LineColor, style.markerOutline
            );

            ImPlot::PlotScatter("##", &osm_coords.x, &osm_coords.y, 1, spec);

            ImGui::PopID();
        }

        if (style.textEnabled) {
            ImGui::PushStyleColor(ImGuiCol_Text, style.markerFill);
            ImGui::PushID(text.c_str());
            ImPlot::PlotText(text.c_str(), osm_coords.x, osm_coords.y, ImVec2(0.f, style.markerSize + ImGui::GetFontSize()));
            ImGui::PopID();
            ImGui::PopStyleColor();
        }

        if (style.radiusEnabled) {
            ImGui::PushID(rx.data());

            const auto spec = ImPlotSpec(
                ImPlotProp_LineColor, style.markerFill,
                ImPlotProp_LineWeight, style.radiusWeight
            );

            ImPlot::PlotLine("##", rx.data(), ry.data(), rx.size(), spec);

            ImGui::PopID();
        }
    }

    void MarkItem::updateRadiusPoints() {
        /*
            double lat{}, lon{};
            double x{}, y{};
            double phi{};
            for (int i{}; i != _rx.size(); ++i, phi += _dphi) {
              destination(lat, lon, _lat, _lon, _r, phi);
              _rx[i] = lon2x(lon);
              _ry[i] = lat2y(lat);
            } */

        // TODO 2026-05-15 (Will Free): this can probably just be done entirely using geographiclib? idk.

        const auto dest = destination(coords, radius, 0);
        const auto x = lon2x(dest.Longitude());
        const auto y = lat2y(dest.Latitude());
        const auto r = sqrt(std::pow(x - osm_coords.x, 2.0) + std::pow(y - osm_coords.y, 2.0));
        const auto dphi = this->dphi * std::numbers::pi / 180.0;
        const auto x0 = x;
        const auto y0 = y + r;

        double phi = 0.0;
        for (auto i = 0u; i != rx.size(); ++i, phi += dphi) {
            rx[i] = x0 + r * cos(phi);
            ry[i] = y0 + r * sin(phi);
        }
    }

    void MarkItem::updateRadiusBounds() {
        for (constexpr double bearings[] = {45.0, 135.0, 225.0, 315.0}; const double b : bearings) {
            double lat;
            double lon;
            GeographicLib::Geodesic::WGS84().Direct(coords.Latitude(), coords.Longitude(), b, radius, lat, lon);

            // Expand the bounding box with (lon, lat)
            bounds.extend(Eigen::Vector2d(lon, lat));
        }
    }
}
