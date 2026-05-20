#pragma once

#include <string>
#include <vector>

#include "foc2-gui/osm/osm_coords.hpp"
#include "foc2-gui/osm/osm_irich_item.hpp"

namespace ImOsm::Rich {
    class MarkItem : public IRichItem {
    public:
        struct Style {
            bool textEnabled = true;
            bool markerEnabled = true;
            bool radiusEnabled = true;
            ImPlotMarker markerType = ImPlotMarker_Circle;
            float markerSize = 10.0;
            float markerWeight = 0.0;
            ImVec4 markerFill = ImVec4(1.f, 0.f, 0.f, 1.f);
            ImVec4 markerOutline = ImVec4();
            float radiusWeight = 2.0;
        };

        MarkItem();
        MarkItem(const GeographicLib::GeoCoords& coords, const std::string& text);
        ~MarkItem() override;

        bool inBounds(Eigen::AlignedBox2d geo_box) const override;

        void setEnabled(const bool enabled) override {
            this->enabled = enabled;
        }

        bool isEnabled() const override {
            return enabled;
        }

        void paint() override;

        void setCoords(const GeographicLib::GeoCoords& coords) {
            this->coords = coords;
            osm_coords = OsmCoords(coords);
            if (radius > 0.0) {
                updateRadiusPoints();
                updateRadiusBounds();
            }
        }

        const GeographicLib::GeoCoords& getCoords() const {
            return coords;
        }

        const OsmCoords& getOsmCoords() const {
            return osm_coords;
        }

        void setRadius(const float radius) {
            this->radius = radius;
            updateRadiusPoints();
            updateRadiusBounds();
        }

        float getRadius() const {
            return radius;
        }

        void setText(const std::string& text) {
            this->text = text;
        }

        const std::string& getText() const {
            return text;
        }

        void setStyle(const Style& style) {
            this->style = style;
        }

        const Style& getStyle() const {
            return style;
        }

        Style& getStyle() {
            return style;
        }

    private:
        void updateRadiusPoints();
        void updateRadiusBounds();

        bool enabled = true;
        GeographicLib::GeoCoords coords;
        OsmCoords osm_coords = OsmCoords(0, 0);
        std::string text;
        Eigen::AlignedBox2d bounds;
        float radius = 0;
        std::vector<float> rx, ry;
        float dphi = 1.0;
        Style style;
    };
}
