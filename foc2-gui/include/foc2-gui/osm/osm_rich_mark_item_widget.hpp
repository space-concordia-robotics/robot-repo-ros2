#pragma once

#include <memory>

#include "foc2-gui/osm/osm_coords.hpp"

namespace ImOsm::Rich {
    class MarkItem;

    class MarkItemWidget {
    public:
        MarkItemWidget(const std::shared_ptr<MarkItem>& item, const GeographicLib::GeoCoords& picked_coords);
        ~MarkItemWidget();

        void paint() const;
        void apply() const;

    private:
        void paint_pickedBtn() const;
        void paint_markerCombo() const;

        std::shared_ptr<MarkItem> item;
        GeographicLib::GeoCoords picked_coords;

        struct Ui;
        std::unique_ptr<Ui> ui;
    };
}
