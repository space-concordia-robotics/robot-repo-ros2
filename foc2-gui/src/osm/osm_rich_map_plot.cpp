#include "foc2-gui/osm/osm_rich_map_plot.hpp"

#include <algorithm>

#include "foc2-gui/osm/osm_irich_item.hpp"

namespace ImOsm::Rich {
    RichMapPlot::RichMapPlot() {}

    RichMapPlot::RichMapPlot(const std::shared_ptr<ITileLoader>& loader)
        : MapPlot(loader) {}

    RichMapPlot::~RichMapPlot() = default;

    void RichMapPlot::paintOverMap() {
        MapPlot::paintOverMap();

        std::erase_if(
            items,
            [](auto item) {
                return item.expired();
            }
        );

        std::ranges::for_each(items, [this](auto ptr) {
            if (auto item = ptr.lock(); item->isEnabled() &&
                item->inBounds(geoBox()))
                item->paint();
        });
    }

    void RichMapPlot::loadState(/*const mINI::INIStructure& ini*/) {
        // if (ini.has("map_plot")) {
        //     if (ini.get("map_plot").has("min_lat") &&
        //         ini.get("map_plot").has("max_lat") &&
        //         ini.get("map_plot").has("min_lon") &&
        //         ini.get("map_plot").has("max_lon")) {
        //         setBoundsGeo(std::stof(ini.get("map_plot").get("min_lat")),
        //                      std::stof(ini.get("map_plot").get("max_lat")),
        //                      std::stof(ini.get("map_plot").get("min_lon")),
        //                      std::stof(ini.get("map_plot").get("max_lon")));
        //     }
        // }
    }

    void RichMapPlot::saveState(/*mINI::INIStructure& ini*/) const {
        // float minLat{}, maxLat{}, minLon{}, maxLon{};
        // getBoundsGeo(minLat, maxLat, minLon, maxLon);
        // ini["map_plot"].set("min_lat", std::to_string(minLat));
        // ini["map_plot"].set("max_lat", std::to_string(maxLat));
        // ini["map_plot"].set("min_lon", std::to_string(minLon));
        // ini["map_plot"].set("max_lon", std::to_string(maxLon));
    }

    void RichMapPlot::addItem(const std::weak_ptr<IRichItem>& item) {
        items.push_back(item);
    }
}
