#include "foc2-gui/osm/osm_tile_source_url_impl.hpp"

#include <sstream>
#include <utility>
#include <fmt/format.h>

namespace ImOsm {
    TileSourceUrlCustom::TileSourceUrlCustom(const int request_limit, const bool preload, std::string url_template)
        : TileSourceUrl(request_limit, preload),
          url_template(std::move(url_template)) {}

    std::string TileSourceUrlCustom::makeUrl(const int z, const int x, const int y) {
        std::string url = url_template;
        if (const auto pos_z = url.find("$Z$"); pos_z < url.size()) {
            url.replace(pos_z, 3, std::to_string(z));
            if (const auto pos_x = url.find("$X$"); pos_x < url.size()) {
                url.replace(pos_x, 3, std::to_string(x));
                if (const auto pos_y = url.find("$Y$"); pos_y < url.size()) {
                    url.replace(pos_y, 3, std::to_string(y));
                }
            }
        }
        return url;
    }

    // -----------------------------------------------------------------------------

    TileSourceUrlOsm::TileSourceUrlOsm(const int request_limit, const bool preload)
        : TileSourceUrl(request_limit, preload) {}

    std::string TileSourceUrlOsm::makeUrl(const int z, const int x, const int y) {
        return fmt::format("{}/{}/{}/{}.png", "https://a.tile.openstreetmap.org", z, x, y);
    }

    // -----------------------------------------------------------------------------

    TileSourceUrlArcImagery::TileSourceUrlArcImagery(const int request_limit, const bool preload) : TileSourceUrl(request_limit, preload) {}

    std::string TileSourceUrlArcImagery::makeUrl(const int z, const int x, const int y) {
        static constexpr auto ARC_URL = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile";
        return fmt::format("{}/{}/{}/{}", ARC_URL, z, y, x);
    }
}
