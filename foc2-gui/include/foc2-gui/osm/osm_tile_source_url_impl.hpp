#pragma once

#include "foc2-gui/osm/osm_tile_source_url.hpp"

// https://www.arcgis.com/apps/mapviewer/index.html
// See Basemap

namespace ImOsm {
    class TileSourceUrlCustom : public TileSourceUrl {
    public:
        TileSourceUrlCustom(int request_limit, bool preload,
                            std::string url_template);

    protected:
        std::string makeUrl(int z, int x, int y) override;

    private:
        const std::string url_template;
    };

    // -----------------------------------------------------------------------------

    class TileSourceUrlOsm : public TileSourceUrl {
    public:
        TileSourceUrlOsm(int request_limit, bool preload);

    protected:
        std::string makeUrl(int z, int x, int y) override;

    public:
        static constexpr auto URL_TEMPLATE = "https://a.tile.openstreetmap.org/$Z$/$X$/$Y$.png";
    };

    // -----------------------------------------------------------------------------

    class TileSourceUrlArcImagery : public TileSourceUrl {
    public:
        TileSourceUrlArcImagery(int request_limit, bool preload);

    protected:
        std::string makeUrl(int z, int x, int y) override;

    public:
        static constexpr auto URL_TEMPLATE = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/$Z$/$Y$/$X$";
    };

    using TileSourceUrlArc = TileSourceUrlArcImagery;
}
