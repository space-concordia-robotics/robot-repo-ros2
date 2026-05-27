#pragma once

#include <memory>

#include "foc2-gui/osm/osm_tile_loader.hpp"
#include "foc2-gui/osm/osm_tile_source_fs.hpp"
#include "foc2-gui/osm/osm_tile_source_url_impl.hpp"

namespace ImOsm {
    inline constexpr int URL_REQUEST_LIMIT = 10;
    inline constexpr int FS_REQUEST_LIMIT = 10;
    inline constexpr bool MAP_PRELOAD = true;

    class TileLoaderOsmMap : public TileLoader {
    public:
        TileLoaderOsmMap(const int request_limit = URL_REQUEST_LIMIT) : TileLoader(std::make_shared<TileSourceUrlOsm>(request_limit, MAP_PRELOAD)) {}
    };

    class TileLoaderArcMap : public TileLoader {
    public:
        TileLoaderArcMap(const int request_limit = URL_REQUEST_LIMIT)
            : TileLoader(std::make_shared<TileSourceUrlArc>(request_limit, MAP_PRELOAD)) {}
    };

    class TileLoaderUrlMap : public TileLoader {
    public:
        TileLoaderUrlMap(const std::string& url_template, const int request_limit = URL_REQUEST_LIMIT)
            : TileLoader(std::make_shared<TileSourceUrlCustom>(request_limit, MAP_PRELOAD, url_template)) {}
    };

    class TileLoaderFsMap : public TileLoader {
    public:
        TileLoaderFsMap(const std::string& dirname, const int request_limit = URL_REQUEST_LIMIT)
            : TileLoader(std::make_shared<TileSourceFsSubDir>(request_limit, MAP_PRELOAD, dirname)) {}
    };
}
