#pragma once

#include <memory>

#include "osm_tile_source_caching.hpp"
#include "foc2-gui/osm/osm_tile_loader.hpp"
#include "foc2-gui/osm/osm_tile_source_fs.hpp"
#include "foc2-gui/osm/osm_tile_source_url_impl.hpp"

namespace ImOsm {
    inline constexpr int URL_REQUEST_LIMIT = 10;
    inline constexpr int FS_REQUEST_LIMIT = 10;
    inline constexpr bool MAP_PRELOAD = true;

    class TileLoaderOsmMap : public TileLoader {
    public:
        explicit TileLoaderOsmMap(const int request_limit = URL_REQUEST_LIMIT) : TileLoader(std::make_shared<TileSourceUrlOsm>(request_limit, MAP_PRELOAD)) {}
    };

    class TileLoaderArcMap : public TileLoader {
    public:
        explicit TileLoaderArcMap(const int request_limit = URL_REQUEST_LIMIT)
            : TileLoader(std::make_shared<TileSourceUrlArc>(request_limit, MAP_PRELOAD)) {}
    };

    class TileLoaderUrlMap : public TileLoader {
    public:
        explicit TileLoaderUrlMap(std::string url_template, const int request_limit = URL_REQUEST_LIMIT)
            : TileLoader(std::make_shared<TileSourceUrlCustom>(request_limit, MAP_PRELOAD, std::move(url_template))) {}
    };

    class TileLoaderFsMap : public TileLoader {
    public:
        // TODO 2026-05-28 (Will Free): add parameter for subdir/non-subdir
        explicit TileLoaderFsMap(
            std::filesystem::path base_path,
            std::optional<std::string> extension = std::nullopt,
            const int request_limit = URL_REQUEST_LIMIT
        ) : TileLoader(std::make_shared<TileSourceFsSubDir>(request_limit, MAP_PRELOAD, std::move(base_path), std::move(extension))) {}
    };

    class TileLoaderCachingMap : public TileLoader {
    public:
        // TODO 2026-05-28 (Will Free): add parameter for subdir/non-subdir
        explicit TileLoaderCachingMap(
            std::shared_ptr<TileSourceAsync> remote_source,
            std::filesystem::path base_path,
            std::optional<std::string> extension = std::nullopt,
            const int request_limit = URL_REQUEST_LIMIT
        ) : TileLoader(
            std::make_shared<TileSourceCachingSubDir>(
                request_limit,
                MAP_PRELOAD,
                std::move(base_path),
                std::move(extension),
                std::move(remote_source)
            )
        ) {}
    };
}
