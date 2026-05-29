#pragma once

#include <filesystem>

#include "osm_tile_saver.hpp"
#include "osm_tile_source_fs.hpp"
#include "foc2-gui/osm/osm_tile_source_async.hpp"

namespace ImOsm {
    class TileSourceCaching : public TileSourceAsync {
    public:
        TileSourceCaching(
            const int request_limit,
            const bool preload,
            std::shared_ptr<TileSourceFs> file_source,
            std::shared_ptr<TileSaver> tile_saver,
            std::shared_ptr<TileSourceAsync> remote_source
        ) : TileSourceAsync(request_limit, preload),
            file_source(std::move(file_source)),
            tile_saver(std::move(tile_saver)),
            remote_source(std::move(remote_source)) {}

        ~TileSourceCaching() override = default;

        TileAsync::FutureData onHandleRequest(int z, int x, int y) override;

    protected:
        // should not be used.
        bool receiveTile(int z, int x, int y, TileData& data) override {
            return true;
        }

    private:
        std::shared_ptr<TileSourceFs> file_source;
        std::shared_ptr<TileSaver> tile_saver;
        std::shared_ptr<TileSourceAsync> remote_source;
    };

    // -----------------------------------------------------------------------------

    class TileSourceCachingDir : public TileSourceCaching {
    public:
        TileSourceCachingDir(
            int request_limit,
            bool preload,
            std::filesystem::path base_path,
            std::string extension,
            std::shared_ptr<TileSourceAsync> remote_source
        );
        ~TileSourceCachingDir() override = default;
    };

    // -----------------------------------------------------------------------------

    class TileSourceCachingSubDir : public TileSourceCaching {
    public:
        TileSourceCachingSubDir(
            int request_limit,
            bool preload,
            std::filesystem::path base_path,
            std::optional<std::string> extension,
            std::shared_ptr<TileSourceAsync> remote_source
        ) : TileSourceCaching(
            request_limit,
            preload,
            std::make_shared<TileSourceFsSubDir>(request_limit, preload, base_path, extension),
            std::make_shared<TileSaverSubDir>(base_path, extension),
            std::move(remote_source)
        ) {}

        ~TileSourceCachingSubDir() override = default;
    };
}
