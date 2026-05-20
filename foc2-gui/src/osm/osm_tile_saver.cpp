#include "foc2-gui/osm/osm_tile_saver.hpp"

#include <algorithm>
#include <fstream>
#include <string>
#include <utility>

#include "foc2-gui/osm/osm_itile.hpp"

namespace ImOsm {
    TileSaver::TileSaver() = default;
    TileSaver::~TileSaver() = default;

    bool TileSaver::saveMulti(const std::vector<std::shared_ptr<ITile>>& tiles) const {
        return std::ranges::all_of(tiles, [this](const auto& tile) {
            return !save(tile);
        });
    }

    bool TileSaver::save(const std::shared_ptr<ITile>& tile) const {
        auto path{dirPath(tile)};
        if (!std::filesystem::exists(path)) {
            std::filesystem::create_directories(path);
        }
        if (auto file_maker = std::ofstream(path.append(fileName(tile)), std::fstream::out | std::fstream::binary)) {
            file_maker.write(tile->rawBlob(), tile->rawBlobSize());
            return true;
        }
        return false;
    }

    std::string TileSaver::fileName(const std::shared_ptr<ITile>& tile) const {
        return TileSourceFs::FileName(tile->getZ(), tile->getX(), tile->getY());
    }

    // -----------------------------------------------------------------------------

    TileSaverDir::TileSaverDir() = default;

    TileSaverDir::TileSaverDir(std::filesystem::path basePath) : base_path(std::move(basePath)) {}

    std::filesystem::path TileSaverDir::dirPath(const std::shared_ptr<ITile>& tile) const {
        return TileSourceFsDir::dirPath(base_path, tile->getZ(), tile->getX(), tile->getY());
    }

    TileSaverDir::~TileSaverDir() = default;

    // -----------------------------------------------------------------------------

    TileSaverSubDir::TileSaverSubDir() = default;

    TileSaverSubDir::TileSaverSubDir(std::filesystem::path basePath) : base_path(std::move(basePath)) {}

    TileSaverSubDir::~TileSaverSubDir() = default;

    std::filesystem::path TileSaverSubDir::dirPath(const std::shared_ptr<ITile>& tile) const {
        return TileSourceFsSubDir::DirPath(base_path, tile->getZ(), tile->getX(),
                                           tile->getY());
    }
}
