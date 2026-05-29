#include "foc2-gui/osm/osm_tile_saver.hpp"

#include <algorithm>
#include <fstream>
#include <string>
#include <utility>
#include <fmt/base.h>

#include "foc2-gui/osm/osm_itile.hpp"

namespace ImOsm {
    TileSaver::TileSaver(std::optional<std::string> extension) : extension(std::move(extension)) {}

    TileSaver::TileSaver(std::filesystem::path base_path, std::optional<std::string> extension)
        : base_path(std::move(base_path)),
          extension(std::move(extension)) {}

    TileSaver::~TileSaver() = default;

    bool TileSaver::saveMulti(const std::vector<std::shared_ptr<ITile>>& tiles) const {
        auto failed = false;
        for (auto&& tile : tiles) {
            failed |= !save(tile);
        }
        return !failed;
    }

    bool TileSaver::save(const std::shared_ptr<ITile>& tile) const {
        auto path = dirPath(tile);

        if (!std::filesystem::exists(path)) {
            std::filesystem::create_directories(path);
        }

        const auto filename = path / fileName(tile);
        if (auto file_maker = std::ofstream(filename, std::fstream::out | std::fstream::binary)) {
            file_maker.write(tile->rawBlob(), tile->rawBlobSize());
            return true;
        } else {
            fmt::println("failed  to open file, {}", filename.string());
        }

        return false;
    }

    std::string TileSaver::fileName(const std::shared_ptr<ITile>& tile) const {
        return TileSourceFs::FileName(tile->getZ(), tile->getX(), tile->getY(), extension);
    }

    // -----------------------------------------------------------------------------

    TileSaverDir::TileSaverDir(std::optional<std::string> extension) : TileSaver(std::move(extension)) {}

    TileSaverDir::TileSaverDir(
        std::filesystem::path base_path,
        std::optional<std::string> extension
    ) : TileSaver(std::move(base_path), std::move(extension)) {}

    std::filesystem::path TileSaverDir::dirPath(const std::shared_ptr<ITile>& tile) const {
        return TileSourceFsDir::dirPath(basePath(), tile->getZ(), tile->getX(), tile->getY());
    }

    TileSaverDir::~TileSaverDir() = default;

    // -----------------------------------------------------------------------------

    TileSaverSubDir::TileSaverSubDir(std::optional<std::string> extension) : TileSaver(std::move(extension)) {}

    TileSaverSubDir::TileSaverSubDir(
        std::filesystem::path base_path,
        std::optional<std::string> extension
    ) : TileSaver(std::move(base_path), std::move(extension)) {}

    TileSaverSubDir::~TileSaverSubDir() = default;

    std::filesystem::path TileSaverSubDir::dirPath(const std::shared_ptr<ITile>& tile) const {
        return TileSourceFsSubDir::DirPath(basePath(), tile->getZ(), tile->getX(), tile->getY());
    }
}
