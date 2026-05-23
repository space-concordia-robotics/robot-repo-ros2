#include "foc2-gui/osm/osm_tile_source_fs.hpp"

#include <fstream>
#include <utility>
#include <fmt/format.h>

namespace ImOsm {
    TileSourceFs::TileSourceFs(const int requestLimit, const bool preload, std::filesystem::path basePath)
        : TileSourceAsync(requestLimit, preload),
          base_path(std::move(basePath)) {}

    std::string TileSourceFs::FileName(const int z, const int x, const int y) {
        return fmt::format("{}-{}-{}", z, x, y);
    }

    std::filesystem::path TileSourceFs::BasePathDefault() {
        return std::filesystem::current_path().append("tiles");
    }

    bool TileSourceFs::receiveTile(int z, int x, int y, TileData& tileData) {
        auto path = dirPath(z, x, y);
        auto reader = std::ifstream(path.append(FileName(z, x, y)), std::fstream::in | std::fstream::binary | std::ios::ate);
        if (auto pos = reader.tellg(); reader && pos > 0) {
            tileData.blob.resize(pos);
            reader.seekg(0, std::ios::beg);
            reader.read(reinterpret_cast<char*>(&tileData.blob[0]), pos);
            return true;
        }
        return false;
    }

    // -----------------------------------------------------------------------------

    TileSourceFsDir::TileSourceFsDir(const int requestLimit, const bool preload, std::filesystem::path basePath)
        : TileSourceFs(requestLimit, preload, std::move(basePath)) {}

    std::filesystem::path TileSourceFsDir::dirPath(const int z, const int x, const int y) const {
        return dirPath(basePath(), z, x, y);
    }

    // -----------------------------------------------------------------------------

    TileSourceFsSubDir::TileSourceFsSubDir(const int requestLimit, const bool preload, std::filesystem::path basePath)
        : TileSourceFs(requestLimit, preload, std::move(basePath)) {}

    std::filesystem::path TileSourceFsSubDir::DirPath(std::filesystem::path basePath, const int z, [[maybe_unused]] int x, [[maybe_unused]] int y) {
        return basePath.append(std::to_string(z));
    }

    std::filesystem::path TileSourceFsSubDir::dirPath(const int z, const int x, const int y) const {
        return DirPath(basePath(), z, x, y);
    }
}
