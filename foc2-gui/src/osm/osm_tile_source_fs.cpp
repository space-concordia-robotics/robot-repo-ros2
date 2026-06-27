#include "foc2-gui/osm/osm_tile_source_fs.hpp"

#include <fstream>
#include <utility>
#include <fmt/format.h>

namespace ImOsm {
    TileSourceFs::TileSourceFs(const int request_limit, const bool preload, std::filesystem::path base_path, std::optional<std::string> extension)
        : TileSourceAsync(request_limit, preload),
          base_path(std::move(base_path)),
          extension(std::move(extension)) {}

    bool TileSourceFs::hasTile(const int z, const int x, const int y) const {
        const auto path = dirPath(z, x, y);
        return std::filesystem::exists(path / FileName(z, x, y, extension));
    }

    std::string TileSourceFs::FileName(const int z, const int x, const int y, std::optional<std::string> extension) {
        return fmt::format("{}-{}-{}.{}", z, x, y, extension.has_value() ? extension.value() : "tile");
    }

    std::filesystem::path TileSourceFs::BasePathDefault() {
        return std::filesystem::current_path() / "tiles";
    }

    bool TileSourceFs::receiveTile(const int z, const int x, const int y, TileData& tileData) {
        const auto path = dirPath(z, x, y);

        auto reader = std::ifstream(path / FileName(z, x, y, extension), std::fstream::binary);

        if (!reader)
            return false;

        reader.seekg(0, std::ios::end);
        const auto size = reader.tellg();

        if (size <= 0)
            return false;

        reader.seekg(0, std::ios::beg);

        tileData.blob.resize(size);

        reader.read(reinterpret_cast<char*>(&tileData.blob[0]), size);

        return true;
    }

    // -----------------------------------------------------------------------------

    TileSourceFsDir::TileSourceFsDir(const int request_limit, const bool preload, std::filesystem::path base_path, std::optional<std::string> extension)
        : TileSourceFs(request_limit, preload, std::move(base_path), std::move(extension)) {}

    std::filesystem::path TileSourceFsDir::dirPath(const int z, const int x, const int y) const {
        return dirPath(basePath(), z, x, y);
    }

    // -----------------------------------------------------------------------------

    TileSourceFsSubDir::TileSourceFsSubDir(const int request_limit, const bool preload, std::filesystem::path base_path, std::optional<std::string> extension)
        : TileSourceFs(request_limit, preload, std::move(base_path), std::move(extension)) {}

    std::filesystem::path TileSourceFsSubDir::DirPath(const std::filesystem::path& base_path, const int z, [[maybe_unused]] int x, [[maybe_unused]] int y) {
        return base_path / std::to_string(z);
    }

    std::filesystem::path TileSourceFsSubDir::dirPath(const int z, const int x, const int y) const {
        return DirPath(basePath(), z, x, y);
    }
}
