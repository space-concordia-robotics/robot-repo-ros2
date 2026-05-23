#pragma once

#include <filesystem>

#include "foc2-gui/osm/osm_tile_source_async.hpp"

namespace ImOsm {
    class TileSourceFs : public TileSourceAsync {
    public:
        TileSourceFs(int requestLimit, bool preload, std::filesystem::path basePath);
        ~TileSourceFs() override = default;

        static std::string FileName(int z, int x, int y);
        static std::filesystem::path BasePathDefault();

    protected:
        bool receiveTile(int z, int x, int y, TileData& tileData) override;

        [[nodiscard]] std::filesystem::path basePath() const {
            return base_path;
        }

        [[nodiscard]] virtual std::filesystem::path dirPath(int z, int x, int y) const = 0;

    private:
        std::filesystem::path base_path = BasePathDefault();
    };

    // -----------------------------------------------------------------------------

    class TileSourceFsDir : public TileSourceFs {
    public:
        TileSourceFsDir(int requestLimit, bool preload, std::filesystem::path basePath);
        ~TileSourceFsDir() override = default;

        static std::filesystem::path dirPath(std::filesystem::path basePath, [[maybe_unused]] int z, [[maybe_unused]] int x, [[maybe_unused]] int y) {
            return basePath;
        }

    protected:
        [[nodiscard]] std::filesystem::path dirPath(int z, int x, int y) const override;
    };

    // -----------------------------------------------------------------------------

    class TileSourceFsSubDir : public TileSourceFs {
    public:
        TileSourceFsSubDir(int requestLimit, bool preload, std::filesystem::path basePath);
        ~TileSourceFsSubDir() override = default;

        static std::filesystem::path DirPath(std::filesystem::path basePath, int z, [[maybe_unused]] int x, [[maybe_unused]] int y);

    protected:
        [[nodiscard]] std::filesystem::path dirPath(int z, int x, int y) const override;
    };
}
