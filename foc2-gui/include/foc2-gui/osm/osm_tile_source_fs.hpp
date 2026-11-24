#pragma once

#include <filesystem>

#include "foc2-gui/osm/osm_tile_source_async.hpp"

namespace ImOsm {
    class TileSourceFs : public TileSourceAsync {
    public:
        TileSourceFs(int request_limit, bool preload, std::filesystem::path base_path, std::optional<std::string> extension);
        ~TileSourceFs() override = default;

        bool hasTile(int z, int x, int y) const;

        static std::string FileName(int z, int x, int y, std::optional<std::string> extension);
        static std::filesystem::path BasePathDefault();

    protected:
        bool receiveTile(int z, int x, int y, TileData& tileData) override;

        [[nodiscard]] std::filesystem::path basePath() const {
            return base_path;
        }

        [[nodiscard]] virtual std::filesystem::path dirPath(int z, int x, int y) const = 0;

    private:
        std::filesystem::path base_path = BasePathDefault();
        std::optional<std::string> extension;
    };

    class TileSourceFsDir : public TileSourceFs {
    public:
        TileSourceFsDir(int request_limit, bool preload, std::filesystem::path base_path, std::optional<std::string> extension);
        ~TileSourceFsDir() override = default;

        static std::filesystem::path dirPath(std::filesystem::path base_path, [[maybe_unused]] int z, [[maybe_unused]] int x, [[maybe_unused]] int y) {
            return base_path;
        }

    protected:
        [[nodiscard]] std::filesystem::path dirPath(int z, int x, int y) const override;
    };

    class TileSourceFsSubDir : public TileSourceFs {
    public:
        TileSourceFsSubDir(int request_limit, bool preload, std::filesystem::path base_path, std::optional<std::string> extension);
        ~TileSourceFsSubDir() override = default;

        static std::filesystem::path DirPath(const std::filesystem::path& base_path, int z, [[maybe_unused]] int x, [[maybe_unused]] int y);

    protected:
        [[nodiscard]] std::filesystem::path dirPath(int z, int x, int y) const override;
    };
}
