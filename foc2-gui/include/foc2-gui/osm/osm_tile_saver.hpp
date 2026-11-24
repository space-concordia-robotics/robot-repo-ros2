#pragma once

#include <filesystem>

#include "foc2-gui/osm/osm_itile_saver.hpp"
#include "foc2-gui/osm/osm_tile_source_fs.hpp"

namespace ImOsm {
    class ITile;

    class TileSaver : public ITileSaver {
    public:
        explicit TileSaver(std::optional<std::string> extension);
        TileSaver(std::filesystem::path base_path, std::optional<std::string> extension);
        ~TileSaver() override;

        [[nodiscard]] bool saveMulti(const std::vector<std::shared_ptr<ITile>>& tiles) const override;
        [[nodiscard]] bool save(const std::shared_ptr<ITile>& tile) const override;

    protected:
        [[nodiscard]] virtual std::filesystem::path dirPath(const std::shared_ptr<ITile>& tile) const = 0;

        [[nodiscard]] virtual std::string fileName(const std::shared_ptr<ITile>& tile) const;

        [[nodiscard]] std::filesystem::path basePath() const {
            return base_path;
        }

    private:
        std::filesystem::path base_path = TileSourceFs::BasePathDefault();
        std::optional<std::string> extension;
    };

    class TileSaverDir : public TileSaver {
    public:
        explicit TileSaverDir(std::optional<std::string> extension);
        TileSaverDir(std::filesystem::path base_path, std::optional<std::string> extension);
        ~TileSaverDir() override;

    protected:
        [[nodiscard]] std::filesystem::path dirPath(const std::shared_ptr<ITile>& tile) const override;
    };

    class TileSaverSubDir : public TileSaver {
    public:
        explicit TileSaverSubDir(std::optional<std::string> extension);
        TileSaverSubDir(std::filesystem::path base_path, std::optional<std::string> extension);
        ~TileSaverSubDir() override;

    protected:
        [[nodiscard]] std::filesystem::path dirPath(const std::shared_ptr<ITile>& tile) const override;
    };
}
