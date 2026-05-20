#pragma once

#include <filesystem>

#include "foc2-gui/osm/osm_itile_saver.hpp"
#include "foc2-gui/osm/osm_tile_source_fs.hpp"

namespace ImOsm {
    class ITile;

    class TileSaver : public ITileSaver {
    public:
        TileSaver();
        ~TileSaver() override;

        [[nodiscard]] bool saveMulti(const std::vector<std::shared_ptr<ITile>>& tiles) const override;
        [[nodiscard]] bool save(const std::shared_ptr<ITile>& tile) const override;

    protected:
        [[nodiscard]] virtual std::filesystem::path dirPath(const std::shared_ptr<ITile>& tile) const = 0;
        [[nodiscard]] virtual std::string fileName(const std::shared_ptr<ITile>& tile) const;
    };

    // -----------------------------------------------------------------------------

    class TileSaverDir : public TileSaver {
    public:
        TileSaverDir();
        TileSaverDir(std::filesystem::path basePath);
        ~TileSaverDir() override;

    protected:
        [[nodiscard]] std::filesystem::path dirPath(const std::shared_ptr<ITile>& tile) const override;

    private:
        std::filesystem::path base_path = TileSourceFs::BasePathDefault();
    };

    // -----------------------------------------------------------------------------

    class TileSaverSubDir : public TileSaver {
    public:
        TileSaverSubDir();
        TileSaverSubDir(std::filesystem::path basePath);
        ~TileSaverSubDir() override;

    protected:
        [[nodiscard]] std::filesystem::path dirPath(const std::shared_ptr<ITile>& tile) const override;

    private:
        std::filesystem::path base_path = TileSourceFs::BasePathDefault();
    };
}
