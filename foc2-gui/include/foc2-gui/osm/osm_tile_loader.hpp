#pragma once

#include <memory>
#include <vector>

#include "foc2-gui/osm/osm_itile_loader.hpp"

namespace ImOsm {
    class ITile;
    class ITileSource;

    class TileLoader : public ITileLoader {
    public:
        explicit TileLoader(const std::shared_ptr<ITileSource>& source);
        ~TileLoader() override = default;

        void beginLoad(int z, int xmin, int xmax, int ymin, int ymax) override;
        ImTextureID tileAt(int z, int x, int y) override;

        int getTileCount() const override {
            return tiles.size();
        }

        void endLoad() override {}

    private:
        std::shared_ptr<ITileSource> source;
        std::vector<std::shared_ptr<ITile>> tiles;
    };
}
