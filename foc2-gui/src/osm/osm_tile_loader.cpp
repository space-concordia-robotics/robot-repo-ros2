#include "foc2-gui/osm/osm_tile_loader.hpp"

#include <algorithm>

#include "foc2-gui/osm/osm_itile.hpp"
#include "foc2-gui/osm/osm_itile_source.hpp"

namespace ImOsm {
    TileLoader::TileLoader(const std::shared_ptr<ITileSource>& source) : source(source) {}

    void TileLoader::beginLoad(int z, int xmin, int xmax, int ymin, int ymax) {
        const auto cond{
            [z, xmin, xmax, ymin, ymax](const std::shared_ptr<ITile>& tile) {
                return !tile->inBounds(z, xmin, xmax, ymin, ymax) || tile->isDummy();
            }
        };
        std::erase_if(tiles, cond);
        source->takeReady(tiles);
    }

    ImTextureID TileLoader::tileAt(int z, int x, int y) {
        const auto cond = [z, x, y](const std::shared_ptr<ITile>& tile) {
            return tile->isTileZXY(z, x, y);
        };

        if (const auto it = std::ranges::find_if(tiles, cond); it != tiles.end()) {
            if (!(*it)->isDummy()) {
                return (*it)->texture();
            }
        }

        if (!source->hasRequest(z, x, y)) {
            source->request(z, x, y);
        }

        return 0;
    }
}
