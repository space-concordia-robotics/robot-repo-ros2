#include "foc2-gui/osm/osm_tile_source_caching.hpp"

#include "foc2-gui/osm/osm_itile.hpp"

namespace ImOsm {
    TileAsync::FutureData TileSourceCaching::onHandleRequest(int z, int x, int y) {
        if (file_source->hasTile(z, x, y))
            return file_source->onHandleRequest(z, x, y);

        const auto result = remote_source->onHandleRequest(z, x, y);

        // TODO 2026-05-28 (Will Free): handle error case
        if (!result.tile->isDummy())
            [[maybe_unused]] const auto _ = tile_saver->save(result.tile);

        return result;
    }
}
