#include "foc2-gui/osm/osm_tile_grabber.hpp"

#include "foc2-gui/osm/osm_coords.hpp"
#include "foc2-gui/osm/osm_itile.hpp"
#include "foc2-gui/osm/osm_itile_saver.hpp"
#include "foc2-gui/osm/osm_itile_source.hpp"

namespace ImOsm {
    TileGrabber::TileGrabber(const std::shared_ptr<ITileSource>& source, const std::shared_ptr<ITileSaver>& saver) : source(source), saver(saver) {}

    TileGrabber::~TileGrabber() {}

    void TileGrabber::grab(
        const double minLat, const double maxLat,
        const double minLon, const double maxLon,
        const int minZ, const int maxZ
    ) {
        stopped = false;
        future = std::async(std::launch::async, &TileGrabber::onLaunchGrab, this, minLat, maxLat, minLon, maxLon, minZ, maxZ);
    }

    void TileGrabber::stop() {
        stopped = true;
    }

    TileGrabber::FutureData
    TileGrabber::onLaunchGrab(const double minLat, const double maxLat, const double minLon, const double maxLon, const int minZ, const int maxZ) {
        tile_counter = 0;
        std::vector<std::shared_ptr<ITile>> tiles;
        tiles.reserve(source->requestLimit());
        const FutureData data;

        for (auto z = minZ; z != maxZ + 1; ++z) {
            const auto [start_x, end_x] = minmax_tx(minLon, maxLon, z);
            const auto [start_y, end_y] = minmax_ty(minLat, maxLat, z);
            for (auto x = start_x; x != end_x + 1; ++x) {
                for (auto y = start_y; y != end_y + 1; ++y) {
                    if (stopped)
                        return data;

                    if (!source->canRequest()) {
                        tiles.clear();
                        source->takeAll(tiles);

                        std::ranges::for_each(
                            tiles,
                            [this](const auto& tile) {
                                if (tile->isDummy()) {
                                    source->request(tile->getZ(), tile->getX(), tile->getY());
                                }
                            }
                        );

                        std::erase_if(
                            tiles,
                            [](const auto& tile) {
                                return tile->isDummy();
                            }
                        );

                        source->takeAll(tiles);
                        tile_counter += tiles.size();
                    }
                    source->request(z, x, y);
                }
            }
        }

        tiles.clear();
        source->takeAll(tiles);
        tile_counter += tiles.size();

        return data;
    }
}
