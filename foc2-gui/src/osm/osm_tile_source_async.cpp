#include "foc2-gui/osm/osm_tile_source_async.hpp"

#include <algorithm>

#include "foc2-gui/osm/osm_itile_saver.hpp"
#include "foc2-gui/osm/osm_tile.hpp"
#include "foc2-gui/osm/osm_tile_async.hpp"

namespace ImOsm {
    using namespace std::chrono_literals;

    TileSourceAsync::TileSourceAsync(const int requestLimit, const bool preload) : request_limit(requestLimit), preload(preload) {}

    TileSourceAsync::~TileSourceAsync() {
        interrupt = true;
    }

    bool TileSourceAsync::hasRequest() {
        return !requests.empty();
    }

    bool TileSourceAsync::hasRequest(const int z, const int x, const int y) {
        return std::ranges::find(requests, TileAsync{z, x, y, {}}) != requests.end();
    }

    bool TileSourceAsync::canRequest() {
        return requests.size() < request_limit;
    }

    bool TileSourceAsync::request(int z, int x, int y) {
        if (canRequest()) {
            requests.emplace_back(
                TileAsync{
                    .z = z,
                    .x = x,
                    .y = y,
                    .future = std::async(std::launch::async, &TileSourceAsync::onHandleRequest, this, z, x, y)
                }
            );
            return true;
        }
        return false;
    }

    void TileSourceAsync::waitAll() {
        while (!canTakeAll()) {
            std::this_thread::sleep_for(1s);
        }
    }

    bool TileSourceAsync::canTakeAll() {
        return std::ranges::all_of(
            requests,
            [](const TileAsync& tile) {
                return tile.ready();
            }
        );
    }

    void TileSourceAsync::takeAll(std::vector<std::shared_ptr<ITile>>& tiles) {
        waitAll();
        for (auto& request : requests) {
            tiles.push_back(request.future.get().tile);
        }
        requests.clear();
    }

    int TileSourceAsync::takeReady(std::vector<std::shared_ptr<ITile>>& tiles) {
        int taken = 0;
        for (auto it{requests.begin()}; it != requests.end();) {
            if (it->ready()) {
                tiles.push_back(it->future.get().tile);
                it = requests.erase(it);
                ++taken;
            } else {
                ++it;
            }
        }
        return taken;
    }

    bool TileSourceAsync::saveAll(const std::shared_ptr<ITileSaver> saver) {
        std::vector<std::shared_ptr<ITile>> tiles;
        takeAll(tiles);
        return saver->saveMulti(tiles);
    }

    TileAsync::FutureData TileSourceAsync::onHandleRequest(int z, int x, int y) {
        auto tileData = TileData(interrupt);
        TileAsync::FutureData futureData;
        if (receiveTile(z, x, y, tileData)) {
            futureData.tile = std::make_shared<Tile>(z, x, y, tileData.blob, preload);
        } else {
            futureData.tile = std::make_shared<TileDummy>(z, x, y);
        }
        return futureData;
    }
}
