#pragma once

#include "foc2-gui/osm/osm_itile_source.hpp"
#include "foc2-gui/osm/osm_tile_async.hpp"

namespace ImOsm {
    class ITile;
    class ITileSaver;

    class TileSourceAsync : public ITileSource {
    public:
        TileSourceAsync(int requestLimit, bool preload);
        ~TileSourceAsync() override;

        bool hasRequest() override;
        bool hasRequest(int z, int x, int y) override;
        bool canRequest() override;
        bool request(int z, int x, int y) override;

        void waitAll() override;
        bool canTakeAll() override;
        void takeAll(std::vector<std::shared_ptr<ITile>>& tiles) override;
        int takeReady(std::vector<std::shared_ptr<ITile>>& tiles) override;
        bool saveAll(std::shared_ptr<ITileSaver> saver) override;

        bool isPreload() const override {
            return preload;
        }

        int requestLimit() const override {
            return request_limit;
        }

        struct TileData {
            std::atomic_bool& interrupt;
            std::vector<std::byte> blob;
        };

    protected:
        virtual bool receiveTile(int z, int x, int y, TileData& data) = 0;

    private:
        TileAsync::FutureData onHandleRequest(int z, int x, int y);

        std::vector<TileAsync> requests;
        int request_limit = 10;
        bool preload = true;
        std::atomic_bool interrupt;
    };
}
