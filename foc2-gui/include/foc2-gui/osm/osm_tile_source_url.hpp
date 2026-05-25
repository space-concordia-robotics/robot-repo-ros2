#pragma once

#include "foc2-gui/osm/osm_tile_source_async.hpp"

namespace ImOsm {
    class TileSourceUrl : public TileSourceAsync {
    public:
        TileSourceUrl(int request_limit, bool preload, std::string user_agent = "curl");
        ~TileSourceUrl() override;

    protected:
        bool receiveTile(int z, int x, int y, TileData& tile_data) override;
        virtual std::string makeUrl(int z, int x, int y) = 0;

    private:
        std::string user_agent = "curl";
    };
}
