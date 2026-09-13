#pragma once

#include <array>
#include <future>
#include <list>
#include <memory>
#include <curl/curl.h>

#include "foc2-gui/osm/osm_itile_loader.hpp"
#include "foc2-gui/osm/osm_tile_texture.hpp"

namespace ImOsm::Old {
    class TileLoaderUrl : public ITileLoader {
    public:
        TileLoaderUrl();
        ~TileLoaderUrl() override;

        void beginLoad(int z, int xmin, int xmax, int ymin, int ymax) override;
        ImTextureID tileAt(int z, int x, int y) override;

        [[nodiscard]] int getTileCount() const override {
            return tiles.size();
        }

        void setTileProvider(const std::string& url) {
            tile_provider = url;
        }

        [[nodiscard]] const std::string& getTileProvider() const {
            return tile_provider;
        }

        void setTileExtension(const std::string& ext) {
            tile_extension = ext;
        }

        [[nodiscard]] const std::string& getTileExtension() const {
            return tile_extension;
        }

        void setClientName(const std::string& name) {
            client_name = name;
        }

        [[nodiscard]] const std::string& clientName() const {
            return client_name;
        }

    private:
        struct Tile {
            struct Remote {
                std::vector<std::byte> blob;
                std::shared_ptr<TileTexture> texture;
                CURLcode code = CURLE_OK;
            };

            std::array<int, 3> zxy = {0, 0, 0};
            std::future<Remote> future;
            std::shared_ptr<TileTexture> texture = nullptr;

            bool operator==(const Tile& other) const {
                return this->zxy == other.zxy;
            }
        };

        std::string client_name = "foc2-gui (test app)";
        std::string tile_provider = "server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile";
        std::string tile_extension = ".png";

        const int tile_size_px = 256;
        TileTexture blank_tile = TileTexture(tile_size_px, TextureColor::Slate);
        std::list<Tile> tiles;
        const int future_limit = 4;
        const int texture_limit = 1;
        int texture_counter = 0;
        int future_counter = 0;

        Tile::Remote onHandleRequest(const std::array<int, 3>& zxy) const;

        static size_t onPullResponse(void* data, size_t size, size_t nmemb, void* userp);
    };
}
