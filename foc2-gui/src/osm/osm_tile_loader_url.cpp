#include "foc2-gui/osm/osm_tile_loader_url.hpp"

#include <algorithm>
#include <chrono>
#include <future>
#include <iostream>
#include <memory>
#include <curl/curl.h>
#include <fmt/format.h>

using namespace std::chrono_literals;

namespace ImOsm::Old {
    TileLoaderUrl::TileLoaderUrl() = default;

    TileLoaderUrl::~TileLoaderUrl() = default;

    void TileLoaderUrl::beginLoad(int z, int xmin, int xmax, int ymin, int ymax) {
        auto rm_cond = [z, xmin, xmax, ymin, ymax](const Tile& tile) {
            const bool c1 = tile.zxy.at(0) != z;
            const bool c2 = tile.zxy.at(1) < xmin || tile.zxy.at(1) > xmax;
            const bool c3 = tile.zxy.at(2) < ymin || tile.zxy.at(2) > ymax;
            const bool c4 = tile.future.valid() && tile.future.wait_for(0s) == std::future_status::ready;
            const bool c5 = tile.texture != nullptr;

            return (c1 || c2 || c3) && (c4 || c5);
        };

        tiles.erase(std::ranges::remove_if(tiles, rm_cond).begin(), tiles.end());

        future_counter = 0;
    }

    ImTextureID TileLoaderUrl::tileAt(const int z, const int x, const int y) {
        const auto it = std::ranges::find(tiles, Tile(std::array{z, x, y}));

        if (it == tiles.end()) {
            if (future_counter++ < future_limit) {
                tiles.insert(
                    it,
                    Tile{
                        .zxy = {z, x, y},
                        .future = std::async(std::launch::async, &TileLoaderUrl::onHandleRequest, this, std::array < int, 3 > ( {
                            z, x, y
                        })),
                        .texture = nullptr
                    }
                );
            }
            return blank_tile.imID();
        }

        if (it->texture) {
            return it->texture.get()->imID();
        }

        if (!it->future.valid()) {
            return blank_tile.imID();
        }

        if (it->future.wait_for(0s) != std::future_status::ready) {
            return blank_tile.imID();
        }

        const auto remote{it->future.get()};

        if (remote.code != CURLE_OK) {
            tiles.erase(it);
            return blank_tile.imID();
        }

        if (texture_counter++ < texture_limit) {
            texture_counter--;
            it->texture = remote.texture;
            return it->texture.get()->imID();
        }

        return blank_tile.imID();
    }

    TileLoaderUrl::Tile::Remote TileLoaderUrl::onHandleRequest(const std::array<int, 3>& zxy) const {
        const auto url = fmt::format("https://{}/{}/{}/{}", tile_provider, zxy[0], zxy[2], zxy[1]);

        Tile::Remote tile;
        CURL* curl = curl_easy_init();
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 1L);
        // curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
        curl_easy_setopt(curl, CURLOPT_USERAGENT, client_name.c_str());
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 1);
        curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 1);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, static_cast<void*>(&tile.blob));
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, onPullResponse);
        tile.code = curl_easy_perform(curl);
        curl_easy_cleanup(curl);

        if (tile.code == CURLE_OK) {
            tile.texture = std::make_shared<TileTexture>(tile_size_px, tile.blob);
        }

        // test for async
        // std::this_thread::sleep_for(1s);

        return tile;
    }

    size_t TileLoaderUrl::onPullResponse(void* data, const size_t size, const size_t nmemb, void* userp) {
        const size_t realsize = size * nmemb;
        auto& blob = *static_cast<std::vector<std::byte>*>(userp);
        auto const* const dataptr = static_cast<std::byte*>(data);

        blob.insert(blob.cend(), dataptr, dataptr + realsize);

        return realsize;
    }
}
