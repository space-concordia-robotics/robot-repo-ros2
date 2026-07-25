#include "foc2-gui/osm/osm_tile_grabber_url.hpp"

#include <algorithm>
#include <fstream>
#include <future>
#include <sstream>
#include <thread>
#include <fmt/format.h>

#include "foc2-gui/osm/osm_coords.hpp"

using namespace std::chrono_literals;

namespace ImOsm::Old {
    TileGrabberUrl::TileGrabberUrl() {}

    void TileGrabberUrl::grab(int minZ, int maxZ, float minLat, float maxLat, float minLon, float maxLon) {
        grab_future = std::async(std::launch::async, &TileGrabberUrl::onLaunchGrab,
                                 this, minZ, maxZ, minLat, maxLat, minLon, maxLon);
    }

    TileGrabberUrl::GrabData TileGrabberUrl::onLaunchGrab(
        const int zmin, const int zmax,
        const float xmin, const float xmax,
        const float ymin, const float ymax
    ) {
        auto rm_cond = [this](Tile& tile) {
            const bool ready{tile.future.wait_for(0s) == std::future_status::ready};
            if (ready) {
                saveFile(tile);
            }
            return ready;
        };

        GrabData data;
        for (auto z = zmin; z != zmax + 1; ++z) {
            const auto tilesNum = 1 << z;
            const auto [start_x, end_x] = std::minmax(std::clamp(lon2tx(xmin, z), 0, tilesNum - 1), std::clamp(lon2tx(xmax, z), 0, tilesNum - 1));
            const auto [start_y, end_y] = std::minmax(std::clamp(lat2ty(ymin, z), 0, tilesNum - 1), std::clamp(lat2ty(ymax, z), 0, tilesNum - 1));

            // const auto total{(maxTX - minTX + 1) * (maxTY - minTY + 1)};
            for (auto x = start_x; x != end_x + 1; ++x) {
                for (auto y = start_y; y != end_y + 1; ++y) {
                    data.tiles.push_back(
                        Tile{
                            .zxy    = {z, x, y},
                            .future = std::async(std::launch::async, &TileGrabberUrl::onHandleRequest, this, std::array<int, 3>({
                                                     z, x, y
                                                 }))
                        }
                    );

                    while (data.tiles.size() > static_cast<std::size_t>(max_parallel_grab)) {
                        std::erase_if(data.tiles, rm_cond);
                        std::this_thread::sleep_for(1s);
                    }
                }
            }
        }

        while (data.tiles.size() > 0) {
            std::erase_if(data.tiles, rm_cond);
            std::this_thread::sleep_for(1s);
        }

        return data;
    }

    TileGrabberUrl::Tile::Remote TileGrabberUrl::onHandleRequest(const std::array<int, 3>& zxy) const {
        const auto url = fmt::format("https://{}/{}/{}/{}.{}", tile_provider, zxy[0], zxy[1], zxy[2], tile_extension);

        // TODO 2026-05-19 (Will Free): replace curl with something a bit nicer to use

        Tile::Remote tile;
        CURL* curl = curl_easy_init();
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 1L);
        // curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
        curl_easy_setopt(curl, CURLOPT_USERAGENT, client_name.c_str());
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 1);
        curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 1);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, static_cast<void*>(&tile));
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, onPullResponse);
        tile.code = curl_easy_perform(curl);
        curl_easy_cleanup(curl);

        return tile;
    }

    size_t TileGrabberUrl::onPullResponse(void* data, const size_t size, const size_t nmemb, void* userp) {
        const size_t realsize = size * nmemb;
        auto& [blob, _] = *static_cast<Tile::Remote*>(userp);
        auto const* const dataptr = static_cast<std::byte*>(data);
        blob.insert(blob.cend(), dataptr, dataptr + realsize);

        return realsize;
    }

    void TileGrabberUrl::saveFile(Tile& tile) const {
        std::ostringstream pathmaker;
        pathmaker << tile_save_path << '/' << tile.zxy[0] << '/' << tile.zxy[1];
        if (!std::filesystem::exists(pathmaker.str())) {
            std::filesystem::create_directories(pathmaker.str());
        }
        pathmaker << '/' << tile.zxy[0] << '-' << tile.zxy[1] << '-' << tile.zxy[2]
            << tile_extension;
        std::ofstream stream(pathmaker.str().c_str(),
                             std::fstream::out | std::fstream::binary);
        if (stream) {
            const auto blob{tile.future.get().blob};
            stream.write(reinterpret_cast<const char*>(blob.data()), blob.size());
        }
    }
}
