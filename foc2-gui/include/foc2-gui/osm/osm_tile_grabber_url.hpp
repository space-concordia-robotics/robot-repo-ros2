#pragma once

#include <array>
#include <filesystem>
#include <future>
#include <memory>
#include <string>
#include <vector>
#include <curl/curl.h>

namespace ImOsm::Old {
    class TileGrabberUrl {
    public:
        TileGrabberUrl();

        void grab(int minZ, int maxZ, float minLat, float maxLat, float minLon, float maxLon);

    private:
        std::string client_name = "foc2-gui (test app)";
        std::string tile_provider = "a.tile.openstreetmap.org";
        std::string tile_extension = "png";
        std::string tile_save_path = std::filesystem::current_path().string() + "/tiles";
        int max_parallel_grab = 4;

        struct Tile {
            struct Remote {
                std::vector<std::byte> blob;
                CURLcode code = CURLE_OK;
            };

            std::array<int, 3> zxy = {0, 0, 0};
            std::future<Remote> future;
        };

        struct GrabData {
            std::vector<Tile> tiles;
        };

        std::future<GrabData> grab_future;

        [[nodiscard]] GrabData onLaunchGrab(int zmin, int zmax, float xmin, float xmax, float ymin, float ymax);
        [[nodiscard]] Tile::Remote onHandleRequest(const std::array<int, 3>& zxy) const;
        static size_t onPullResponse(void* data, size_t size, size_t nmemb, void* userp);

        void saveFile(Tile& tile) const;
    };
};
