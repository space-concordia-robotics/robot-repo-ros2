#pragma once

#include <atomic>
#include <future>
#include <memory>

namespace ImOsm {
    class ITileSource;
    class ITileSaver;

    class TileGrabber {
    public:
        TileGrabber(const std::shared_ptr<ITileSource>& source,
                    const std::shared_ptr<ITileSaver>& saver);
        ~TileGrabber();

        void grab(double minLat, double maxLat, double minLon, double maxLon, int minZ, int maxZ);

        void stop();

        int tileCounter() const {
            return tile_counter.load();
        }

    private:
        struct FutureData {};

        FutureData onLaunchGrab(double minLat, double maxLat, double minLon, double maxLon, int minZ, int maxZ);

        std::shared_ptr<ITileSource> source;
        std::shared_ptr<ITileSaver> saver;
        std::future<FutureData> future;
        std::atomic_int tile_counter = 0;
        std::atomic_bool stopped = false;
    };
}
