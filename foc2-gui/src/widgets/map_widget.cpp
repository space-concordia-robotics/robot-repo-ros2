#include "foc2-gui/widgets/map_widget.hpp"

#include "foc2-gui/resources.hpp"
#include "foc2-gui/osm/osm_map_plot.hpp"
#include "foc2-gui/osm/osm_rich_map_plot.hpp"
#include "foc2-gui/osm/osm_tile_loader_impl.hpp"
#include "foc2-gui/osm/osm_tile_source_caching.hpp"

std::optional<std::filesystem::path> cacheDirectory(const std::string& name) {
    // TODO 2026-05-28 (Will Free): support like osx/windows at some point.
    if (const char* xdg = std::getenv("XDG_CACHE_HOME"))
        return std::filesystem::path(xdg) / name;
    else if (const char* home = std::getenv("HOME"))
        return std::filesystem::path(home) / ".cache" / name;
    else
        return std::nullopt;
}

MapWidget::MapWidget(ImApplication& application)
    : UiWidget(application) {}

void MapWidget::onInit() {
    std::shared_ptr<ImOsm::ITileLoader> tile_loader;

    if (const auto cache_dir = cacheDirectory(FOC2_PACKAGE_NAME); !cache_dir) {
        logger.error("Could not find $XDG_CACHE_HOME or $HOME. you may be using an unsupported operating system. not caching maps.");
        tile_loader = std::make_shared<ImOsm::TileLoaderArcMap>(16);
    } else {
        const auto map_cache = cache_dir.value() / "maps";

        tile_loader = std::make_shared<ImOsm::TileLoaderCachingMap>(
            std::make_shared<ImOsm::TileSourceUrlArc>(4, true),
            map_cache,
            "jpeg",
            16
        );
    }

    map_plot = std::make_shared<ImOsm::Rich::RichMapPlot>(tile_loader);
    robot_marker = std::make_shared<RichRobotMarker>(application);
    gps_marker = std::make_shared<RichGPSMarker>(application);
    map_plot->addItem(robot_marker);
    map_plot->addItem(gps_marker);
    robot_marker->onInit();
    gps_marker->onInit();
}

void MapWidget::onShutdown() {
    robot_marker->onShutdown();
    gps_marker->onShutdown();
}

void MapWidget::draw() {
    map_plot->paint();
}
