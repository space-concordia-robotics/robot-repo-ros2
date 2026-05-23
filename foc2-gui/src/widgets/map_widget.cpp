#include "foc2-gui/widgets/map_widget.hpp"

#include "foc2-gui/osm/osm_map_plot.hpp"
#include "foc2-gui/osm/osm_rich_map_plot.hpp"
#include "foc2-gui/osm/osm_tile_loader_impl.hpp"

MapWidget::MapWidget(ImApplication& application) : UiWidget(application) {}

void MapWidget::onInit() {
    map_plot = std::make_shared<ImOsm::Rich::RichMapPlot>(std::make_shared<ImOsm::TileLoaderOsmMap>(16));
    robot_marker = std::make_shared<RichRobotMarker>(application);
    map_plot->addItem(robot_marker);
    robot_marker->onInit();
}

void MapWidget::onShutdown() {
    robot_marker->onShutdown();
}

void MapWidget::draw() {
    map_plot->paint();
}
