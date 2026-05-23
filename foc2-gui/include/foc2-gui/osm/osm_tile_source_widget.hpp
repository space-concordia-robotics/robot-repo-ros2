#pragma once

#include <memory>

namespace ImOsm {
    class MapPlot;
    class ITileLoader;

    class TileSourceWidget {
    public:
        TileSourceWidget(const std::shared_ptr<MapPlot>& map_plot);
        ~TileSourceWidget();

        void loadState(/*const mINI::INIStructure& ini*/);
        void saveState(/*mINI::INIStructure& ini*/) const;
        void paint();

    private:
        void updateTileLoader();

        std::shared_ptr<MapPlot> map_plot;
        std::shared_ptr<ITileLoader> tile_loader;

        struct Ui;
        std::unique_ptr<Ui> ui;
    };
}
