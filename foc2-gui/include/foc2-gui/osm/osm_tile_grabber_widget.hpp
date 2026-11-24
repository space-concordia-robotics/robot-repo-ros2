#pragma once

#include <memory>

namespace ImOsm {
    class MapPlot;
    class TileGrabber;

    class TileGrabberWidget {
    public:
        explicit TileGrabberWidget(const std::shared_ptr<MapPlot>& mapPlot);
        ~TileGrabberWidget();

        void loadState(/*const mINI::INIStructure& ini*/);
        void saveState(/*mINI::INIStructure& ini*/) const;
        void paint();

    private:
        std::unique_ptr<TileGrabber> tile_grabber;
        std::shared_ptr<MapPlot> map_plot;

        struct Ui;
        std::unique_ptr<Ui> ui;
    };
}
