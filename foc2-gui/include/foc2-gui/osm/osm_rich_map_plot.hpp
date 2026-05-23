#pragma once

#include <memory>
#include <vector>

#include "foc2-gui/osm/osm_map_plot.hpp"

namespace ImOsm::Rich {
    class IRichItem;

    class RichMapPlot : public MapPlot {
    public:
        RichMapPlot();
        RichMapPlot(const std::shared_ptr<ITileLoader>& loader);
        ~RichMapPlot() override;

        void paintOverMap() override;
        void loadState(/*const mINI::INIStructure& ini*/);
        void saveState(/*mINI::INIStructure& ini*/) const;
        void addItem(const std::weak_ptr<IRichItem>& item);

    private:
        std::vector<std::weak_ptr<IRichItem>> items;
    };
}
